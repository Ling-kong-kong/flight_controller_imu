"""BMM150 diagnostic protocol, capture/replay, and real C state-machine tests."""
import csv
import ctypes
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import imu_capture as imu
import mag_capture as mag


def sample(seq=1, **changes):
    values = dict(sequence=seq, timestamp_us=seq*40000, x_raw=123, y_raw=-456,
                  z_raw=500, rhall=12000, x_ut=12.5, y_ut=-32.0, z_ut=35.0,
                  flags=15, address=16, chip_id=50, state=6, error=0, rep_xy=4,
                  rep_z=7, bus_errors=0, invalid_samples=0, not_ready=3,
                  tx_overwrites=0, read_us=1100)
    values.update(changes)
    return imu.MagFrame(**values)


class MagHostTests(unittest.TestCase):
    def test_mixed_split_corrupt_resync(self):
        a = imu.PairedFrame(0, 100000, 0, 0, 1365, 0, 0, 0, 100, 100)
        b = imu.PairedFrame(1, 101000, 0, 0, 1365, 0, 0, 0, 100, 100,
                            protocol_version=3, qw=0.0, qx=0.0, qy=0.0, qz=0.0,
                            roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)
        ack = imu.ControlReply(1, 0, 100000, 0)
        bad = bytearray(sample().encode()); bad[30] ^= 1
        data = a.encode()+sample().encode()+ack.encode()+bytes(bad)+b.encode()+sample(2).encode()
        for n in (1, 2, 7, 71, 72, 73, 4096):
            p = imu.StreamParser(); frames=[]; records=[]
            for offset in range(0, len(data), n):
                frames += p.feed(data[offset:offset+n]); records += p.records
            self.assertEqual(frames, [a,b])
            self.assertEqual([r for r in records if isinstance(r, imu.MagFrame)], [sample(),sample(2)])
            self.assertEqual(p.mag_frames, 2); self.assertEqual(p.crc_errors, 1)
            self.assertLess(len(p.buffer), p.BUFFER_LIMIT)

    def test_invalid_csv_and_replay(self):
        data = b''.join(f.encode() for f in (sample(), sample(2, flags=30),
                       sample(3, x_raw=-4096), sample(4, x_ut=float('nan')), sample(5)))
        outputs=[]
        for chunk in (1, 4096):
            out=io.StringIO(); c=mag.Capture(csv_file=out)
            for offset in range(0,len(data),chunk): c.feed(data[offset:offset+chunk])
            rows=list(csv.DictReader(io.StringIO(out.getvalue())))
            self.assertEqual([r['valid'] for r in rows], ['1','0','0','0','1'])
            self.assertTrue(all(rows[i]['x_ut']=='' and rows[i]['norm_ut']=='' for i in (1,2,3)))
            self.assertEqual(c.summary()['valid_samples'],2)
            json.dumps(c.summary(), allow_nan=False)
            outputs.append(out.getvalue())
        self.assertEqual(*outputs)

    def test_last_invalid_nan_is_json_safe(self):
        capture=mag.Capture()
        capture.feed(sample(x_ut=float("nan")).encode())
        self.assertIsNone(capture.summary()["latest"]["x_ut"])
        json.dumps(capture.summary(),allow_nan=False)

    def test_gui_skips_magnetic_records(self):
        spec=importlib.util.spec_from_file_location('mag_gui_protocol',ROOT/'viewer/imu_viewer/protocol.py')
        m=importlib.util.module_from_spec(spec); sys.modules[spec.name]=m; spec.loader.exec_module(m)
        p=m.Parser(); ack=m.ControlReply(1,0,40000,1)
        a=m.Frame(0,100000,0,0,0,0,0,0,100,100)
        self.assertEqual(p.feed(sample().encode()+ack.encode()+a.encode()),[a])
        self.assertEqual(p.records,[ack,a]); self.assertEqual(p.stats['mag_frames'],1)
        self.assertEqual(p.stats['header_errors'],0)

    def test_large_bounded_input_and_invalid_flags(self):
        p=imu.StreamParser(); self.assertFalse(p.feed(sample().encode()*20000))
        self.assertEqual(p.mag_frames,20000); self.assertLess(len(p.buffer),4096)
        for changes in ({'flags':14},{'flags':31},{'error':1},{'rhall':0},{'chip_id':0},
                        {'rep_z':14},{'z_raw':-16384},{'state':0},{'rhall':16384},
                        {'x_raw':4096},{'y_raw':-4097},{'z_raw':16384}):
            self.assertFalse(sample(**changes).valid)


if os.name=='nt':
    os.environ['PATH']=r'D:\msys64\ucrt64\bin'+os.pathsep+os.environ['PATH']

@unittest.skipUnless(shutil.which('cc'), 'native C compiler required')
class MagNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(prefix='mag-native-'); cls.index=0
        cls.library=Path(cls.tmp.name)/'mag.so'
        cmd=[shutil.which('cc'),'-std=c11','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',
             '-DBMM150_USE_FLOATING_POINT','-I',str(ROOT/'tests/native'),
             '-I',str(ROOT/'firmware/include'),'-I',str(ROOT/'third_party/bmm150'),
             str(ROOT/'tests/native/mag_harness.c'),str(ROOT/'firmware/src/mag.c'),
             str(ROOT/'third_party/bmm150/bmm150.c'),'-lm','-o',str(cls.library)]
        subprocess.run(cmd,check=True,capture_output=True,text=True)

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def setUp(self):
        type(self).index+=1; path=Path(self.tmp.name)/f'{self.index}.so'
        shutil.copy2(self.library,path); self.c=ctypes.CDLL(str(path))
        self.c.mock_frame.restype=ctypes.POINTER(ctypes.c_uint8)
        self.c.mock_compensate.restype=ctypes.c_bool
        self.c.mock_init()

    def tearDown(self):
        if os.name=='nt':
            import _ctypes
            _ctypes.FreeLibrary(self.c._handle)

    def frames(self):
        p=imu.StreamParser()
        p.feed(b''.join(bytes(self.c.mock_frame(i)[:72]) for i in range(self.c.mock_count())))
        self.assertEqual(p.crc_errors,0)
        return p.records

    def test_normal_configuration_ready_and_crc(self):
        self.c.mock_run(1000000)
        frames=self.frames(); self.assertTrue(24 <= len(frames) <= 26)
        self.assertTrue(all(f.valid for f in frames))
        self.assertEqual([f.sequence for f in frames],list(range(len(frames))))
        self.assertEqual(frames[-1].bus_errors,0); self.assertGreater(frames[-1].not_ready,0)
        self.assertEqual((frames[0].x_raw,frames[0].y_raw,frames[0].z_raw),(123,-456,500))
        raw=bytes(self.c.mock_frame(0)[:72])
        self.assertEqual(raw,frames[0].encode())
        self.assertEqual(struct.unpack_from('<I',raw,68)[0], zlib.crc32(raw[:68]))

    def test_conversion_during_unready_burst_does_not_publish_old_vector(self):
        # Unique X tags expose stale shadow data; do not deduplicate real values.
        self.c.mock_faults(32)
        self.c.mock_run(2000000)
        frames=self.frames()
        self.assertTrue(all(f.valid for f in frames))
        tags=[f.x_raw for f in frames]
        self.assertEqual(tags, sorted(set(tags)), "old shadow vector published twice")
        self.assertTrue(48 <= len(frames) <= 50)
        self.assertTrue(all(b.timestamp_us-a.timestamp_us >= 25000
                            for a,b in zip(frames,frames[1:])))

    def test_identical_new_measurements_are_not_deduplicated(self):
        self.c.mock_run(2000000)
        frames=self.frames()
        self.assertTrue(48 <= len(frames) <= 50)
        self.assertTrue(all(f.valid and f.x_raw==123 for f in frames))
        # Reading ready acknowledges it; the following burst still is valid.
        self.assertTrue(all(f.flags & 2 for f in frames))

    def test_failures_limited_retry_then_recover(self):
        for fault,error in ((1,1),(4,2),(8,4),(16,3)):
            # Each loop moves to next one-second retry, no busy-loop fault spam.
            self.c.mock_faults(fault); before=self.c.mock_count(); self.c.mock_run(1100000)
            frames=self.frames()[before:]; self.assertTrue(frames)
            self.assertTrue(all(not f.valid and f.error==error for f in frames))
            self.assertLessEqual(len(frames),2)
        self.c.mock_faults(0); self.c.mock_run(1100000)
        self.assertTrue(self.frames()[-1].valid)

    def test_no_drdy_does_not_fabricate_samples(self):
        self.c.mock_faults(2); self.c.mock_run(900000)
        frames=self.frames(); self.assertEqual(len(frames),1)
        self.assertFalse(frames[0].valid); self.assertEqual(frames[0].error,5)
        self.assertGreater(frames[0].not_ready,0)

    def test_compensation_reference_and_sentinels(self):
        def compensate(x,y,z,h):
            data=struct.pack('<HHHH',(x<<3)&65535,(y<<3)&65535,(z<<1)&65535,(h<<2)&65535)
            buf=(ctypes.c_uint8*8).from_buffer_copy(data)
            raw=(ctypes.c_int16*3)(); hall=ctypes.c_uint16(); ut=(ctypes.c_float*3)()
            valid=self.c.mock_compensate(buf,raw,ctypes.byref(hall),ut)
            self.assertEqual(bytes(buf),data)
            self.assertEqual(list(raw),[x,y,z]); self.assertEqual(hall.value,h)
            return valid,list(ut)
        valid,values=compensate(123,-456,500,12000); self.assertTrue(valid)
        r=25000*16384/12000-16384
        xy_poly=(-3*r*r/268435456 + r*40/16384 + 256)
        expected=[(123*(xy_poly*153/8192)-40)/16,
                  (-456*(xy_poly*168/8192)+48)/16,
                  ((500+1000)*131072-(-100)*(12000-25000))/((-2000+30000*12000/32768)*64)]
        for actual,want in zip(values,expected): self.assertAlmostEqual(actual,want,places=4)
        for raw in ((-4096,0,0,12000),(0,-4096,0,12000),(0,0,-16384,12000),(0,0,0,0)):
            self.assertFalse(compensate(*raw)[0])
