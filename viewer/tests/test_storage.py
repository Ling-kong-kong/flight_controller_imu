import csv
import json
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from imu_viewer.capture import Config
from imu_viewer.protocol import Frame
from imu_viewer.storage import SessionWriter, finalize, export_euroc, read_csv

class StorageTests(unittest.TestCase):
    def setUp(self):
        # Test artifacts stay under viewer; no repository-external file dependencies.
        self.parent = Path(__file__).resolve().parents[1] / "test-results"
        self.parent.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=self.parent)
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def session(self, bad_pts=False):
        path = self.root/"原始会话"
        writer = SessionWriter(path,Config(port="FAKE",width=8,height=4))
        image = np.zeros((4,8,3),np.uint8)
        image[:,:4] = (10,50,90)
        image[:,4:] = (220,170,120)
        raw = bytearray()
        for i in range(4000):
            f = Frame(i,1000000+i*1000,1,2,3,4,5,6,20,10,16 if i==0 else 0)
            b = f.encode()
            raw.extend(b)
            host = 5000000000 + i*1000000 + 2000000
            writer.usb(i,b,host-10000,host,[f])
            if i%40==0:
                pts = i*1000000 + 1000000000
                writer.image(i//40, pts/1e6, None if bad_pts else pts,
                             host-1000000,host,image)
        writer.event("test","synthetic data")
        writer.close()
        self.assertEqual((path/"usb.bin").read_bytes(),bytes(raw))
        return path

    def test_finalize_repeat_export_units_and_correspondence(self):
        source = self.session()
        first = finalize(source,dict(error=None,timeline_anomaly=False))
        original = (source/"imu.csv").read_bytes()
        model = (source/"clock_models.json").read_bytes()
        self.assertFalse(first["models_valid"])
        self.assertTrue(first["timestamps_valid"])
        self.assertEqual(first["aligned_frames"],100)
        finalize(source)
        self.assertEqual((source/"clock_models.json").read_bytes(),model)
        self.assertEqual((source/"imu.csv").read_bytes(),original)
        target = self.root/"导出"
        result = export_euroc(source,target)
        self.assertEqual(result["imu_frames"],4000)
        for cam in ("cam0","cam1"):
            rows = list(read_csv(target/"mav0"/cam/"data.csv"))
            self.assertEqual(len(rows),100)
            stamps = [int(r["#timestamp [ns]"]) for r in rows]
            self.assertTrue(all(b>a for a,b in zip(stamps,stamps[1:])))
            self.assertTrue((target/"mav0"/cam/"data"/rows[0]["filename"]).exists())
        left = list(read_csv(target/"mav0/cam0/data.csv"))
        right = list(read_csv(target/"mav0/cam1/data.csv"))
        self.assertEqual(left,right)
        imu = list(read_csv(target/"mav0/imu0/data.csv"))
        self.assertEqual(len(imu),4000)
        self.assertAlmostEqual(float(imu[0]["w_RS_S_x [rad s^-1]"]),4*2000/32768*np.pi/180)
        self.assertAlmostEqual(float(imu[0]["a_RS_S_z [m s^-2]"]),3*24/32768*9.80665)
        self.assertFalse((target/"mav0/cam0/sensor.yaml").exists())
        self.assertFalse((target/"EXPORT_INCOMPLETE").exists())
        self.assertTrue((target/"provenance/frames.csv").exists())
        with self.assertRaises(FileExistsError):
            export_euroc(source,target)

    def test_missing_pts_does_not_invalidate_host_timestamps(self):
        source = self.session(True)
        report = finalize(source,dict(error="bad PTS",timeline_anomaly=True))
        self.assertFalse(report["models_valid"])
        self.assertTrue(report["timestamps_valid"])
        self.assertTrue(all(r["unified_timestamp_ns"] for r in read_csv(source/"aligned_frames.csv")))
        result = export_euroc(source,self.root/"host_export")
        self.assertEqual(result["imu_frames"],4000)
        self.assertEqual(result["stereo_pairs"],100)
        finalize(source)
        self.assertEqual(json.loads((source/"session.json").read_text(encoding="utf8"))["state"],"stopped_error")

    def test_png_is_color_and_stereo_geometry(self):
        source = self.session()
        for cam in ("cam0","cam1"):
            data = np.fromfile(source/cam/"000000000.png",dtype=np.uint8)
            image = cv2.imdecode(data,cv2.IMREAD_UNCHANGED)
            self.assertEqual(image.shape,(4,4,3))
        self.assertEqual(list(read_csv(source/"imu.csv"))[0]["timestamp_ns"],"1000000000")

if __name__ == "__main__":
    unittest.main()
