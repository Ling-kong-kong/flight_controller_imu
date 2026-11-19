"""Catch syntax errors in modules loaded lazily when connecting devices."""
from pathlib import Path
import tokenize
import unittest


class SourceSyntaxTests(unittest.TestCase):
    def test_all_viewer_modules_compile(self):
        package = Path(__file__).resolve().parents[1] / "imu_viewer"
        sources = sorted(package.rglob("*.py"))
        self.assertTrue(sources)
        for path in sources:
            with self.subTest(module=str(path.relative_to(package))):
                with tokenize.open(path) as source:
                    compile(source.read(), str(path), "exec")
