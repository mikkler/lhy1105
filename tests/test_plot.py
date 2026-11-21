import csv
import tempfile
import unittest
from collections import deque
from pathlib import Path

from PIL import Image

from scripts.plot_single_joint import (LivePlot, choose_id, frequency_text, run_paths,
                                       read_plot_csv, sample_rate_stats, save_plot_png,
                                       velocity_series)


class PlotTests(unittest.TestCase):
    def test_actual_sample_rate_uses_timestamps(self):
        times = [index / 90 for index in range(271)]
        actual, p95_ms = sample_rate_stats(times)
        self.assertAlmostEqual(actual, 90.)
        self.assertAlmostEqual(p95_ms, 1000 / 90)
        self.assertIn("Configured: 90 Hz", frequency_text(90, actual, p95_ms))
        self.assertIn("Actual (full run): 90.0 Hz", frequency_text(90, actual, p95_ms))

    def test_id_selection_excludes_grippers(self):
        self.assertEqual(choose_id(1), 1)
        self.assertEqual(choose_id(13), 13)
        for ident in (0, 7, 14):
            with self.assertRaises(ValueError):
                choose_id(ident)

    def test_velocity_uses_position_change_and_sample_time(self):
        samples = [(0., 0., 0.), (.1, .1, .04),
                   (.2, .2, .08)]
        speeds = velocity_series(samples)
        self.assertEqual(speeds[0], (0., 0., 0.))
        for row in speeds[1:]:
            self.assertAlmostEqual(row[1], 1.)
            self.assertAlmostEqual(row[2], .4)

    def test_csv_tail_reads_only_complete_new_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "samples.csv"
            plot = object.__new__(LivePlot)
            plot.csv_path = path
            plot.offset = 0
            plot.pending = ""
            plot.columns = None
            plot.samples = deque(maxlen=20000)
            header = ["elapsed_s", "mapped_target_rad", "measured", "configured_hz"]
            with path.open("w", newline="") as file:
                csv.writer(file).writerow(header)
                file.write("0.01,0.02,")
            plot.read_new_rows()
            self.assertEqual(len(plot.samples), 0)
            with path.open("a", newline="") as file:
                file.write("0.04,60\r\n")
            plot.read_new_rows()
            self.assertEqual(list(plot.samples), [(0.01, 0.02, 0.04)])
            plot.read_new_rows()
            self.assertEqual(len(plot.samples), 1)

    def test_run_paths_and_complete_png(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path, png_path = run_paths(Path(directory), 2, 60, .8)
            self.assertEqual(csv_path.parent, Path(directory) / "1120" / "id2")
            self.assertEqual(csv_path.name, "id2_60Hz_0.8radps.csv")
            self.assertEqual(png_path.name, "id2_60Hz_0.8radps.png")
            self.assertTrue(csv_path.parent.is_dir())
            with csv_path.open("w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["elapsed_s", "mapped_target_rad", "measured", "configured_hz"])
                writer.writerow([0.1, 0.02, 0.08, 60])
                writer.writerow([0.2, 0.12, 0.12, 60])
            samples, recorded_hz = read_plot_csv(csv_path)
            self.assertEqual(samples[0], (0.1, 0.02, 0.08))
            self.assertEqual(recorded_hz, 60.)
            self.assertTrue(save_plot_png(csv_path, png_path, 2))
            self.assertEqual(png_path.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            with Image.open(png_path) as image:
                self.assertEqual(image.size, (1500, 1150))
            self.assertEqual(list(csv_path.parent.glob("*.png")), [png_path])


if __name__ == "__main__":
    unittest.main()
