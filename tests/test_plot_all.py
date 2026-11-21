import csv
import tempfile
import unittest
from collections import deque
from pathlib import Path

from PIL import Image

from scripts.plot_all_joints import IDS, LiveAllPlot, read_run_csv, run_paths, save_plot_png


class AllPlotTests(unittest.TestCase):
    def test_twelve_joint_csv_and_one_image(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path, png_path = run_paths(Path(directory), 90, [.8] * 6)
            self.assertEqual(csv_path.parent, Path(directory) / "1120" / "all")
            self.assertEqual(csv_path.name, "all_90Hz_0.8radps.csv")
            self.assertEqual(png_path.name, "all_90Hz_0.8radps.png")
            mixed_csv, _ = run_paths(Path(directory), 90, [1.5, .8, .8, .8, .8, .8])
            self.assertEqual(mixed_csv.name, "all_90Hz_v1.5-0.8-0.8-0.8-0.8-0.8radps.csv")
            with csv_path.open("w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["elapsed_s", "leader_id", "raw_target", "measured"])
                for cycle in range(5):
                    for ident in IDS:
                        writer.writerow([cycle / 90, ident, ident / 10 + cycle / 100,
                                         ident / 10 + (cycle - 1) / 100])
            rows = read_run_csv(csv_path)
            self.assertEqual(set(rows), set(IDS))
            self.assertTrue(all(len(rows[ident]) == 5 for ident in IDS))
            self.assertAlmostEqual(rows[1][0][0], 0.)
            self.assertAlmostEqual(rows[1][0][1], .1)
            self.assertAlmostEqual(rows[1][0][2], .09)
            self.assertTrue(save_plot_png(csv_path, png_path, 90))
            with Image.open(png_path) as image:
                self.assertEqual(image.size, (2100, 1680))
            self.assertEqual(list(csv_path.parent.glob("*.png")), [png_path])

    def test_live_csv_tail_waits_for_complete_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "all.csv"
            plot = object.__new__(LiveAllPlot)
            plot.csv_path = path
            plot.offset = 0
            plot.pending = ""
            plot.columns = None
            plot.samples = {ident: deque(maxlen=20000) for ident in IDS}
            with path.open("w", newline="") as file:
                csv.writer(file).writerow(["elapsed_s", "leader_id", "raw_target", "measured"])
                file.write("0.01,1,0.2,")
            plot.read_new_rows()
            self.assertFalse(plot.samples[1])
            with path.open("a", newline="") as file:
                file.write("0.18\r\n")
            plot.read_new_rows()
            self.assertEqual(list(plot.samples[1]), [(0.01, 0.2, 0.18)])
            self.assertFalse(plot.samples[2])


if __name__ == "__main__":
    unittest.main()
