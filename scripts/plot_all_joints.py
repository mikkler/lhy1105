"""Plot all twelve mapped leader targets and follower feedback during live teleop."""

from __future__ import annotations

import argparse
import csv
import signal
import subprocess
import sys
import tkinter as tk
from collections import deque
from pathlib import Path
from typing import Sequence

from PIL import Image, ImageDraw, ImageFont

if __package__:
    from .plot_single_joint import frequency_text, sample_rate_stats
else:
    from plot_single_joint import frequency_text, sample_rate_stats


ROOT = Path(__file__).resolve().parents[1]
IDS = (1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13)
COLORS = {"target": "#2563eb", "measured": "#16a34a"}
Sample = tuple[float, float, float]


def run_paths(root: Path, hz: float,
              max_velocity: Sequence[float]) -> tuple[Path, Path]:
    directory = root / "1120" / "all"
    directory.mkdir(parents=True, exist_ok=True)
    speeds = [float(value) for value in max_velocity]
    if len(speeds) != 6:
        raise ValueError("expected six arm joint velocity limits")
    speed_tag = (f"{speeds[0]:g}" if len(set(speeds)) == 1 else
                 "v" + "-".join(f"{value:g}" for value in speeds))
    stem = directory / f"all_{hz:g}Hz_{speed_tag}radps"
    return Path(f"{stem}.csv"), Path(f"{stem}.png")


def read_run_csv(csv_path: Path) -> dict[int, list[Sample]]:
    samples: dict[int, list[Sample]] = {ident: [] for ident in IDS}
    if not csv_path.exists():
        return samples
    with csv_path.open("r", encoding="utf-8", newline="") as file:
        for row in csv.DictReader(file):
            ident = int(row["leader_id"])
            if ident in samples:
                samples[ident].append((float(row["elapsed_s"]),
                                       float(row["raw_target"]),
                                       float(row["measured"])))
    return samples


def _sample_rate(samples: dict[int, Sequence[Sample]]) -> tuple[float | None, float | None]:
    for ident in IDS:
        if len(samples[ident]) >= 2:
            return sample_rate_stats([row[0] for row in samples[ident]])
    return None, None


def save_plot_png(csv_path: Path, png_path: Path, configured_hz: float) -> bool:
    """Save a four by three grid with all twelve mapped/actual angle pairs."""
    samples = read_run_csv(csv_path)
    if not any(samples.values()):
        return False
    actual_hz, p95_ms = _sample_rate(samples)
    image = Image.new("RGB", (2100, 1680), "white")
    draw = ImageDraw.Draw(image)
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    font = ImageFont.truetype(str(font_path), 17) if font_path.exists() else ImageFont.load_default()
    title_font = ImageFont.truetype(str(font_path), 29) if font_path.exists() else font
    panel_font = ImageFont.truetype(str(font_path), 20) if font_path.exists() else font
    draw.text((55, 17), "all-12-joints-angle", fill="#0f172a", font=title_font)
    draw.text((2040, 26), frequency_text(configured_hz, actual_hz, p95_ms),
              fill="#475569", font=font, anchor="ra")
    for key, label, x in (("target", "Leader mapped angle", 705),
                          ("measured", "Follower measured angle", 1115)):
        draw.line([(x, 79), (x + 32, 79)], fill=COLORS[key], width=4)
        draw.text((x + 40, 66), label, fill=COLORS[key], font=font)
    duration = max((row[0] for values in samples.values() for row in values), default=0.)
    x_max = max(1., duration)
    panel_width, panel_height = 660, 380
    for index, ident in enumerate(IDS):
        col, row_index = index % 3, index // 3
        x0 = 45 + col * 685
        y0 = 110 + row_index * panel_height
        left, right = x0 + 66, x0 + panel_width - 13
        top, bottom = y0 + 49, y0 + 309
        side = "left" if ident <= 6 else "right"
        joint = ident if ident <= 6 else ident - 7
        draw.text((x0 + 9, y0 + 7), f"ID {ident}  {side} joint {joint}",
                  fill="#0f172a", font=panel_font)
        values = [value for sample in samples[ident] for value in sample[1:]]
        if not values:
            draw.text(((left + right) / 2, (top + bottom) / 2), "No samples",
                      fill="#64748b", font=font, anchor="mm")
            continue
        y_min, y_max = min(values), max(values)
        pad = max(.05, (y_max - y_min) * .12)
        y_min -= pad
        y_max += pad

        def point(t: float, angle: float) -> tuple[float, float]:
            return (left + t * (right - left) / x_max,
                    bottom - (angle - y_min) * (bottom - top) / (y_max - y_min))

        for step in range(5):
            xx = left + (right - left) * step / 4
            yy = top + (bottom - top) * step / 4
            draw.line([(xx, top), (xx, bottom)], fill="#e2e8f0", width=1)
            draw.line([(left, yy), (right, yy)], fill="#e2e8f0", width=1)
            draw.text((xx, bottom + 8), f"{x_max * step / 4:.1f}",
                      fill="#475569", font=font, anchor="mt")
            draw.text((left - 7, yy), f"{y_max - (y_max-y_min) * step / 4:.2f}",
                      fill="#475569", font=font, anchor="rm")
        for key, axis in (("target", 1), ("measured", 2)):
            points = [point(sample[0], sample[axis]) for sample in samples[ident]]
            if len(points) > 1:
                draw.line(points, fill=COLORS[key], width=3, joint="curve")
            else:
                x, y = points[0]
                draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=COLORS[key])
    draw.text((1050, 1643), "Time after ARM (s) | Angle (rad)",
              fill="#475569", font=font, anchor="mm")
    image.save(png_path, format="PNG")
    return True


class LiveAllPlot:
    def __init__(self, root: tk.Tk, process: subprocess.Popen, csv_path: Path,
                 png_path: Path, configured_hz: float, window_s: float):
        self.root = root
        self.process = process
        self.csv_path = csv_path
        self.png_path = png_path
        self.configured_hz = configured_hz
        self.window_s = window_s
        self.samples: dict[int, deque[Sample]] = {ident: deque(maxlen=20000) for ident in IDS}
        self.offset = 0
        self.pending = ""
        self.columns: dict[str, int] | None = None
        self.finished = False
        self.stopping = False
        self.image_saved = False
        self.status = tk.StringVar(value="Waiting for CONNECT and ARM in terminal")
        root.title("all-12-joints-angle")
        self.canvas = tk.Canvas(root, width=1600, height=1000, bg="white")
        self.canvas.pack(fill="both", expand=True)
        tk.Label(root, textvariable=self.status, anchor="w", padx=12, pady=8).pack(fill="x")
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.poll()

    def read_new_rows(self) -> None:
        if not self.csv_path.exists():
            return
        with self.csv_path.open("r", encoding="utf-8", newline="") as file:
            file.seek(self.offset)
            chunk = file.read()
            self.offset = file.tell()
        if not chunk:
            return
        lines = (self.pending + chunk).split("\n")
        self.pending = lines.pop()
        for line in lines:
            if not line.strip():
                continue
            row = next(csv.reader([line]))
            if self.columns is None:
                self.columns = {name: index for index, name in enumerate(row)}
                required = {"elapsed_s", "leader_id", "raw_target", "measured"}
                if not required.issubset(self.columns):
                    raise ValueError(f"CSV missing columns: {sorted(required - self.columns.keys())}")
                continue
            col = self.columns
            ident = int(row[col["leader_id"]])
            if ident in self.samples:
                self.samples[ident].append((float(row[col["elapsed_s"]]),
                                            float(row[col["raw_target"]]),
                                            float(row[col["measured"]])))

    def draw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 900)
        height = max(canvas.winfo_height(), 720)
        canvas.create_text(22, 16, text="all-12-joints-angle", anchor="w", fill="#0f172a")
        for key, label, x in (("target", "Leader mapped angle", 330),
                              ("measured", "Follower measured angle", 610)):
            canvas.create_line(x, 19, x + 22, 19, fill=COLORS[key], width=3)
            canvas.create_text(x + 29, 19, text=label, anchor="w", fill=COLORS[key])
        latest = max((values[-1][0] for values in self.samples.values() if values), default=None)
        if latest is None:
            canvas.create_text(width / 2, height / 2, text="Waiting for samples...",
                               fill="#475569")
            canvas.create_text(width - 20, 47,
                               text=frequency_text(self.configured_hz, None, None),
                               anchor="e", fill="#475569")
            return
        x_min = max(0., latest - self.window_s)
        x_max = x_min + self.window_s
        actual_hz, p95_ms = sample_rate_stats(
            [row[0] for row in self.samples[IDS[0]] if row[0] >= latest - 3.])
        canvas.create_text(width - 20, 47,
                           text=frequency_text(self.configured_hz, actual_hz, p95_ms,
                                               rolling=True), anchor="e", fill="#475569")
        panel_width = width / 3
        panel_height = (height - 90) / 4
        for index, ident in enumerate(IDS):
            col, row_index = index % 3, index // 3
            x0 = col * panel_width
            y0 = 75 + row_index * panel_height
            left, right = x0 + 63, x0 + panel_width - 17
            top, bottom = y0 + 31, y0 + panel_height - 24
            side = "L" if ident <= 6 else "R"
            joint = ident if ident <= 6 else ident - 7
            canvas.create_text(x0 + 15, y0 + 11, text=f"ID {ident} ({side}{joint})",
                               anchor="w", fill="#0f172a")
            visible = [sample for sample in self.samples[ident] if sample[0] >= x_min]
            if not visible:
                continue
            values = [value for sample in visible for value in sample[1:]]
            y_min, y_max = min(values), max(values)
            pad = max(.05, (y_max - y_min) * .12)
            y_min -= pad
            y_max += pad
            for step in range(4):
                xx = left + (right - left) * step / 3
                yy = top + (bottom - top) * step / 3
                canvas.create_line(xx, top, xx, bottom, fill="#e2e8f0")
                canvas.create_line(left, yy, right, yy, fill="#e2e8f0")
                canvas.create_text(left - 5, yy,
                                   text=f"{y_max - (y_max-y_min) * step / 3:.2f}",
                                   anchor="e", fill="#475569")
                if row_index == 3:
                    canvas.create_text(xx, bottom + 12,
                                       text=f"{x_min + self.window_s * step / 3:.1f}",
                                       fill="#475569")
            stride = max(1, len(visible) // 500)
            display = visible[::stride]
            if display[-1] != visible[-1]:
                display.append(visible[-1])
            for key, axis in (("target", 1), ("measured", 2)):
                points = [coordinate for sample in display for coordinate in (
                    left + (sample[0] - x_min) * (right - left) / (x_max - x_min),
                    bottom - (sample[axis] - y_min) * (bottom - top) / (y_max - y_min))]
                if len(points) >= 4:
                    canvas.create_line(*points, fill=COLORS[key], width=2)
        canvas.create_text(width / 2, height - 9, text="Time after ARM (s) | Angle (rad)",
                           fill="#475569")
        self.status.set(f"12 joints  t={latest:.1f}s  actual={actual_hz:.1f} Hz"
                        if actual_hz is not None else f"12 joints  t={latest:.1f}s")

    def poll(self) -> None:
        try:
            self.read_new_rows()
            self.draw()
        except (OSError, ValueError, IndexError, KeyError) as exc:
            self.status.set(f"Plot read error: {exc}")
        if self.process.poll() is not None and not self.finished:
            self.finished = True
            print(f"控制程序已结束，退出码 {self.process.returncode}；数据：{self.csv_path}", flush=True)
            self.save_image()
            if not any(self.samples.values()):
                self.status.set(f"Control process exited ({self.process.returncode}); no samples")
            if self.stopping:
                self.root.destroy()
                return
        self.root.after(150, self.poll)

    def save_image(self) -> None:
        if self.image_saved or self.process.poll() is None:
            return
        self.image_saved = True
        try:
            if save_plot_png(self.csv_path, self.png_path, self.configured_hz):
                print(f"12 关节完整角度图已保存：{self.png_path}", flush=True)
            else:
                print("未开始采样，因此没有生成图片。", flush=True)
        except (OSError, ValueError, KeyError) as exc:
            print(f"图片保存失败：{exc}", file=sys.stderr, flush=True)

    def close(self) -> None:
        if self.process.poll() is None:
            if self.stopping:
                return
            self.stopping = True
            self.status.set("Stopping control process and waiting for both followers to return to zero...")
            self.process.send_signal(signal.SIGINT)
        else:
            self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="实时绘制整体 live 的 12 个主从关节角度")
    parser.add_argument("--config", type=Path, default=ROOT / "config_can_leader.json")
    parser.add_argument("--window", type=float, default=20., help="实时显示最近多少秒，默认 20")
    parser.add_argument("--duration", type=float, help="最多运行多少秒")
    args = parser.parse_args()
    try:
        if args.window <= 0 or (args.duration is not None and args.duration <= 0):
            raise ValueError("--window 和 --duration 必须大于 0")
        from lhy1105.config import load_config
        cfg = load_config(args.config)
        if cfg.control_ids != IDS or any(cfg.follower_has_gripper.values()):
            raise ValueError("此脚本要求左右从臂各 6 个关节、无电动夹爪")
        root = tk.Tk()
    except (OSError, ValueError, tk.TclError) as exc:
        parser.exit(2, f"无法启动：{exc}\n")
    csv_path, png_path = run_paths(ROOT, cfg.hz, cfg.max_velocity[:6])
    csv_path.unlink(missing_ok=True)
    png_path.unlink(missing_ok=True)
    command = [sys.executable, "-m", "lhy1105", "--config", str(args.config.resolve()),
               "teleop", "--live", "--output", str(csv_path)]
    if not cfg.mapping_confirmed:
        command.append("--trial")
        print("当前映射未确认；整体控制使用 --trial 模式。", flush=True)
    if args.duration is not None:
        command += ["--duration", str(args.duration)]
    print("整体实时图将显示 12 个关节；在终端输入 CONNECT、ARM，按 x/q 停止或关闭图窗。",
          flush=True)
    process = subprocess.Popen(command, cwd=ROOT)
    plot = LiveAllPlot(root, process, csv_path, png_path, cfg.hz, args.window)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        # The terminal sends Ctrl+C to both processes in the same foreground group.
        # A second SIGINT could interrupt the child's controlled return to zero.
        plot.stopping = True
        print("正在等待两条从臂回零并保存图片…", flush=True)
    finally:
        if process.poll() is None and not plot.stopping:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            print("控制进程未退出；请检查终端和机械臂状态。", file=sys.stderr)
        plot.save_image()


if __name__ == "__main__":
    main()
