"""Plot one leader/follower joint live while the existing calibrate command controls it."""

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


ROOT = Path(__file__).resolve().parents[1]
VALID_IDS = tuple(range(1, 7)) + tuple(range(8, 14))
COLORS = {"leader": "#2563eb", "measured": "#16a34a"}


def choose_id(value: int | None) -> int:
    if value is None:
        value = int(input("选择主臂舵机 ID（1–6 或 8–13）：").strip())
    if value not in VALID_IDS:
        raise ValueError("舵机 ID 必须是 1–6 或 8–13；7、14 是夹爪")
    return value


def run_paths(root: Path, leader_id: int, hz: float,
              max_velocity: float) -> tuple[Path, Path]:
    directory = root / "1120" / f"id{choose_id(leader_id)}"
    directory.mkdir(parents=True, exist_ok=True)
    stem = directory / f"id{leader_id}_{hz:g}Hz_{max_velocity:g}radps"
    return Path(f"{stem}.csv"), Path(f"{stem}.png")


def sample_rate_stats(times: Sequence[float]) -> tuple[float | None, float | None]:
    """Observed loop samples per second and 95th percentile sample interval in ms."""
    intervals = [later - earlier for earlier, later in zip(times, times[1:])
                 if later > earlier]
    if not intervals:
        return None, None
    periods = sorted(intervals)
    return len(intervals) / sum(intervals), periods[(95 * len(periods) - 1) // 100] * 1000


def frequency_text(configured_hz: float | None, actual_hz: float | None,
                   p95_period_ms: float | None, *, rolling: bool = False) -> str:
    configured = f"{configured_hz:g} Hz" if configured_hz is not None else "unknown"
    if actual_hz is None or p95_period_ms is None:
        return f"Configured: {configured} | Actual: measuring..."
    label = "Actual (last 3 s)" if rolling else "Actual (full run)"
    return (f"Configured: {configured} | {label}: {actual_hz:.1f} Hz | "
            f"p95 interval: {p95_period_ms:.1f} ms")


def velocity_series(samples: Sequence[tuple[float, float, float]],
                    smoothing_s: float = .1) -> list[tuple[float, float, float]]:
    """Compute mapped leader target and follower velocities from positions."""
    result = []
    reference = 0
    for index, row in enumerate(samples):
        while reference + 1 < index and row[0] - samples[reference + 1][0] >= smoothing_s:
            reference += 1
        dt = row[0] - samples[reference][0]
        speeds = tuple((row[axis] - samples[reference][axis]) / dt if dt > 0 else 0.
                       for axis in (1, 2))
        result.append((row[0], *speeds))
    return result


def read_plot_csv(csv_path: Path) -> tuple[list[tuple[float, float, float]],
                                           float | None]:
    if not csv_path.exists():
        return [], None
    with csv_path.open("r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        return [], None
    samples = [(float(row["elapsed_s"]), float(row["mapped_target_rad"]),
                float(row["measured"])) for row in rows]
    configured_hz = float(rows[0]["configured_hz"]) if rows[0].get("configured_hz") else None
    return samples, configured_hz


def save_plot_png(csv_path: Path, png_path: Path, leader_id: int,
                  configured_hz: float | None = None) -> bool:
    """Save mapped leader target and follower feedback angles and speeds."""
    samples, recorded_hz = read_plot_csv(csv_path)
    if not samples:
        return False
    speeds = velocity_series(samples)
    actual_hz, p95_period_ms = sample_rate_stats([row[0] for row in samples])
    image = Image.new("RGB", (1500, 1150), "white")
    draw = ImageDraw.Draw(image)
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    font = ImageFont.truetype(str(font_path), 20) if font_path.exists() else ImageFont.load_default()
    title_font = ImageFont.truetype(str(font_path), 27) if font_path.exists() else font
    left, right = 105, 1460
    x_min, x_max = 0., max(1., samples[-1][0])

    def draw_panel(rows: Sequence[tuple[float, float, float]],
                   top: int, bottom: int, label: str, min_pad: float,
                   show_time: bool) -> None:
        values = [value for row in rows for value in row[1:]]
        y_min, y_max = min(values), max(values)
        pad = max(min_pad, (y_max - y_min) * .12)
        y_min -= pad
        y_max += pad

        def point(t: float, value: float) -> tuple[float, float]:
            return (left + (t - x_min) * (right - left) / (x_max - x_min),
                    bottom - (value - y_min) * (bottom - top) / (y_max - y_min))

        draw.text((left, top - 32), label, fill="#475569", font=font)
        for step in range(6):
            x = left + (right - left) * step / 5
            y = top + (bottom - top) * step / 5
            draw.line([(x, top), (x, bottom)], fill="#e2e8f0", width=1)
            draw.line([(left, y), (right, y)], fill="#e2e8f0", width=1)
            if show_time:
                draw.text((x - 18, bottom + 12), f"{x_max * step / 5:.1f}",
                          fill="#475569", font=font)
            draw.text((10, y - 12), f"{y_max - (y_max-y_min) * step / 5:.2f}",
                      fill="#475569", font=font)
        for key, index in (("leader", 1), ("measured", 2)):
            points = [point(row[0], row[index]) for row in rows]
            if len(points) > 1:
                draw.line(points, fill=COLORS[key], width=3, joint="curve")
            else:
                x, y = points[0]
                draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=COLORS[key])

    draw.text((left, 18), f"id{leader_id}-angle-speed", fill="#0f172a", font=title_font)
    draw.text((right, 89), frequency_text(
        recorded_hz if recorded_hz is not None else configured_hz,
        actual_hz, p95_period_ms),
              fill="#475569", font=font, anchor="ra")
    draw.text(((left + right) / 2, 1118), "Time after ARM (s)",
              fill="#475569", font=font, anchor="mm")
    for key, label, legend_x in (("leader", "Leader mapped angle / speed", 510),
                                 ("measured", "Follower measured angle / speed", 1010)):
        draw.line([(legend_x, 54), (legend_x + 30, 54)], fill=COLORS[key], width=4)
        draw.text((legend_x + 37, 39), label, fill=COLORS[key], font=font)
    draw_panel(samples, 155, 565, "Angle (rad)", .06, False)
    draw_panel(speeds, 660, 1060, "Speed (rad/s)", .1, True)
    image.save(png_path, format="PNG")
    return True


class LivePlot:
    def __init__(self, root: tk.Tk, process: subprocess.Popen, csv_path: Path, png_path: Path,
                 leader_id: int, window_s: float, configured_hz: float):
        self.root = root
        self.process = process
        self.csv_path = csv_path
        self.png_path = png_path
        self.leader_id = leader_id
        self.window_s = window_s
        self.configured_hz = configured_hz
        self.samples: deque[tuple[float, float, float]] = deque(maxlen=20000)
        self.offset = 0
        self.pending = ""
        self.columns: dict[str, int] | None = None
        self.stopping = False
        self.finished = False
        self.image_saved = False
        self.status = tk.StringVar(value="Waiting for CONNECT and ARM; mapped target starts at follower pose")
        root.title(f"id{leader_id}-angle and speed")
        self.canvas = tk.Canvas(root, width=1050, height=800, bg="white")
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
                required = {"elapsed_s", "mapped_target_rad", "measured"}
                if not required.issubset(self.columns):
                    raise ValueError(f"CSV missing columns: {sorted(required - self.columns.keys())}")
                continue
            col = self.columns
            self.samples.append((float(row[col["elapsed_s"]]),
                                 float(row[col["mapped_target_rad"]]),
                                 float(row[col["measured"]])))

    def draw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 600)
        height = max(canvas.winfo_height(), 640)
        left, right = 78, width - 20
        angle_top, angle_bottom = 112, int(height * .49)
        speed_top, speed_bottom = int(height * .61), height - 50
        canvas.create_text(left, 18, text=f"id{self.leader_id}-angle", anchor="w",
                           fill="#0f172a")
        canvas.create_text(left, speed_top - 26, text=f"id{self.leader_id}-speed",
                           anchor="w", fill="#0f172a")
        if not self.samples:
            canvas.create_text(right, 70,
                               text=frequency_text(self.configured_hz, None, None),
                               anchor="e", fill="#475569")
            canvas.create_text(width / 2, height / 2, text="Waiting for samples...", fill="#475569")
            return
        latest = self.samples[-1]
        x_min = max(0., latest[0] - self.window_s)
        x_max = x_min + self.window_s
        visible = [row for row in self.samples if row[0] >= x_min]
        context = [row for row in self.samples if row[0] >= x_min - .12]
        speeds = [row for row in velocity_series(context) if row[0] >= x_min]
        recent_times = [row[0] for row in self.samples if row[0] >= latest[0] - 3.]
        actual_hz, p95_period_ms = sample_rate_stats(recent_times)
        canvas.create_text(right, 70,
                           text=frequency_text(self.configured_hz, actual_hz,
                                               p95_period_ms, rolling=True),
                           anchor="e", fill="#475569")

        def draw_panel(rows: Sequence[tuple[float, float, float]], top: int,
                       bottom: int, y_label: str, min_pad: float) -> None:
            values = [value for row in rows for value in row[1:]]
            y_min, y_max = min(values), max(values)
            pad = max(min_pad, (y_max - y_min) * .12)
            y_min -= pad
            y_max += pad

            def point(t: float, value: float) -> tuple[float, float]:
                return (left + (t - x_min) * (right - left) / (x_max - x_min),
                        bottom - (value - y_min) * (bottom - top) / (y_max - y_min))

            canvas.create_text(right, top - 18, text=y_label, anchor="e", fill="#475569")
            for step in range(6):
                xx = left + (right - left) * step / 5
                yy = top + (bottom - top) * step / 5
                canvas.create_line(xx, top, xx, bottom, fill="#e2e8f0")
                canvas.create_line(left, yy, right, yy, fill="#e2e8f0")
                canvas.create_text(xx, bottom + 17,
                                   text=f"{x_min + self.window_s * step / 5:.1f}",
                                   fill="#475569")
                canvas.create_text(left - 10, yy,
                                   text=f"{y_max - (y_max-y_min) * step / 5:.2f}",
                                   anchor="e", fill="#475569")
            for key, index in (("leader", 1), ("measured", 2)):
                points = [coordinate for row in rows for coordinate in point(row[0], row[index])]
                if len(points) >= 4:
                    canvas.create_line(*points, fill=COLORS[key], width=2)

        draw_panel(visible, angle_top, angle_bottom, "Angle (rad)", .06)
        draw_panel(speeds, speed_top, speed_bottom, "Speed (rad/s)", .1)
        canvas.create_text((left + right) / 2, height - 12, text="Time after ARM (s)")
        for key, label in (("leader", "Leader mapped / speed"),
                           ("measured", "Follower measured / speed")):
            legend_x = left + {"leader": 0, "measured": 385}[key]
            canvas.create_line(legend_x, 45, legend_x + 22, 45, fill=COLORS[key], width=3)
            canvas.create_text(legend_x + 28, 45, text=label, anchor="w", fill=COLORS[key])
        self.status.set(f"ID {self.leader_id}  t={latest[0]:.1f}s  "
                        f"Leader mapped={latest[1]:+.3f} rad  "
                        f"Follower measured={latest[2]:+.3f} rad")

    def poll(self) -> None:
        try:
            self.read_new_rows()
            self.draw()
        except (OSError, ValueError, IndexError) as exc:
            self.status.set(f"Plot read error: {exc}")
        if self.process.poll() is not None and not self.finished:
            self.finished = True
            print(f"控制程序已结束，退出码 {self.process.returncode}；数据：{self.csv_path}", flush=True)
            self.save_image()
            if not self.samples:
                self.status.set(f"Control process exited ({self.process.returncode}); no samples")
            if self.stopping:
                self.root.destroy()
                return
        self.root.after(100, self.poll)

    def save_image(self) -> None:
        if self.image_saved or self.process.poll() is None:
            return
        self.image_saved = True
        try:
            if save_plot_png(self.csv_path, self.png_path, self.leader_id,
                             self.configured_hz):
                print(f"完整角度和速度曲线已保存：{self.png_path}", flush=True)
            else:
                print("未开始采样，因此没有生成图片。", flush=True)
        except (OSError, ValueError, KeyError) as exc:
            print(f"图片保存失败：{exc}", file=sys.stderr, flush=True)

    def close(self) -> None:
        if self.process.poll() is None:
            if self.stopping:
                return
            self.stopping = True
            self.status.set("Stopping control process and waiting for motor controller to close...")
            self.process.send_signal(signal.SIGINT)
        else:
            self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="实时绘制单个主从关节的角度和速度")
    parser.add_argument("--id", type=int, help="主臂舵机 ID：1–6 或 8–13；省略时交互选择")
    parser.add_argument("--config", type=Path, default=ROOT / "config_can_leader.json")
    parser.add_argument("--window", type=float, default=20., help="显示最近多少秒，默认 20")
    parser.add_argument("--duration", type=float, help="最多运行多少秒")
    parser.add_argument("--sign", type=int, choices=(-1, 1), help="临时覆盖该关节方向")
    args = parser.parse_args()
    try:
        leader_id = choose_id(args.id)
        if args.window <= 0 or (args.duration is not None and args.duration <= 0):
            raise ValueError("--window 和 --duration 必须大于 0")
        from lhy1105.config import load_config
        cfg = load_config(args.config)
        if cfg.follower_has_gripper["left"] or cfg.follower_has_gripper["right"]:
            raise ValueError("此脚本只用于无电动夹爪的两条新从臂")
        root = tk.Tk()
    except (OSError, ValueError, tk.TclError) as exc:
        parser.exit(2, f"无法启动：{exc}\n")
    axis = (leader_id - 1) if leader_id <= 6 else (leader_id - 8)
    output, image_path = run_paths(ROOT, leader_id, cfg.hz, float(cfg.max_velocity[axis]))
    output.unlink(missing_ok=True)
    image_path.unlink(missing_ok=True)
    command = [sys.executable, "-m", "lhy1105", "--config", str(args.config.resolve()),
               "calibrate", "--id", str(leader_id), "--output", str(output),
               "--quiet", "--mapped-pair"]
    if args.duration is not None:
        command += ["--duration", str(args.duration)]
    if args.sign is not None:
        command += ["--sign", str(args.sign)]
    print(f"实时图将显示 ID {leader_id}；在终端输入 CONNECT、ARM；按 x/q 停止，或关闭图窗。", flush=True)
    process = subprocess.Popen(command, cwd=ROOT)
    plot = LivePlot(root, process, output, image_path, leader_id, args.window, cfg.hz)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        # A terminal Ctrl+C reaches both processes in their shared foreground
        # process group. Let the child finish closing its motor controller.
        plot.stopping = True
        print("正在等待控制程序退出并保存图片…", flush=True)
    finally:
        if process.poll() is None and not plot.stopping:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print("控制进程未退出；请检查终端和机械臂状态。", file=sys.stderr)
        plot.save_image()


if __name__ == "__main__":
    main()
