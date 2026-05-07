from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk


class SignedBarGauge(ttk.Frame):
    def __init__(
        self,
        master: tk.Misc,
        *,
        label: str,
        minimum: float,
        maximum: float,
        unit: str = "",
        width: int = 240,
        height: int = 28,
    ):
        super().__init__(master)
        self.minimum = minimum
        self.maximum = maximum
        self.unit = unit
        self.width = width
        self.height = height
        self._value: float | None = None
        self._text_override: str | None = None

        self.label_var = tk.StringVar(value=label)
        ttk.Label(self, textvariable=self.label_var).grid(row=0, column=0, sticky="w")
        self.value_var = tk.StringVar(value="--")
        ttk.Label(self, textvariable=self.value_var, width=12, anchor="e").grid(row=0, column=1, sticky="e")
        self.canvas = tk.Canvas(self, width=width, height=height, highlightthickness=0, bg="#f6f7f8")
        self.canvas.grid(row=1, column=0, columnspan=2, sticky="ew")
        self.columnconfigure(0, weight=1)
        self.canvas.bind("<Configure>", lambda _event: self._redraw())
        self._redraw()

    def set_range(self, minimum: float, maximum: float) -> None:
        if minimum >= maximum:
            raise ValueError("gauge minimum must be less than maximum")
        self.minimum = minimum
        self.maximum = maximum
        self._redraw()

    def set_value(self, value: float | None, *, text: str | None = None) -> None:
        self._value = None if value is None else float(value)
        self._text_override = text
        if text is not None:
            self.value_var.set(text)
        elif value is None:
            self.value_var.set("--")
        elif self.unit:
            self.value_var.set(f"{value:.2f} {self.unit}")
        else:
            self.value_var.set(f"{value:.2f}")
        self._redraw()

    def _redraw(self) -> None:
        self.canvas.delete("all")
        width = max(1, self.canvas.winfo_width() or self.width)
        height = max(1, self.canvas.winfo_height() or self.height)
        pad = 3
        track_left = pad
        track_right = width - pad
        track_top = pad
        track_bottom = height - pad
        track_width = track_right - track_left

        self.canvas.create_rectangle(track_left, track_top, track_right, track_bottom, fill="#e5e7eb", outline="#c8cdd2")
        zero_ratio = self._ratio(0.0)
        zero_x = track_left + zero_ratio * track_width
        self.canvas.create_line(zero_x, track_top, zero_x, track_bottom, fill="#6b7280")

        if self._value is None:
            return

        clamped = min(max(self._value, self.minimum), self.maximum)
        value_x = track_left + self._ratio(clamped) * track_width
        if clamped >= 0.0:
            fill_left, fill_right = zero_x, value_x
            color = "#2e7d32"
        else:
            fill_left, fill_right = value_x, zero_x
            color = "#c62828"
        if abs(fill_right - fill_left) >= 1.0:
            self.canvas.create_rectangle(fill_left, track_top + 1, fill_right, track_bottom - 1, fill=color, outline="")

    def _ratio(self, value: float) -> float:
        if self.maximum == self.minimum:
            return 0.0
        return min(max((value - self.minimum) / (self.maximum - self.minimum), 0.0), 1.0)


class DialGauge(ttk.Frame):
    DEFAULT_CANVAS_BACKGROUND = "#f0f0f0"

    def __init__(
        self,
        master: tk.Misc,
        *,
        label: str,
        minimum: float,
        maximum: float,
        unit: str = "",
        width: int = 190,
        height: int = 190,
        tick_count: int = 9,
        label_count: int = 9,
    ):
        super().__init__(master)
        self.minimum = minimum
        self.maximum = maximum
        self.unit = unit
        self.width = width
        self.height = height
        self.tick_count = max(2, tick_count)
        self.label_count = max(2, label_count)
        self._value: float | None = None
        self._target: float | None = None
        self._zones: tuple[tuple[float, float, str], ...] = ()
        self._face_color: str | None = None
        self._scale_values: tuple[float, ...] | None = None
        self._scale_pivot: tuple[float, float] | None = None

        self.title_var = tk.StringVar(value=label)
        self.value_var = tk.StringVar(value="--")
        self.target_var = tk.StringVar(value="Target: --")

        ttk.Label(self, textvariable=self.title_var, anchor="center").grid(row=0, column=0, sticky="ew")
        self.canvas = tk.Canvas(self, width=width, height=height, highlightthickness=0, bg=self._canvas_background_color())
        self.canvas.grid(row=1, column=0, sticky="nsew")
        ttk.Label(self, textvariable=self.value_var, anchor="center").grid(row=2, column=0, sticky="ew")
        ttk.Label(self, textvariable=self.target_var, anchor="center").grid(row=3, column=0, sticky="ew")

        self.columnconfigure(0, weight=1)
        self.canvas.bind("<Configure>", lambda _event: self._redraw())
        self._redraw()

    def set_range(self, minimum: float, maximum: float) -> None:
        if minimum >= maximum:
            raise ValueError("gauge minimum must be less than maximum")
        self.minimum = minimum
        self.maximum = maximum
        self._redraw()

    def set_scale_pivot(self, value: float | None, ratio: float | None = None) -> None:
        if value is None or ratio is None:
            self._scale_pivot = None
        else:
            self._scale_pivot = (float(value), min(max(float(ratio), 0.0), 1.0))
        self._redraw()

    def set_zones(self, zones: tuple[tuple[float, float, str], ...]) -> None:
        self._zones = tuple(zones)
        self._redraw()

    def set_scale_values(self, values: tuple[float, ...] | None) -> None:
        if values is None:
            self._scale_values = None
        else:
            self._scale_values = tuple(float(value) for value in values)
        self._redraw()

    def set_face_color(self, color: str | None) -> None:
        self._face_color = color
        self._redraw()

    def set_value(
        self,
        value: float | None,
        *,
        target: float | None = None,
        text: str | None = None,
        target_text: str | None = None,
    ) -> None:
        self._value = None if value is None else float(value)
        self._target = None if target is None else float(target)
        self.value_var.set(text if text is not None else self._format_value(self._value))
        self.target_var.set(target_text if target_text is not None else f"Target: {self._format_value(self._target)}")
        self._redraw()

    def _redraw(self) -> None:
        self.canvas.configure(bg=self._canvas_background_color())
        self.canvas.delete("all")
        width = max(1, self.canvas.winfo_width() or self.width)
        height = max(1, self.canvas.winfo_height() or self.height)
        center_x = width / 2.0
        center_y = height * 0.6
        radius = min(width * 0.42, height * 0.62)

        if self._face_color is not None:
            self._draw_face(width, height, center_x, self._face_color)
        self._draw_arc(center_x, center_y, radius)
        self._draw_ticks(center_x, center_y, radius)
        if self._target is not None:
            self._draw_target(center_x, center_y, radius, self._target)
        self._draw_needle(center_x, center_y, radius, self._value)
        self.canvas.create_oval(center_x - 5, center_y - 5, center_x + 5, center_y + 5, fill="#111827", outline="")

    def _draw_face(self, width: float, height: float, center_x: float, color: str) -> None:
        face_center_y = height * 0.51
        face_radius = min(
            width * 0.49,
            center_x - 2,
            width - center_x - 2,
            face_center_y - 2,
            height - face_center_y,
        )
        if face_radius <= 0.0:
            return
        self.canvas.create_oval(
            center_x - face_radius,
            face_center_y - face_radius,
            center_x + face_radius,
            face_center_y + face_radius,
            fill=color,
            outline="",
        )

    def _canvas_background_color(self) -> str:
        try:
            style_background = ttk.Style(self).lookup("TFrame", "background")
            if style_background:
                return style_background
        except tk.TclError:
            pass
        try:
            return str(self.master.cget("background"))
        except tk.TclError:
            return self.DEFAULT_CANVAS_BACKGROUND

    def _draw_arc(self, center_x: float, center_y: float, radius: float) -> None:
        self._draw_arc_segment(center_x, center_y, radius, 0.0, 1.0, "#cbd5e1", width=5)
        for start, end, color in self._zones:
            self._draw_arc_segment(center_x, center_y, radius, self._ratio(start), self._ratio(end), color, width=7)

        zero_ratio = self._ratio(0.0)
        if 0.0 <= zero_ratio <= 1.0:
            zero_angle = self._angle_for_ratio(zero_ratio)
            inner = self._point(center_x, center_y, radius - 12, zero_angle)
            outer = self._point(center_x, center_y, radius + 3, zero_angle)
            self.canvas.create_line(*inner, *outer, fill="#475569", width=2)

    def _draw_arc_segment(
        self,
        center_x: float,
        center_y: float,
        radius: float,
        start_ratio: float,
        end_ratio: float,
        color: str,
        *,
        width: int,
    ) -> None:
        lower_ratio = max(0.0, min(start_ratio, end_ratio))
        upper_ratio = min(1.0, max(start_ratio, end_ratio))
        if upper_ratio <= lower_ratio:
            return

        steps = max(2, int((upper_ratio - lower_ratio) * 72.0))
        points: list[float] = []
        for step in range(steps + 1):
            ratio = lower_ratio + ((upper_ratio - lower_ratio) * step / steps)
            angle = self._angle_for_ratio(ratio)
            x, y = self._point(center_x, center_y, radius, angle)
            points.extend((x, y))
        if len(points) >= 4:
            self.canvas.create_line(*points, fill=color, width=width, smooth=True, capstyle="round")

    def _draw_ticks(self, center_x: float, center_y: float, radius: float) -> None:
        if self._scale_values is not None:
            tick_values = self._scale_values
            label_values = self._scale_values
        else:
            tick_values = tuple(
                self.minimum + (self.maximum - self.minimum) * step / (self.tick_count - 1)
                for step in range(self.tick_count)
            )
            label_values = tuple(
                self.minimum + (self.maximum - self.minimum) * step / (self.label_count - 1)
                for step in range(self.label_count)
            )

        for value in tick_values:
            angle = self._angle_for_value(value)
            inner = self._point(center_x, center_y, radius - 7, angle)
            outer = self._point(center_x, center_y, radius + 3, angle)
            self.canvas.create_line(*inner, *outer, fill="#64748b", width=1)

        for value in label_values:
            angle = self._angle_for_value(value)
            label_point = self._point(center_x, center_y, radius - 28, angle)
            self.canvas.create_text(
                *label_point,
                text=self._format_tick(value),
                fill="#334155",
                font=("TkDefaultFont", 8),
            )

    def _draw_target(self, center_x: float, center_y: float, radius: float, target: float) -> None:
        angle = self._angle_for_value(target)
        inner = self._point(center_x, center_y, radius - 18, angle)
        outer = self._point(center_x, center_y, radius + 7, angle)
        self.canvas.create_line(*inner, *outer, fill="#f97316", width=4, capstyle="round")

    def _draw_needle(self, center_x: float, center_y: float, radius: float, value: float | None) -> None:
        if value is None:
            angle = self._angle_for_ratio(self._ratio(0.0))
            fill = "#94a3b8"
        else:
            angle = self._angle_for_value(value)
            fill = "#2563eb"
        end = self._point(center_x, center_y, radius - 20, angle)
        self.canvas.create_line(center_x, center_y, *end, fill=fill, width=4, capstyle="round")

    def _ratio(self, value: float) -> float:
        if self.maximum == self.minimum:
            return 0.0
        if self._scale_pivot is not None:
            pivot_value, pivot_ratio = self._scale_pivot
            if self.minimum < pivot_value < self.maximum:
                if value <= pivot_value:
                    return min(max(((value - self.minimum) / (pivot_value - self.minimum)) * pivot_ratio, 0.0), 1.0)
                return min(
                    max(pivot_ratio + ((value - pivot_value) / (self.maximum - pivot_value)) * (1.0 - pivot_ratio), 0.0),
                    1.0,
                )
        return min(max((value - self.minimum) / (self.maximum - self.minimum), 0.0), 1.0)

    def _angle_for_value(self, value: float) -> float:
        return self._angle_for_ratio(self._ratio(value))

    def _angle_for_ratio(self, ratio: float) -> float:
        return 225.0 - ratio * 270.0

    def _point(self, center_x: float, center_y: float, radius: float, angle_degrees: float) -> tuple[float, float]:
        angle = math.radians(angle_degrees)
        return center_x + math.cos(angle) * radius, center_y - math.sin(angle) * radius

    def _format_value(self, value: float | None) -> str:
        if value is None:
            return "--"
        suffix = f" {self.unit}" if self.unit else ""
        return f"{value:.2f}{suffix}"

    def _format_tick(self, value: float) -> str:
        if abs(value) >= 100.0:
            return f"{value:.0f}"
        if abs(value) >= 10.0:
            return f"{value:.1f}"
        return f"{value:.2g}"
