"""
Renders a world to images and records it to video, without a browser.

A frame shows what the web UI shows: the shapes, labels, and per-robot traces
come from the same figure code (:mod:`pyrobosim.web.figure`), with the same
colors and line styles, but are drawn with Pillow instead of in a browser. This
lets a world be recorded while it runs headless, e.g., while a robot executes a
task.

As in the web UI, the static content (rooms, hallways, locations, object spawns,
resting objects, and their labels) is redrawn only when ``world.change_count``
changes, and the robots are redrawn on every frame.

Video is encoded by an ``ffmpeg`` executable, which must be installed separately.
"""

import math
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from PIL import Image, ImageChops, ImageColor, ImageDraw, ImageFont

from ..core.robot import Robot
from ..core.world import World
from ..utils.logging import get_global_logger
from .figure import _label, _robot_traces, color_to_css, make_figure, num_graph_traces

# Image width, in pixels, at which line widths and font sizes are drawn at
# their nominal (CSS pixel) size. Wider images scale them up proportionally.
REFERENCE_WIDTH = 800
# Margin around the plot area, in pixels at the reference width.
PLOT_MARGIN = 20

# An (R, G, B) color and an alpha in the range (0.0, 1.0).
RGBA = tuple[int, int, int, float]
# A polyline or polygon ring, in pixel coordinates.
PixelPath = list[tuple[float, float]]


def parse_color(color: str | None) -> RGBA | None:
    """
    Parses a CSS color as used by the figure.

    :param color: A CSS color, e.g., ``"rgb(204, 0, 204)"``,
        ``"rgba(204, 0, 204, 0.5)"``, or ``"white"``.
    :return: The (R, G, B, alpha) color, or None if no color is given.
    """
    if color is None:
        return None
    text = color.strip().lower()
    if text.startswith("rgb"):
        inner = text[text.index("(") + 1 : text.index(")")]
        parts = [float(part) for part in inner.split(",")]
        r, g, b = (int(round(part)) for part in parts[:3])
        alpha = parts[3] if len(parts) > 3 else 1.0
        return (r, g, b, alpha)
    r, g, b = ImageColor.getrgb(text)[:3]
    return (r, g, b, 1.0)


def parse_svg_path(path: str) -> list[list[tuple[float, float]]]:
    """
    Parses the SVG path strings the figure builds (``M``, ``L``, and ``Z`` only).

    :param path: The SVG path string.
    :return: The rings (or open polylines) of the path, in world coordinates.
    """
    rings: list[list[tuple[float, float]]] = []
    ring: list[tuple[float, float]] = []
    for token in path.split():
        command = token[0]
        if command == "Z":
            if ring:
                ring.append(ring[0])
                rings.append(ring)
            ring = []
            continue
        if command == "M" and ring:
            rings.append(ring)
            ring = []
        x, y = token[1:].split(",")
        ring.append((float(x), float(y)))
    if ring:
        rings.append(ring)
    return rings


@dataclass
class _Stamp:
    """One color, painted through a coverage mask onto a region of a frame."""

    box: tuple[int, int, int, int]
    color: tuple[int, int, int]
    mask: Image.Image

    def apply(self, image: Image.Image) -> None:
        """
        Paints the stamp onto an image.

        :param image: The RGB image to paint on.
        """
        image.paste(self.color, self.box, self.mask)


class _Canvas:
    """Maps world coordinates to pixels and draws figure primitives as stamps."""

    def __init__(
        self,
        x_range: Sequence[float],
        y_range: Sequence[float],
        width: int,
        supersample: int,
    ) -> None:
        """
        Creates a canvas showing the given world extent.

        :param x_range: The (min, max) world x coordinates to show.
        :param y_range: The (min, max) world y coordinates to show.
        :param width: The output image width, in pixels.
        :param supersample: The factor by which frames are drawn larger than
            the output, then reduced, to smooth edges.
        """
        self.supersample = supersample
        # Pixels per nominal (CSS) pixel, at the drawing resolution.
        self.px = supersample * width / REFERENCE_WIDTH
        margin = PLOT_MARGIN * self.px
        out_width = width + (width % 2)
        self.xmin, self.xmax = float(x_range[0]), float(x_range[1])
        self.ymin, self.ymax = float(y_range[0]), float(y_range[1])
        self.scale = (out_width * supersample - 2.0 * margin) / (self.xmax - self.xmin)
        inner_height = (self.ymax - self.ymin) * self.scale
        # Video encoders need even dimensions.
        out_height = int(math.ceil((inner_height + 2.0 * margin) / supersample))
        out_height += out_height % 2
        self.out_size = (out_width, out_height)
        self.size = (out_width * supersample, out_height * supersample)
        self.left = margin
        self.top = (self.size[1] - inner_height) / 2.0
        self._fonts: dict[int, ImageFont.FreeTypeFont | ImageFont.ImageFont] = {}

    def to_pixel(self, x: float, y: float) -> tuple[float, float]:
        """
        Converts a world position to pixel coordinates.

        :param x: The world x coordinate.
        :param y: The world y coordinate.
        :return: The (column, row) pixel coordinates.
        """
        return (
            self.left + (x - self.xmin) * self.scale,
            self.top + (self.ymax - y) * self.scale,
        )

    def paths(self, xs: Sequence[Any], ys: Sequence[Any]) -> list[PixelPath]:
        """
        Splits None-separated trace coordinates into pixel paths.

        :param xs: The x coordinates, with None separating each path.
        :param ys: The y coordinates, with None separating each path.
        :return: The pixel paths.
        """
        paths: list[PixelPath] = []
        current: PixelPath = []
        for x, y in zip(xs, ys):
            if x is None or y is None:
                if current:
                    paths.append(current)
                current = []
            else:
                current.append(self.to_pixel(float(x), float(y)))
        if current:
            paths.append(current)
        return paths

    def _box(
        self, paths: Sequence[PixelPath], pad: float
    ) -> tuple[int, int, int, int] | None:
        """
        Computes the clipped integer bounding box of some paths.

        :param paths: The pixel paths.
        :param pad: Extra pixels around the paths (e.g., half a line width).
        :return: The (left, top, right, bottom) box, or None if it is empty.
        """
        points = [point for path in paths for point in path]
        if not points:
            return None
        left = max(0, int(math.floor(min(p[0] for p in points) - pad)))
        top = max(0, int(math.floor(min(p[1] for p in points) - pad)))
        right = min(self.size[0], int(math.ceil(max(p[0] for p in points) + pad)) + 1)
        bottom = min(self.size[1], int(math.ceil(max(p[1] for p in points) + pad)) + 1)
        if right <= left or bottom <= top:
            return None
        return (left, top, right, bottom)

    @staticmethod
    def _stamp(
        box: tuple[int, int, int, int],
        coverage: Image.Image,
        color: RGBA,
        opacity: float,
    ) -> _Stamp | None:
        """
        Builds a stamp from a full-coverage mask, scaled by the color's alpha.

        :param box: The region the mask covers.
        :param coverage: The coverage mask (mode ``"L"``).
        :param color: The color to paint.
        :param opacity: An extra opacity multiplier (e.g., a trace's opacity).
        :return: The stamp, or None if it is fully transparent.
        """
        level = color[3] * opacity
        if level <= 0.0:
            return None
        if level < 1.0:
            coverage = coverage.point(lambda value: int(round(value * level)))
        return _Stamp(box, color[:3], coverage)

    def fill(
        self,
        paths: Sequence[PixelPath],
        color: RGBA | None,
        opacity: float = 1.0,
        evenodd: bool = True,
    ) -> _Stamp | None:
        """
        Fills polygon rings.

        :param paths: The rings to fill.
        :param color: The fill color, or None for no fill.
        :param opacity: An extra opacity multiplier.
        :param evenodd: If True, overlapping rings cancel out (e.g., holes);
            otherwise, the rings are filled as a union.
        :return: The stamp, or None if nothing is drawn.
        """
        rings = [path for path in paths if len(path) >= 3]
        box = self._box(rings, 1.0) if color is not None else None
        if box is None or color is None:
            return None
        left, top, right, bottom = box
        size = (right - left, bottom - top)
        mask = Image.new("1", size, 0)
        for ring in rings:
            points = [(x - left, y - top) for x, y in ring]
            if evenodd:
                ring_mask = Image.new("1", size, 0)
                ImageDraw.Draw(ring_mask).polygon(points, fill=1)
                mask = ImageChops.logical_xor(mask, ring_mask)
            else:
                ImageDraw.Draw(mask).polygon(points, fill=1)
        return self._stamp(box, mask.convert("L"), color, opacity)

    def stroke(
        self,
        paths: Sequence[PixelPath],
        color: RGBA | None,
        width: float,
        opacity: float = 1.0,
        dash: str | None = None,
    ) -> _Stamp | None:
        """
        Draws polylines.

        :param paths: The polylines to draw.
        :param color: The line color, or None for no line.
        :param width: The line width, in nominal (CSS) pixels.
        :param opacity: An extra opacity multiplier.
        :param dash: A Plotly dash style. Any style other than ``"solid"`` is
            drawn as Plotly's ``"dash"``.
        :return: The stamp, or None if nothing is drawn.
        """
        if color is None or width <= 0.0:
            return None
        line_width = max(1, int(round(width * self.px)))
        box = self._box(paths, line_width)
        if box is None:
            return None
        left, top, right, bottom = box
        mask = Image.new("L", (right - left, bottom - top), 0)
        draw = ImageDraw.Draw(mask)
        # Plotly's "dash" pattern: dashes and gaps of 3 line widths (at least 3px).
        dash_length = 3.0 * max(width, 3.0) * self.px
        for path in paths:
            points = [(x - left, y - top) for x, y in path]
            if len(points) < 2:
                continue
            if dash is None or dash == "solid":
                draw.line(points, fill=255, width=line_width, joint="curve")
            else:
                for segment in _dashes(points, dash_length, dash_length):
                    draw.line(segment, fill=255, width=line_width)
        return self._stamp(box, mask, color, opacity)

    def markers(
        self,
        paths: Sequence[PixelPath],
        color: RGBA | None,
        size: float,
        opacity: float,
    ) -> _Stamp | None:
        """
        Draws circular markers at points.

        :param paths: The paths whose points get markers.
        :param color: The marker color, or None for no markers.
        :param size: The marker diameter, in nominal (CSS) pixels.
        :param opacity: An extra opacity multiplier.
        :return: The stamp, or None if nothing is drawn.
        """
        radius = 0.5 * size * self.px
        box = self._box(paths, radius + 1.0) if color is not None else None
        if box is None or color is None:
            return None
        left, top, right, bottom = box
        mask = Image.new("L", (right - left, bottom - top), 0)
        draw = ImageDraw.Draw(mask)
        for x, y in (point for path in paths for point in path):
            x, y = x - left, y - top
            draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=255)
        return self._stamp(box, mask, color, opacity)

    def text(
        self,
        x: float,
        y: float,
        text: str,
        color: RGBA | None,
        size: float,
        anchor: str = "mm",
        opacity: float = 1.0,
    ) -> _Stamp | None:
        """
        Draws a text label.

        :param x: The world x coordinate of the anchor point.
        :param y: The world y coordinate of the anchor point.
        :param text: The label text.
        :param color: The text color, or None for no text.
        :param size: The font size, in nominal (CSS) pixels.
        :param anchor: The Pillow text anchor, e.g., ``"mt"`` for middle-top.
        :param opacity: An extra opacity multiplier.
        :return: The stamp, or None if nothing is drawn.
        """
        if not text or color is None:
            return None
        font_size = max(1, int(round(size * self.px)))
        font = self._fonts.get(font_size)
        if font is None:
            font = ImageFont.load_default(size=font_size)
            self._fonts[font_size] = font
        col, row = self.to_pixel(x, y)
        bounds = ImageDraw.Draw(Image.new("L", (1, 1))).textbbox(
            (col, row), text, font=font, anchor=anchor
        )
        box = self._box([[(bounds[0], bounds[1]), (bounds[2], bounds[3])]], 1.0)
        if box is None:
            return None
        left, top, right, bottom = box
        mask = Image.new("L", (right - left, bottom - top), 0)
        ImageDraw.Draw(mask).text(
            (col - left, row - top), text, fill=255, font=font, anchor=anchor
        )
        return self._stamp(box, mask, color, opacity)


def _dashes(
    points: Sequence[tuple[float, float]], on: float, off: float
) -> list[list[tuple[float, float]]]:
    """
    Splits a polyline into dashes.

    :param points: The polyline points.
    :param on: The dash length, in pixels.
    :param off: The gap length, in pixels.
    :return: The dash segments.
    """
    segments: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    drawing, remaining = True, on
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        length = math.hypot(x1 - x0, y1 - y0)
        position = 0.0
        while position < length:
            step = min(remaining, length - position)
            t0, t1 = position / length, (position + step) / length
            start = (x0 + t0 * (x1 - x0), y0 + t0 * (y1 - y0))
            end = (x0 + t1 * (x1 - x0), y0 + t1 * (y1 - y0))
            if drawing:
                if not current:
                    current.append(start)
                current.append(end)
            position += step
            remaining -= step
            if remaining <= 1e-9:
                if drawing and current:
                    segments.append(current)
                current = []
                drawing = not drawing
                remaining = on if drawing else off
    if drawing and len(current) > 1:
        segments.append(current)
    return segments


# Plotly anchors, mapped to Pillow's horizontal and vertical anchor characters.
_X_ANCHORS = {"left": "l", "center": "m", "right": "r"}
_Y_ANCHORS = {"top": "t", "middle": "m", "bottom": "b"}
# Plotly trace text positions, mapped to the Pillow anchor of the text box.
_TEXT_POSITIONS = {
    "top left": "rb",
    "top center": "mb",
    "top right": "lb",
    "middle left": "rm",
    "middle center": "mm",
    "middle right": "lm",
    "bottom left": "rt",
    "bottom center": "mt",
    "bottom right": "lt",
}


def _shape_stamps(canvas: _Canvas, shape: Any) -> list[_Stamp]:
    """
    Draws a figure ``path`` shape.

    :param canvas: The canvas to draw on.
    :param shape: The Plotly layout shape.
    :return: The stamps for the shape's fill and outline.
    """
    if shape.type != "path" or not shape.path:
        return []
    opacity = 1.0 if shape.opacity is None else float(shape.opacity)
    paths = [
        [canvas.to_pixel(x, y) for x, y in ring] for ring in parse_svg_path(shape.path)
    ]
    line_width = 2.0 if shape.line.width is None else float(shape.line.width)
    stamps = [
        canvas.fill(
            paths,
            parse_color(shape.fillcolor),
            opacity,
            evenodd=shape.fillrule == "evenodd",
        ),
        canvas.stroke(
            paths, parse_color(shape.line.color), line_width, opacity, shape.line.dash
        ),
    ]
    return [stamp for stamp in stamps if stamp is not None]


def _trace_stamps(canvas: _Canvas, trace: Any) -> list[_Stamp]:
    """
    Draws a figure scatter trace.

    :param canvas: The canvas to draw on.
    :param trace: The Plotly scatter trace.
    :return: The stamps for the trace's fill, lines, markers, and text.
    """
    xs = list(trace.x or [])
    ys = list(trace.y or [])
    paths = canvas.paths(xs, ys)
    if not paths:
        return []
    opacity = 1.0 if trace.opacity is None else float(trace.opacity)
    mode = trace.mode or "lines"
    stamps: list[_Stamp | None] = []
    if trace.fill == "toself":
        stamps.append(
            canvas.fill(paths, parse_color(trace.fillcolor), opacity, evenodd=False)
        )
    if "lines" in mode:
        width = 2.0 if trace.line.width is None else float(trace.line.width)
        stamps.append(
            canvas.stroke(
                paths, parse_color(trace.line.color), width, opacity, trace.line.dash
            )
        )
    if "markers" in mode:
        size = 6.0 if trace.marker.size is None else float(trace.marker.size)
        stamps.append(
            canvas.markers(paths, parse_color(trace.marker.color), size, opacity)
        )
    if "text" in mode and trace.text is not None:
        texts = [trace.text] * len(xs) if isinstance(trace.text, str) else trace.text
        anchor = _TEXT_POSITIONS.get(trace.textposition or "middle center", "mm")
        color = parse_color(trace.textfont.color) or parse_color(trace.line.color)
        size = 12.0 if trace.textfont.size is None else float(trace.textfont.size)
        for x, y, text in zip(xs, ys, texts):
            if x is not None and y is not None and text:
                stamps.append(
                    canvas.text(x, y, str(text), color, size, anchor, opacity)
                )
    return [stamp for stamp in stamps if stamp is not None]


def _annotation_stamp(canvas: _Canvas, annotation: Any) -> _Stamp | None:
    """
    Draws a figure annotation (a text label).

    :param canvas: The canvas to draw on.
    :param annotation: The Plotly annotation, or an annotation dictionary.
    :return: The stamp, or None if nothing is drawn.
    """
    if isinstance(annotation, dict):
        get = annotation.get
        font = annotation.get("font", {})
        font_color, font_size = font.get("color"), font.get("size")
    else:
        get = annotation.__getitem__
        font_color, font_size = annotation.font.color, annotation.font.size
    x, y = get("x"), get("y")
    if x is None or y is None:
        return None
    anchor = _X_ANCHORS.get(get("xanchor") or "center", "m") + _Y_ANCHORS.get(
        get("yanchor") or "middle", "m"
    )
    return canvas.text(
        float(x),
        float(y),
        str(get("text") or ""),
        parse_color(font_color),
        12.0 if font_size is None else float(font_size),
        anchor,
    )


class WorldRenderer:
    """Draws frames of a world, as the web UI shows it."""

    def __init__(
        self,
        world: World,
        *,
        width: int = 1280,
        supersample: int = 2,
        selected_robot: Robot | None = None,
        show_room_names: bool = True,
        show_location_names: bool = True,
        show_object_names: bool = True,
        show_robot_names: bool = True,
        show_collision_polygons: bool = False,
    ) -> None:
        """
        Creates a renderer for a world.

        :param world: The world to render.
        :param width: The image width, in pixels. The height follows from the
            world's extent.
        :param supersample: The factor by which frames are drawn larger, then
            reduced, to smooth edges. 1 turns smoothing off.
        :param selected_robot: The robot whose knowledge (closed hallways, known
            objects) and planner graphs are drawn, or None for the full world state.
        :param show_room_names: If True, shows room name labels.
        :param show_location_names: If True, shows location name labels.
        :param show_object_names: If True, shows object name labels.
        :param show_robot_names: If True, shows robot name labels.
        :param show_collision_polygons: If True, overlays the collision polygons.
        """
        self.world = world
        self.width = width
        self.supersample = max(1, int(supersample))
        self.selected_robot = selected_robot
        self.show_object_names = show_object_names
        self.show_robot_names = show_robot_names
        self._figure_options = {
            "show_room_names": show_room_names,
            "show_location_names": show_location_names,
            "show_object_names": show_object_names,
            "show_collision_polygons": show_collision_polygons,
        }
        self._static_key: tuple[int, int] | None = None
        self._canvas: _Canvas | None = None
        self._background: Image.Image | None = None
        self._overlay: list[_Stamp] = []
        self.static_redraws = 0

    @property
    def size(self) -> tuple[int, int]:
        """The (width, height) of rendered frames, in pixels."""
        self._update_static()
        assert self._canvas is not None
        return self._canvas.out_size

    def _update_static(self) -> None:
        """Redraws the static content if the world changed since it was drawn."""
        key = (self.world.change_count, id(self.selected_robot))
        if key == self._static_key and self._background is not None:
            return
        figure = make_figure(
            self.world,
            selected_robot=self.selected_robot,
            show_robot_names=False,
            **self._figure_options,
        )
        canvas = _Canvas(
            figure.layout.xaxis.range,
            figure.layout.yaxis.range,
            self.width,
            self.supersample,
        )
        background_color = parse_color(figure.layout.plot_bgcolor) or (
            255,
            255,
            255,
            1.0,
        )
        background = Image.new("RGB", canvas.size, background_color[:3])
        overlay: list[_Stamp] = []
        for shape in figure.layout.shapes:
            stamps = _shape_stamps(canvas, shape)
            if shape.layer == "above":
                overlay.extend(stamps)
            else:
                for stamp in stamps:
                    stamp.apply(background)
        # Planner graphs come first in the trace order, so they sit under the robots.
        for trace in figure.data[: num_graph_traces(self.selected_robot)]:
            for stamp in _trace_stamps(canvas, trace):
                stamp.apply(background)
        for annotation in figure.layout.annotations:
            label_stamp = _annotation_stamp(canvas, annotation)
            if label_stamp is not None:
                overlay.append(label_stamp)
        self._canvas = canvas
        self._background = background
        self._overlay = overlay
        self._static_key = key
        self.static_redraws += 1

    def render(self) -> Image.Image:
        """
        Draws the world as it is now.

        :return: The frame, as an RGB image of size :attr:`size`.
        """
        self._update_static()
        assert self._canvas is not None and self._background is not None
        canvas = self._canvas
        frame = self._background.copy()
        robots = list(self.world.robots)
        for robot in robots:
            for trace in _robot_traces(robot, show_object_names=self.show_object_names):
                for stamp in _trace_stamps(canvas, trace):
                    stamp.apply(frame)
        # Shapes in the figure's "above" layer and all labels go over the traces.
        for stamp in self._overlay:
            stamp.apply(frame)
        if self.show_robot_names:
            for robot in robots:
                pose = robot.get_pose()
                label = _label(
                    pose.x,
                    pose.y - 2.0 * robot.radius,
                    robot.name,
                    color_to_css(robot.color),
                    10,
                )
                label_stamp = _annotation_stamp(canvas, label)
                if label_stamp is not None:
                    label_stamp.apply(frame)
        if self.supersample > 1:
            frame = frame.reduce(self.supersample)
        return frame


def render_frame(world: World, **options: Any) -> Image.Image:
    """
    Draws a world as the web UI shows it.

    :param world: The world to render.
    :param options: Keyword options for :class:`WorldRenderer`.
    :return: The frame, as an RGB image.
    """
    return WorldRenderer(world, **options).render()


@dataclass(frozen=True)
class RecordingStats:
    """A summary of a finished recording."""

    output_file: Path
    """The video file."""
    fps: float
    """The video frame rate, in frames per second."""
    start_time: float
    """The ``time.monotonic()`` time of the first frame. Frame ``i`` shows the
    world at about ``start_time + i / fps``."""
    frames: int
    """The number of frames in the video."""
    rendered_frames: int
    """The number of frames drawn. The rest repeat the previous frame, to fill
    time slots that passed while a frame was being drawn."""
    mean_render_ms: float
    """The mean time to draw a frame, in milliseconds."""
    max_render_ms: float
    """The longest time to draw a frame, in milliseconds."""

    @property
    def duration(self) -> float:
        """The video duration, in seconds."""
        return self.frames / self.fps


class WorldRecorder:
    """
    Records a world to an MP4 video while it runs.

    A background thread draws a frame at each tick of a fixed wall-clock rate
    and pipes it to ``ffmpeg``. If drawing a frame takes longer than one tick,
    the frame is repeated for the ticks it missed, so video time matches wall
    time.

    Usage::

        with WorldRecorder(world, "run.mp4") as recorder:
            robot.execute_plan(plan)
        print(recorder.stats)
    """

    def __init__(
        self,
        world: World,
        output_file: str | Path,
        *,
        fps: float = 10.0,
        crf: int = 23,
        preset: str = "veryfast",
        ffmpeg: str = "ffmpeg",
        **render_options: Any,
    ) -> None:
        """
        Creates a recorder for a world.

        :param world: The world to record.
        :param output_file: The MP4 file to write.
        :param fps: The frame rate, in frames per second.
        :param crf: The H.264 constant rate factor (lower is higher quality).
        :param preset: The H.264 encoder preset (faster presets use less CPU).
        :param ffmpeg: The ``ffmpeg`` executable to run.
        :param render_options: Keyword options for :class:`WorldRenderer`.
        """
        self.world = world
        self.output_file = Path(output_file)
        self.fps = float(fps)
        self.crf = crf
        self.preset = preset
        self.ffmpeg = ffmpeg
        self.renderer = WorldRenderer(world, **render_options)
        self.stats: RecordingStats | None = None
        self.logger = get_global_logger()
        self._process: subprocess.Popen[bytes] | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._start_time = 0.0
        self._frames = 0
        self._render_times: list[float] = []
        self._last_frame: bytes | None = None
        self._error: BaseException | None = None

    def start(self) -> None:
        """Starts recording."""
        if self._thread is not None:
            raise RuntimeError("The recorder was already started.")
        executable = shutil.which(self.ffmpeg)
        if executable is None:
            raise FileNotFoundError(
                f"Could not find '{self.ffmpeg}'. Install ffmpeg to record videos."
            )
        width, height = self.renderer.size
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        # fmt: off
        command = [
            executable, "-y", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
            "-r", f"{self.fps:g}", "-i", "-",
            "-c:v", "libx264", "-preset", self.preset, "-crf", str(self.crf),
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(self.output_file),
        ]
        # fmt: on
        self._process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stderr=subprocess.PIPE
        )
        self._start_time = time.monotonic()
        self._thread = threading.Thread(
            target=self._run, name="pyrobosim-recorder", daemon=True
        )
        self._thread.start()

    def _draw(self) -> bytes:
        """
        Draws a frame, falling back to the previous one if drawing fails.

        Robot actions run on other threads, so a frame can be drawn while the
        world is changing. A frame that fails to draw is replaced by the
        previous one.

        :return: The frame, as raw RGB bytes.
        """
        start = time.perf_counter()
        try:
            frame = self.renderer.render().tobytes()
        except Exception as exception:  # noqa: BLE001
            if self._last_frame is None:
                raise
            self.logger.debug(f"Repeating the previous frame: {exception}")
            return self._last_frame
        self._render_times.append(time.perf_counter() - start)
        self._last_frame = frame
        return frame

    def _write(self, frame: bytes, until: float) -> None:
        """
        Writes a frame for every tick up to a time.

        :param frame: The frame, as raw RGB bytes.
        :param until: The ``time.monotonic()`` time to fill up to.
        """
        assert self._process is not None and self._process.stdin is not None
        ticks = int((until - self._start_time) * self.fps) + 1
        count = max(1, ticks - self._frames)
        for _ in range(count):
            self._process.stdin.write(frame)
        self._frames += count

    def _run(self) -> None:
        """Draws and writes frames until stopped."""
        try:
            while not self._stop.is_set():
                frame = self._draw()
                self._write(frame, time.monotonic())
                next_tick = self._start_time + self._frames / self.fps
                self._stop.wait(max(0.0, next_tick - time.monotonic()))
        except BaseException as exception:  # noqa: BLE001
            self._error = exception

    def stop(self) -> RecordingStats:
        """
        Stops recording and finishes the video.

        The last frame shows the world at the time of this call.

        :return: A summary of the recording.
        """
        if self._thread is None or self._process is None:
            raise RuntimeError("The recorder was not started.")
        if self.stats is not None:
            return self.stats
        stop_time = time.monotonic()
        self._stop.set()
        self._thread.join()
        process = self._process
        assert process.stdin is not None and process.stderr is not None
        try:
            if self._error is None:
                self._write(self._draw(), stop_time)
        except BaseException as exception:  # noqa: BLE001
            self._error = exception
        try:
            process.stdin.close()
        except BrokenPipeError:
            pass
        stderr = process.stderr.read()
        process.wait()
        if self._error is not None or process.returncode != 0:
            message = stderr.decode(errors="replace").strip()
            raise RuntimeError(
                f"Recording {self.output_file} failed: {self._error or message}"
            )
        times = self._render_times or [0.0]
        self.stats = RecordingStats(
            output_file=self.output_file,
            fps=self.fps,
            start_time=self._start_time,
            frames=self._frames,
            rendered_frames=len(self._render_times),
            mean_render_ms=1000.0 * sum(times) / len(times),
            max_render_ms=1000.0 * max(times),
        )
        return self.stats

    def __enter__(self) -> "WorldRecorder":
        self.start()
        return self

    def __exit__(self, *_: Any) -> None:
        self.stop()
