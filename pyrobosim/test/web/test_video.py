"""
Tests for rendering worlds to images and recording them to video.
"""

import json
import shutil
import subprocess
import time
from pathlib import Path

import pytest
from PIL import Image, ImageStat

from pyrobosim.core import World
from pyrobosim.web.video import (
    WorldRecorder,
    WorldRenderer,
    parse_color,
    parse_svg_path,
    render_frame,
)


def _has_color_near(
    image: Image.Image,
    center: tuple[float, float],
    color: tuple[int, ...],
    radius: int = 12,
    tolerance: int = 40,
) -> bool:
    """Checks whether any pixel near a point is close to a color."""
    cx, cy = int(round(center[0])), int(round(center[1]))
    for x in range(max(0, cx - radius), min(image.width, cx + radius + 1)):
        for y in range(max(0, cy - radius), min(image.height, cy + radius + 1)):
            pixel = image.getpixel((x, y))
            assert isinstance(pixel, tuple)
            if all(abs(p - c) <= tolerance for p, c in zip(pixel, color)):
                return True
    return False


def test_parse_color() -> None:
    """CSS colors from the figure parse to (R, G, B, alpha)."""
    assert parse_color("rgb(204, 0, 204)") == (204, 0, 204, 1.0)
    assert parse_color("rgba(204, 0, 204, 0.5)") == (204, 0, 204, 0.5)
    assert parse_color("white") == (255, 255, 255, 1.0)
    assert parse_color(None) is None


def test_parse_svg_path() -> None:
    """Figure SVG paths parse to closed rings, one per subpath."""
    rings = parse_svg_path("M0,0 L1,0 L1,1 Z M0.2,0.2 L0.4,0.2 L0.4,0.4 Z")
    assert rings == [
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)],
        [(0.2, 0.2), (0.4, 0.2), (0.4, 0.4), (0.2, 0.2)],
    ]


def test_render_frame(test_world: World) -> None:
    """A frame has the requested width, even dimensions, and shows the robot."""
    renderer = WorldRenderer(test_world, width=640)
    frame = renderer.render()
    assert frame.mode == "RGB"
    assert frame.size == renderer.size
    assert frame.width == 640
    assert frame.height % 2 == 0
    assert render_frame(test_world, width=320).width == 320

    robot = test_world.robots[0]
    pose = robot.get_pose()
    assert renderer._canvas is not None
    col, row = renderer._canvas.to_pixel(pose.x, pose.y)
    scale = renderer.supersample
    color = tuple(int(round(255 * channel)) for channel in robot.color)
    assert _has_color_near(frame, (col / scale, row / scale), color)


def test_static_content_redrawn_only_on_change(test_world: World) -> None:
    """Static content is redrawn only when the world's change count changes."""
    renderer = WorldRenderer(test_world, width=320, supersample=1)
    renderer.render()
    renderer.render()
    assert renderer.static_redraws == 1
    test_world.mark_changed()
    renderer.render()
    assert renderer.static_redraws == 2


def test_closed_location_is_filled(test_world: World) -> None:
    """A closed location is drawn filled, as in the web UI."""
    renderer = WorldRenderer(test_world, width=640, supersample=1)
    trash = test_world.get_location_by_name("trash")
    assert trash is not None and not trash.is_open

    def brightness() -> float:
        frame = renderer.render()
        assert renderer._canvas is not None
        xmin, ymin, xmax, ymax = trash.polygon.bounds
        left, top = renderer._canvas.to_pixel(xmin, ymax)
        right, bottom = renderer._canvas.to_pixel(xmax, ymin)
        crop = frame.crop((int(left), int(top), int(right), int(bottom)))
        return float(ImageStat.Stat(crop.convert("L")).mean[0])

    closed = brightness()
    assert test_world.open_location(trash).is_success()
    try:
        assert brightness() > closed
    finally:
        test_world.close_location(trash)


def test_recorder_requires_ffmpeg(test_world: World, tmp_path: Path) -> None:
    """Starting a recording without ffmpeg fails clearly."""
    recorder = WorldRecorder(test_world, tmp_path / "run.mp4", ffmpeg="no-such-ffmpeg")
    with pytest.raises(FileNotFoundError):
        recorder.start()
    with pytest.raises(RuntimeError):
        recorder.stop()


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg is not installed")
def test_recorder_writes_video(test_world: World, tmp_path: Path) -> None:
    """A recording writes one frame per tick of wall-clock time."""
    output_file = tmp_path / "run.mp4"
    with WorldRecorder(test_world, output_file, fps=10.0, width=320) as recorder:
        time.sleep(0.5)
    stats = recorder.stats
    assert stats is not None
    assert output_file.stat().st_size > 0
    assert 5 <= stats.frames <= 10
    assert 1 <= stats.rendered_frames <= stats.frames
    assert stats.duration == pytest.approx(stats.frames / 10.0)

    if shutil.which("ffprobe") is not None:
        result = subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-count_frames",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=width,height,nb_read_frames",
                "-of",
                "json",
                str(output_file),
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        stream = json.loads(result.stdout)["streams"][0]
        assert (stream["width"], stream["height"]) == recorder.renderer.size
        assert int(stream["nb_read_frames"]) == stats.frames
