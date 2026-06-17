#!/usr/bin/env python3
"""Generate Traffic Light Detection presentation slides as a PDF."""

from __future__ import annotations

import textwrap
from pathlib import Path

from PIL import Image
from reportlab.lib import colors
from reportlab.lib.pagesizes import landscape
from reportlab.lib.units import inch
from reportlab.pdfgen import canvas


PROJECT_DIR = Path(__file__).resolve().parents[3]
OUTPUT_DIR = PROJECT_DIR / "outputs" / "presentation"
SLIDE_PDF = OUTPUT_DIR / "traffic_light_detection_slides.pdf"
NOTES_MD = OUTPUT_DIR / "traffic_light_detection_slide_notes.md"
MAP_IMAGE = PROJECT_DIR / "outputs" / "map_overview" / "town10hd_path_signals_full.png"
OVERLAY_IMAGE = (
    PROJECT_DIR / "outputs" / "projection_validation" / "keyframes" / "frame_0000.jpg"
)

PAGE_W, PAGE_H = landscape((13.333 * inch, 7.5 * inch))
ORANGE = colors.HexColor("#e87700")
ORANGE_DARK = colors.HexColor("#c95514")
TEAL = colors.HexColor("#168fb5")
INK = colors.HexColor("#231925")
MUTED = colors.HexColor("#666666")
LIGHT = colors.HexColor("#f4f4f2")
LANE_GRAY = colors.HexColor("#596267")
GREEN = colors.HexColor("#20a65a")


def draw_theme(c: canvas.Canvas, title: str, page_no: int) -> None:
    c.setFillColor(colors.white)
    c.rect(0, 0, PAGE_W, PAGE_H, stroke=0, fill=1)
    c.setFillColor(ORANGE)
    c.rect(0, PAGE_H - 0.62 * inch, PAGE_W * 0.70, 0.62 * inch, stroke=0, fill=1)
    c.setFillColor(colors.HexColor("#ffb15a"))
    c.rect(PAGE_W * 0.70, PAGE_H - 0.62 * inch, PAGE_W * 0.10, 0.62 * inch, stroke=0, fill=1)
    c.setFillColor(ORANGE)
    c.rect(PAGE_W * 0.30, 0, PAGE_W * 0.70, 0.28 * inch, stroke=0, fill=1)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 11)
    c.drawRightString(PAGE_W - 0.25 * inch, PAGE_H - 0.28 * inch, str(page_no))
    c.setFillColor(colors.white)
    c.setFont("Helvetica-Bold", 25)
    c.drawString(0.22 * inch, PAGE_H - 0.43 * inch, title)
    c.setFillColor(ORANGE)
    c.setFont("Helvetica-Bold", 22)
    c.drawString(0.28 * inch, 0.50 * inch, "KMUTT")
    c.setFont("Helvetica-Bold", 12)
    c.drawString(1.25 * inch, 0.60 * inch, "FIBO")
    c.setFont("Helvetica", 8.5)
    c.drawString(1.88 * inch, 0.58 * inch, "Institute of Field Robotics")


def draw_section(c: canvas.Canvas, title: str, page_no: int) -> None:
    c.setFillColor(colors.white)
    c.rect(0, 0, PAGE_W, PAGE_H, stroke=0, fill=1)
    c.setFillColor(ORANGE)
    c.rect(0, PAGE_H - 0.42 * inch, PAGE_W * 0.46, 0.42 * inch, stroke=0, fill=1)
    c.setFillColor(colors.HexColor("#ffb15a"))
    c.rect(PAGE_W * 0.46, PAGE_H - 0.42 * inch, PAGE_W * 0.14, 0.42 * inch, stroke=0, fill=1)
    c.setFillColor(ORANGE)
    c.rect(PAGE_W * 0.30, 0, PAGE_W * 0.70, 0.28 * inch, stroke=0, fill=1)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 11)
    c.drawRightString(PAGE_W - 0.25 * inch, PAGE_H - 0.28 * inch, str(page_no))
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 47)
    c.drawCentredString(PAGE_W / 2, PAGE_H / 2 + 0.20 * inch, title)
    c.setFillColor(ORANGE_DARK)
    c.rect(PAGE_W / 2 - 1.55 * inch, PAGE_H / 2 + 1.16 * inch, 3.1 * inch, 0.18 * inch, stroke=0, fill=1)
    c.rect(PAGE_W / 2 - 1.55 * inch, PAGE_H / 2 - 0.82 * inch, 3.1 * inch, 0.18 * inch, stroke=0, fill=1)
    c.setFillColor(ORANGE)
    c.setFont("Helvetica-Bold", 22)
    c.drawString(0.28 * inch, 0.50 * inch, "KMUTT")
    c.setFont("Helvetica-Bold", 12)
    c.drawString(1.25 * inch, 0.60 * inch, "FIBO")
    c.setFont("Helvetica", 8.5)
    c.drawString(1.88 * inch, 0.58 * inch, "Institute of Field Robotics")


def textbox(c: canvas.Canvas, x: float, y: float, w: float, h: float, title: str, lines: list[str]) -> None:
    c.setFillColor(LIGHT)
    c.roundRect(x, y, w, h, 8, stroke=0, fill=1)
    c.setStrokeColor(colors.HexColor("#d9d9d9"))
    c.roundRect(x, y, w, h, 8, stroke=1, fill=0)
    c.setFillColor(TEAL)
    c.setFont("Helvetica-Bold", 16)
    c.drawString(x + 0.18 * inch, y + h - 0.34 * inch, title)
    c.setFillColor(INK)
    c.setFont("Helvetica", 11)
    ty = y + h - 0.64 * inch
    for line in lines:
        for wrapped in textwrap.wrap(line, width=48):
            c.drawString(x + 0.23 * inch, ty, wrapped)
            ty -= 0.20 * inch
        ty -= 0.08 * inch


def bullet_list(c: canvas.Canvas, x: float, y: float, lines: list[str], size: int = 17, leading: float = 0.38) -> None:
    c.setFont("Helvetica", size)
    c.setFillColor(INK)
    ty = y
    for line in lines:
        c.setFillColor(ORANGE)
        c.circle(x, ty + 0.06 * inch, 0.045 * inch, stroke=0, fill=1)
        c.setFillColor(INK)
        for i, wrapped in enumerate(textwrap.wrap(line, width=64)):
            c.drawString(x + 0.20 * inch, ty - i * leading * inch, wrapped)
        ty -= (leading * max(1, len(textwrap.wrap(line, width=64))) + 0.13) * inch


def draw_arrow(c: canvas.Canvas, x1: float, y1: float, x2: float, y2: float, color=TEAL) -> None:
    c.setStrokeColor(color)
    c.setFillColor(color)
    c.setLineWidth(3)
    c.line(x1, y1, x2, y2)
    angle = 0 if x2 >= x1 else 3.14159
    dx = 0.16 * inch
    dy = 0.08 * inch
    c.line(x2, y2, x2 - dx, y2 + dy)
    c.line(x2, y2, x2 - dx, y2 - dy)


def image_box(c: canvas.Canvas, path: Path, x: float, y: float, w: float, h: float) -> None:
    if not path.exists():
        c.setFillColor(LIGHT)
        c.rect(x, y, w, h, stroke=0, fill=1)
        c.setFillColor(MUTED)
        c.setFont("Helvetica", 12)
        c.drawCentredString(x + w / 2, y + h / 2, f"Missing image: {path.name}")
        return
    with Image.open(path) as img:
        iw, ih = img.size
    scale = min(w / iw, h / ih)
    dw = iw * scale
    dh = ih * scale
    c.drawImage(str(path), x + (w - dw) / 2, y + (h - dh) / 2, dw, dh, preserveAspectRatio=True, mask="auto")
    c.setStrokeColor(colors.HexColor("#dddddd"))
    c.rect(x, y, w, h, stroke=1, fill=0)


def make_pdf() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    c = canvas.Canvas(str(SLIDE_PDF), pagesize=(PAGE_W, PAGE_H))

    draw_section(c, "TRAFFIC LIGHT DETECTION", 11)
    c.showPage()

    draw_theme(c, "Traffic Light Detection", 12)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 30)
    c.drawString(0.75 * inch, 6.25 * inch, "Goal: identify the light that controls the ego lane")
    bullet_list(
        c,
        0.95 * inch,
        5.48 * inch,
        [
            "Do not use CARLA traffic-light state as perception input.",
            "Use HD map + localization + planned path to find the relevant light.",
            "Project the expected light-head ROI into the camera image.",
            "Run YOLO/color classification only inside the ROI when it is visible.",
        ],
        size=18,
    )
    textbox(
        c,
        8.05 * inch,
        1.55 * inch,
        4.40 * inch,
        4.25 * inch,
        "Inputs",
        [
            "Town10HD OpenDRIVE map",
            "Future odom path",
            "Ego odometry pose",
            "Camera intrinsics + extrinsics",
            "CARLA light-box geometry",
        ],
    )
    c.showPage()

    draw_theme(c, "Traffic Light Detection", 13)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 26)
    c.drawString(0.70 * inch, 6.18 * inch, "1) Associate traffic lights from the future path")
    steps = [
        ("Future odom path", "Use recorded/planned future poses"),
        ("Road / lane / turn", "Match path to OpenDRIVE lanes"),
        ("signalReference", "Find map references on that lane"),
        ("Physical signal", "Use id + lane validity + same path heading"),
    ]
    x0 = 0.65 * inch
    y0 = 4.20 * inch
    box_w = 2.55 * inch
    for i, (title, desc) in enumerate(steps):
        x = x0 + i * 3.05 * inch
        textbox(c, x, y0, box_w, 1.15 * inch, title, [desc])
        if i < len(steps) - 1:
            draw_arrow(c, x + box_w + 0.10 * inch, y0 + 0.55 * inch, x + 2.95 * inch, y0 + 0.55 * inch)
    bullet_list(
        c,
        0.95 * inch,
        2.55 * inch,
        [
            "Lane validity prevents selecting lights for neighboring lanes.",
            "turnRelation filters Right / Left / Straight references.",
            "For this map, signal heading is treated as traffic-flow direction.",
        ],
        size=15,
    )
    c.showPage()

    draw_theme(c, "Traffic Light Detection", 14)
    c.setFont("Helvetica-Bold", 26)
    c.setFillColor(INK)
    c.drawString(0.70 * inch, 6.18 * inch, "2) Use the real light-head position, not the map anchor")
    image_box(c, MAP_IMAGE, 0.70 * inch, 1.00 * inch, 5.15 * inch, 4.80 * inch)
    textbox(
        c,
        6.25 * inch,
        3.60 * inch,
        5.95 * inch,
        2.15 * inch,
        "Why light boxes?",
        [
            "OpenDRIVE signal position is an actor/map anchor.",
            "The visible lamp heads are offset from the anchor.",
            "CARLA get_light_boxes() provides the 3D centers of the light heads.",
        ],
    )
    textbox(
        c,
        6.25 * inch,
        1.10 * inch,
        5.95 * inch,
        2.10 * inch,
        "Corrections found",
        [
            "Traffic-light box Y must be flipped into the odom/map frame.",
            "cam_front_right TF translation is x=1.5, y=-0.25, z=1.5.",
            "Image horizontal projection needs h_sign=flip for this bag.",
        ],
    )
    c.showPage()

    draw_theme(c, "Traffic Light Detection", 15)
    c.setFont("Helvetica-Bold", 26)
    c.setFillColor(INK)
    c.drawString(0.70 * inch, 6.18 * inch, "3) Project the selected light into the camera frame")
    c.setFont("Helvetica-Bold", 17)
    labels = ["World / HD map", "Ego vehicle", "Camera frame", "Image pixel"]
    for i, label in enumerate(labels):
        x = 0.85 * inch + i * 3.05 * inch
        c.setFillColor(LIGHT)
        c.roundRect(x, 4.52 * inch, 2.22 * inch, 0.76 * inch, 8, stroke=0, fill=1)
        c.setStrokeColor(TEAL)
        c.roundRect(x, 4.52 * inch, 2.22 * inch, 0.76 * inch, 8, stroke=1, fill=0)
        c.setFillColor(INK)
        c.drawCentredString(x + 1.11 * inch, 4.76 * inch, label)
        if i < len(labels) - 1:
            draw_arrow(c, x + 2.33 * inch, 4.90 * inch, x + 2.92 * inch, 4.90 * inch)
    c.setFont("Courier-Bold", 14)
    c.setFillColor(INK)
    equations = [
        "p_ego = R_world_ego.T @ (p_world - t_ego)",
        "p_cam = R_ego_cam.T @ (p_ego - t_cam)",
        "u = fx * x_opt / z_opt + cx",
        "v = fy * y_opt / z_opt + cy",
    ]
    y = 3.42 * inch
    for eq in equations:
        c.drawString(1.00 * inch, y, eq)
        y -= 0.35 * inch
    bullet_list(
        c,
        7.35 * inch,
        3.25 * inch,
        [
            "Projection is recalculated every camera frame.",
            "The traffic light is static in the world.",
            "The ROI moves because the ego camera moves.",
            "If status=ready, the ROI is suitable for YOLO/color detection.",
        ],
        size=14,
        leading=0.30,
    )
    c.showPage()

    draw_theme(c, "Traffic Light Detection", 16)
    c.setFont("Helvetica-Bold", 26)
    c.setFillColor(INK)
    c.drawString(0.70 * inch, 6.18 * inch, "4) Validation result: dynamic ROI on the correct light")
    image_box(c, OVERLAY_IMAGE, 0.70 * inch, 1.18 * inch, 6.15 * inch, 4.60 * inch)
    textbox(
        c,
        7.15 * inch,
        3.72 * inch,
        5.20 * inch,
        2.05 * inch,
        "Runtime behavior",
        [
            "Every frame uses current odometry to update camera pose.",
            "The projected ROI remains on the selected light as the vehicle moves.",
            "YOLO receives a small search area instead of the full image.",
        ],
    )
    textbox(
        c,
        7.15 * inch,
        1.15 * inch,
        5.20 * inch,
        2.12 * inch,
        "Validated sequence",
        [
            "Path-selected signals in this bag: 957 -> 952 -> 944.",
            "ROI source: CARLA light-box centers.",
            "Not using CARLA traffic-light color/status topics.",
        ],
    )
    c.showPage()

    draw_theme(c, "Traffic Light Detection", 17)
    c.setFont("Helvetica-Bold", 26)
    c.setFillColor(INK)
    c.drawString(0.70 * inch, 6.18 * inch, "Implementation summary and issues found")
    textbox(
        c,
        0.78 * inch,
        3.58 * inch,
        5.62 * inch,
        2.20 * inch,
        "Final pipeline",
        [
            "Path -> lane/reference -> traffic signal id.",
            "Light-box geometry -> 3D lamp-head center.",
            "World-to-camera projection -> ROI.",
            "YOLO/color model -> traffic-light state.",
        ],
    )
    textbox(
        c,
        6.70 * inch,
        3.58 * inch,
        5.62 * inch,
        2.20 * inch,
        "Main issues",
        [
            "OpenDRIVE signal anchor is not the visible lamp head.",
            "CARLA light-box Y axis must be converted to the map/odom frame.",
            "Camera TF in objects.json had to be corrected.",
            "Image horizontal sign needed to be flipped for this dataset.",
        ],
    )
    c.setFillColor(TEAL)
    c.roundRect(1.15 * inch, 1.25 * inch, 10.95 * inch, 1.00 * inch, 10, stroke=0, fill=1)
    c.setFillColor(colors.white)
    c.setFont("Helvetica-Bold", 18)
    c.drawCentredString(
        PAGE_W / 2,
        1.62 * inch,
        "Output to controller: GREEN = go, YELLOW = slow down, RED = stop",
    )
    c.showPage()
    c.save()


def make_notes() -> None:
    notes = """# Traffic Light Detection slide notes

## Slide 12 - Goal
Explain that this module does not simply detect every traffic light in the image. The first task is to identify which traffic light controls the ego lane, then use ML only for the visual state.

## Slide 13 - Path Association
Use future odometry/path to match road, lane, and turn in the OpenDRIVE map. The selected signalReference gives the traffic light that controls that movement.

## Slide 14 - Light Head Geometry
The OpenDRIVE signal point is not the visible lamp head. We exported CARLA light boxes and use the light-head center as the 3D point for projection.

## Slide 15 - Projection
The ROI is dynamic. It is recalculated every frame from the current ego pose, camera transform, and fixed light position in the world.

## Slide 16 - Validation
Show the overlay video. The boxes are not YOLO results; they are expected ROIs from HD map + odometry + camera calibration.

## Slide 17 - Issues
Mention the practical calibration issues found: map anchor vs lamp head, Y-axis conversion, camera TF correction, and image horizontal flip.
"""
    NOTES_MD.write_text(notes, encoding="utf-8")


def main() -> None:
    make_pdf()
    make_notes()
    print(f"slides: {SLIDE_PDF}")
    print(f"notes: {NOTES_MD}")


if __name__ == "__main__":
    main()
