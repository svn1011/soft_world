#!/usr/bin/env python3

"""Create the post-position-control research progress presentation."""

from pathlib import Path
import subprocess
import time

import matplotlib


matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import uno
from com.sun.star.awt import Point, Size
from com.sun.star.beans import PropertyValue


ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "presentation_assets_working"
ODP_PATH = ROOT / "soft_limb_progress_update_working.odp"
PPTX_PATH = ROOT / "soft_limb_progress_update_working.pptx"
PDF_PATH = ROOT / "soft_limb_progress_update_working.pdf"

SLIDE_W = 33.867
SLIDE_H = 19.05
CM = 1000

MAROON = 0x500000
DARK = 0x17212B
MID = 0x44515E
LIGHT = 0xF4F2EF
WHITE = 0xFFFFFF
GOLD = 0xD6A84B
TEAL = 0x167D8D
RED = 0xB63A3A
GREEN = 0x3E8E62


def prop(name, value):
    item = PropertyValue()
    item.Name = name
    item.Value = value
    return item


def cm(value):
    return int(round(value * CM))


def connect_office():
    process = subprocess.Popen(
        [
            "soffice",
            "--headless",
            "--accept=socket,host=localhost,port=2083;urp;",
            "--norestore",
            "--nodefault",
            "--nofirststartwizard",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    local_context = uno.getComponentContext()
    resolver = local_context.ServiceManager.createInstanceWithContext(
        "com.sun.star.bridge.UnoUrlResolver", local_context
    )
    deadline = time.monotonic() + 15.0
    while True:
        try:
            context = resolver.resolve(
                "uno:socket,host=localhost,port=2083;urp;"
                "StarOffice.ComponentContext"
            )
            return process, context
        except Exception:
            if time.monotonic() >= deadline:
                process.terminate()
                raise
            time.sleep(0.25)


def make_validation_figures():
    ASSETS.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({
        "font.size": 11,
        "axes.titlesize": 15,
        "axes.labelsize": 12,
    })

    requested = np.linspace(0.0, 1.5, 301)
    applied = np.minimum(requested, 1.0)
    bend = 180.0 * applied
    fig, axis = plt.subplots(figsize=(9.4, 5.2), dpi=180)
    axis.plot(requested, bend, color="#500000", linewidth=3)
    axis.scatter(
        [0.1, 0.5, 1.0, 1.5], [18, 90, 180, 180],
        color=["#167d8d", "#167d8d", "#167d8d", "#b63a3a"],
        zorder=3,
    )
    axis.axvline(1.0, color="#b63a3a", linestyle="--", linewidth=1.5)
    axis.axhline(180.0, color="#b63a3a", linestyle="--", linewidth=1.5)
    axis.annotate(
        "Active cap: 1.0 N·m / 180°",
        xy=(1.0, 180), xytext=(0.57, 137),
        arrowprops={"arrowstyle": "->", "color": "#b63a3a"},
        color="#7c2020", fontweight="bold",
    )
    axis.annotate(
        "1.5 N·m request\nclips to 1.0 N·m",
        xy=(1.5, 180), xytext=(1.17, 103),
        arrowprops={"arrowstyle": "->", "color": "#b63a3a"},
        color="#7c2020",
    )
    axis.set(xlabel="Requested torque command (N·m)", ylabel="Equilibrium bend φ (degrees)")
    axis.set_title("Calibrated torque-commanded admittance")
    axis.set_xlim(0, 1.55)
    axis.set_ylim(0, 195)
    axis.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(ASSETS / "torque_mapping_180_cap_working.png", bbox_inches="tight")
    plt.close(fig)

    intervals = np.array([
        35.9999979440, 36.0000062590, 35.9999328403,
        35.9999862525, 35.9999818008,
    ])
    fig, axis = plt.subplots(figsize=(9.4, 5.2), dpi=180)
    bars = axis.bar(
        np.arange(1, 6), intervals, color="#167d8d", edgecolor="#0f5360"
    )
    axis.axhline(36.0, color="#500000", linestyle="--", linewidth=2,
                 label="Ideal 180° / 5 = 36°")
    axis.set_ylim(35.9998, 36.00015)
    axis.set_xticks(np.arange(1, 6))
    axis.set_xlabel("Physical disc interval")
    axis.set_ylabel("Measured local bend (degrees)")
    axis.set_title("Uniform-curvature distribution at the 180° endpoint")
    axis.legend(loc="upper right")
    axis.grid(axis="y", alpha=0.25)
    for bar, value in zip(bars, intervals):
        axis.text(
            bar.get_x() + bar.get_width() / 2, value + 0.000015,
            f"{value:.5f}", ha="center", va="bottom", fontsize=8,
        )
    fig.tight_layout()
    fig.savefig(ASSETS / "uniform_interval_validation_working.png", bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(9.4, 4.8), dpi=180,
                             subplot_kw={"projection": "polar"})
    rings = [
        (axes[0], [0, 120, 240], "Ring 1: one-third length", "#167d8d"),
        (axes[1], [180, 300, 60], "Ring 2: two-thirds length\n(180° set offset)", "#d6a84b"),
    ]
    for axis, angles, title, color in rings:
        radians = np.deg2rad(angles)
        axis.scatter(radians, np.ones(3), s=180, color=color, edgecolor="#17212b")
        for index, (angle, radian) in enumerate(zip(angles, radians), start=1):
            axis.text(radian, 1.25, f"G{index}\n{angle}°", ha="center", va="center")
        theta = np.linspace(0, 2 * np.pi, 300)
        axis.plot(theta, np.ones_like(theta), color="#44515e", linewidth=2)
        axis.set_ylim(0, 1.45)
        axis.set_yticklabels([])
        axis.set_xticklabels([])
        axis.grid(False)
        axis.set_title(title, pad=18, fontweight="bold")
    fig.suptitle("Six virtual strain gauges around the limb circumference", fontsize=16)
    fig.tight_layout()
    fig.savefig(ASSETS / "sensor_ring_layout_working.png", bbox_inches="tight")
    plt.close(fig)


class Deck:
    def __init__(self, document, context):
        self.document = document
        self.context = context
        self.pages = document.getDrawPages()
        self.graphic_provider = context.ServiceManager.createInstanceWithContext(
            "com.sun.star.graphic.GraphicProvider", context
        )
        self.slide_number = 0

    def new_slide(self, title, section=None):
        if self.slide_number == 0:
            page = self.pages.getByIndex(0)
        else:
            page = self.pages.insertNewByIndex(self.pages.getCount())
        page.Width = cm(SLIDE_W)
        page.Height = cm(SLIDE_H)
        self.rectangle(page, 0, 0, SLIDE_W, SLIDE_H, LIGHT, LIGHT)
        self.rectangle(page, 0, 0, 0.35, SLIDE_H, MAROON, MAROON)
        self.text(page, title, 1.15, 0.65, 30.7, 1.15, 24, MAROON, True)
        self.line(page, 1.15, 1.95, 32.2, 1.95, GOLD, 0.04)
        if section:
            self.text(page, section.upper(), 25.0, 0.25, 7.0, 0.35, 8, MID, True,
                      align=2)
        self.slide_number += 1
        self.text(page, f"Soft continuum limb research progress  •  {self.slide_number:02d}",
                  1.15, 18.45, 21, 0.28, 7.5, MID)
        self.text(page, "ETID / MSET research", 25.0, 18.45, 7.1, 0.28, 7.5, MID,
                  align=2)
        return page

    def text(self, page, content, x, y, w, h, size=18, color=DARK,
             bold=False, align=0, fill=None, margin=0.12):
        shape = self.document.createInstance("com.sun.star.drawing.TextShape")
        shape.Position = Point(cm(x), cm(y))
        shape.Size = Size(cm(w), cm(h))
        page.add(shape)
        shape.String = content
        shape.CharHeight = float(size)
        shape.CharColor = color
        shape.CharWeight = 150.0 if bold else 100.0
        shape.ParaAdjust = align
        shape.TextLeftDistance = cm(margin)
        shape.TextRightDistance = cm(margin)
        shape.TextUpperDistance = cm(margin)
        shape.TextLowerDistance = cm(margin)
        shape.LineStyle = 0
        if fill is not None:
            shape.FillStyle = 1
            shape.FillColor = fill
        else:
            shape.FillStyle = 0
        return shape

    def rectangle(self, page, x, y, w, h, fill, line=MID, radius=False):
        service = (
            "com.sun.star.drawing.RectangleShape"
            if not radius else "com.sun.star.drawing.RectangleShape"
        )
        shape = self.document.createInstance(service)
        shape.Position = Point(cm(x), cm(y))
        shape.Size = Size(cm(w), cm(h))
        shape.FillStyle = 1
        shape.FillColor = fill
        shape.LineColor = line
        shape.LineWidth = cm(0.02)
        if radius:
            shape.CornerRadius = cm(0.18)
        page.add(shape)
        return shape

    def line(self, page, x1, y1, x2, y2, color=MID, width=0.03):
        shape = self.document.createInstance("com.sun.star.drawing.LineShape")
        shape.Position = Point(cm(x1), cm(y1))
        shape.Size = Size(cm(x2 - x1), cm(y2 - y1))
        shape.LineColor = color
        shape.LineWidth = cm(width)
        page.add(shape)
        return shape

    def image(self, page, path, x, y, w, h, contain=True):
        path = Path(path)
        if contain:
            width_px, height_px = Image.open(path).size
            source_ratio = width_px / height_px
            target_ratio = w / h
            if source_ratio > target_ratio:
                fitted_h = w / source_ratio
                y += (h - fitted_h) / 2
                h = fitted_h
            else:
                fitted_w = h * source_ratio
                x += (w - fitted_w) / 2
                w = fitted_w
        shape = self.document.createInstance(
            "com.sun.star.drawing.GraphicObjectShape"
        )
        shape.Position = Point(cm(x), cm(y))
        shape.Size = Size(cm(w), cm(h))
        graphic = self.graphic_provider.queryGraphic((
            prop("URL", uno.systemPathToFileUrl(str(path.resolve()))),
        ))
        shape.Graphic = graphic
        shape.LineStyle = 0
        page.add(shape)
        return shape

    def bullet_list(self, page, bullets, x, y, w, line_height=1.0,
                    size=16, color=DARK):
        for index, bullet in enumerate(bullets):
            yy = y + index * line_height
            self.text(page, "●", x, yy + 0.04, 0.5, 0.45, size - 2, TEAL, True)
            self.text(page, bullet, x + 0.55, yy, w - 0.55, line_height,
                      size, color)

    def metric(self, page, value, label, x, y, w, accent=TEAL):
        self.rectangle(page, x, y, w, 2.25, WHITE, 0xD7D2CC, True)
        self.rectangle(page, x, y, 0.16, 2.25, accent, accent)
        self.text(page, value, x + 0.45, y + 0.28, w - 0.65, 0.75,
                  23, accent, True)
        self.text(page, label, x + 0.45, y + 1.18, w - 0.65, 0.65,
                  10.5, MID)


def build_deck(document, context):
    deck = Deck(document, context)

    page = deck.new_slide("Soft Continuum Limb: Research Progress Update", "overview")
    deck.text(page, "Progress after successful single-module position control",
              1.35, 3.0, 19.0, 1.35, 27, DARK, True)
    deck.text(page,
              "Six-disc constant-curvature simulation • hybrid torque control • "
              "virtual strain sensing • Wheatstone-bridge output • RViz reconstruction",
              1.35, 4.65, 18.5, 2.2, 17, MID)
    deck.image(page, ASSETS / "isaac_180deg_torque_working.png",
               20.2, 2.55, 12.3, 12.7)
    deck.rectangle(page, 1.35, 13.1, 17.9, 2.3, MAROON, MAROON, True)
    deck.text(page, "Current verified endpoint", 1.85, 13.45, 7.0, 0.45,
              11, WHITE, True)
    deck.text(page, "1.0 N·m  →  179.9999°", 1.85, 14.05, 12.0, 0.65,
              24, WHITE, True)
    deck.text(page, "September 2026", 1.35, 16.35, 8.0, 0.6, 13, MID)

    page = deck.new_slide("Starting Point and Scope of This Update", "milestone")
    deck.metric(page, "✓", "Single-module position control established", 1.2, 2.7, 9.2, GREEN)
    deck.metric(page, "6 discs", "Current reduced-order limb architecture", 11.1, 2.7, 9.2, TEAL)
    deck.metric(page, "180°", "Active maximum bend in both interfaces", 21.0, 2.7, 10.9, MAROON)
    deck.text(page, "Work completed after the baseline milestone", 1.25, 5.75,
              17.0, 0.7, 18, DARK, True)
    deck.bullet_list(page, [
        "Converted the twelve-disc normalization logic to a six-disc, five-interval model.",
        "Synchronized all physical intervals so the limb assumes one curve during motion.",
        "Added torque-commanded admittance while preserving position-control operation.",
        "Added two rings of three pose-derived virtual strain gauges and bridge voltages.",
        "Published robot_description and joint states for sensor-driven RViz visualization.",
        "Added reproducible regression tests and a continuously updated research record.",
    ], 1.3, 6.75, 30.2, 1.35, 15.5)

    page = deck.new_slide("Current Six-Disc Working Architecture", "simulation")
    deck.image(page, ASSETS / "isaac_straight_working.png", 1.15, 2.4, 17.4, 14.8)
    deck.text(page, "Working configuration", 19.2, 2.7, 11.5, 0.6, 18, MAROON, True)
    deck.bullet_list(page, [
        "Six physical discs: Disk_00 through Disk_05",
        "Five physical bending intervals",
        "Hanging base with active gravity compensation",
        "Direct runtime D6 joint per interval",
        "240 Hz physics and 60 Hz rendering",
        "Overdamped position drives: stiffness 6400, damping 240",
        "Twist locked; bend direction controlled by θ",
    ], 19.1, 3.8, 12.3, 1.45, 14.5)
    deck.text(page, "Captured from the current Isaac Sim working launch.",
              19.2, 15.3, 11.5, 0.8, 10.5, MID)

    page = deck.new_slide("From Position Control to Hybrid Torque Control", "control")
    deck.image(page, ASSETS / "torque_mapping_180_cap_working.png",
               1.0, 2.45, 19.2, 12.8)
    deck.text(page, "Implemented control law", 21.0, 2.75, 10.5, 0.6,
              17, MAROON, True)
    deck.text(page, "φeq = 180° · τc / 1.0 N·m", 21.0, 3.6, 10.5, 0.85,
              20, DARK, True, align=1, fill=WHITE)
    deck.bullet_list(page, [
        "0.1 N·m → 18°",
        "0.5 N·m → 90°",
        "1.0 N·m → 180°",
        "Commands beyond ±1.0 N·m are clipped",
        "2.0 N·m/s command slew limit",
        "Position commands are ignored while nonzero torque is active",
    ], 21.0, 5.0, 10.6, 1.45, 14.5)
    deck.text(page,
              "This is calibrated torque-commanded admittance—not raw open-loop torque "
              "and not a physical moment–curvature calibration.",
              21.0, 14.15, 10.5, 1.55, 11, RED, True)

    page = deck.new_slide("Constant-Curvature Motion Is Enforced Across the Limb", "validation")
    deck.image(page, ASSETS / "isaac_180deg_torque_working.png",
               1.05, 2.4, 15.0, 13.7)
    deck.image(page, ASSETS / "uniform_interval_validation_working.png",
               16.6, 2.45, 15.4, 10.1)
    deck.metric(page, "≈36° each", "Five local rotations at the 180° endpoint",
                17.1, 13.0, 7.0, TEAL)
    deck.metric(page, "0.0001°", "Measured torque-mapping tracking error",
                24.7, 13.0, 7.0, MAROON)
    deck.text(page,
              "The controller distributes one total bend uniformly; synchronized target "
              "slew prevents the former segment-by-segment transient.",
              16.8, 15.7, 14.8, 1.3, 12.5, MID)

    page = deck.new_slide("Six Virtual Strain Gauges", "sensing")
    deck.image(page, ASSETS / "sensor_ring_layout_working.png",
               1.0, 2.45, 16.9, 11.1)
    deck.text(page, "Placement", 18.7, 2.6, 6.0, 0.6, 17, MAROON, True)
    deck.bullet_list(page, [
        "Ring 1 at one-third length: 0°, 120°, 240°",
        "Ring 2 at two-thirds length: 180°, 300°, 60°",
        "Second ring uses the requested 180° set offset",
        "Readings derive from live disc poses—even during manual dragging",
    ], 18.7, 3.55, 13.0, 1.4, 14.5)
    deck.text(page, "Implemented surface-strain model", 18.7, 9.7, 10.5, 0.55,
              16, MAROON, True)
    deck.text(page, "ε(α) = r [−κx sin(α) + κy cos(α)]",
              18.7, 10.55, 12.3, 0.85, 18, DARK, True, align=1, fill=WHITE)
    deck.text(page,
              "The readings are kinematic, pose-derived estimates. Physical gauge bonding, "
              "material hysteresis, temperature drift, and calibration remain future work.",
              18.7, 12.25, 12.5, 2.1, 11.5, RED, True)

    page = deck.new_slide("Live Six-Channel Strain Response", "results")
    deck.image(page, ASSETS / "strain_six_channel_working.png",
               1.0, 2.35, 24.0, 13.9)
    deck.text(page, "Acquisition", 25.7, 2.8, 6.0, 0.5, 16, MAROON, True)
    deck.bullet_list(page, [
        "Captured from live ROS topics",
        "Torque stepped through 0, 0.5, and 1.0 N·m",
        "Two stations × three circumferential gauges",
        "Opposing signs distinguish tension and compression",
        "Signals remain available without a control command",
    ], 25.5, 3.7, 6.6, 1.65, 12.5)
    deck.text(page, "Simulated, pose-derived strain; not yet hardware validated.",
              25.6, 13.5, 6.2, 1.5, 10.5, RED, True)

    page = deck.new_slide("Wheatstone-Bridge Voltage Layer", "instrumentation")
    deck.image(page, ASSETS / "bridge_voltage_six_channel_working.png",
               1.0, 2.35, 24.0, 13.9)
    deck.text(page, "Nominal bridge model", 25.6, 2.65, 6.1, 0.8,
              14, MAROON, True)
    deck.bullet_list(page, [
        "350 Ω foil gauge",
        "Gauge factor GF = 2.0",
        "Quarter-bridge topology",
        "Excitation Vexc = 2.5 V",
        "Validity flags on separate ROS topics",
        "Non-finite model regions displayed at a marked 1.25 V boundary",
    ], 25.5, 3.75, 6.6, 1.45, 12.5)
    deck.text(page,
              "The voltage layer supports DAQ planning; it does not establish a purchased "
              "sensor’s survivability or experimental accuracy.",
              25.6, 13.2, 6.2, 2.0, 10.5, RED, True)

    page = deck.new_slide("ROS 2 and RViz Visualization Pipeline", "integration")
    boxes = [
        (1.3, "Isaac Sim\nsix-disc limb", MAROON),
        (7.9, "Live disc poses\nand commands", TEAL),
        (14.5, "Six strain +\nbridge channels", GOLD),
        (21.1, "Shape\nreconstruction", TEAL),
        (27.7, "RViz RobotModel\nand TF", MAROON),
    ]
    for x, label, color in boxes:
        deck.rectangle(page, x, 6.2, 4.8, 3.1, WHITE, color, True)
        deck.text(page, label, x + 0.2, 6.8, 4.4, 1.7, 15, color, True, align=1)
    for x in [6.25, 12.85, 19.45, 26.05]:
        deck.text(page, "→", x, 7.0, 1.3, 1.0, 28, MID, True, align=1)
    deck.bullet_list(page, [
        "robot_description is published at simulator launch.",
        "Joint states reconstruct the sensed limb rather than displaying a second physical Isaac limb.",
        "A static map → soft_limb_disk_00 transform supplies the RViz fixed frame.",
    ], 3.0, 11.2, 27.5, 1.45, 14.5)

    page = deck.new_slide("Verification Summary", "evidence")
    deck.metric(page, "179.9999°", "1.0 N·m endpoint", 1.2, 2.7, 7.2, MAROON)
    deck.metric(page, "PASS", "1.5 N·m request clipped to 1.0 N·m", 8.95, 2.7, 7.2, GREEN)
    deck.metric(page, "≈36°", "Every physical interval", 16.7, 2.7, 7.2, TEAL)
    deck.metric(page, "0.0001°", "Torque mapping error", 24.45, 2.7, 7.2, GOLD)
    deck.text(page, "Additional verified behavior", 1.3, 6.0, 11.5, 0.6,
              18, MAROON, True)
    deck.bullet_list(page, [
        "Position control still reaches the 180° endpoint.",
        "First-step X/Y targets are identical across all five physical intervals.",
        "The sixth disc is treated as the final disc—not as a midpoint from the old twelve-disc model.",
        "Syntax, topology, bridge-model, ROS build, Isaac regression, and LaTeX builds pass.",
    ], 1.3, 7.0, 19.0, 1.65, 14.5)
    deck.rectangle(page, 21.4, 6.0, 10.3, 8.3, WHITE, 0xD7D2CC, True)
    deck.text(page, "Research interpretation", 22.0, 6.55, 9.0, 0.6,
              17, MAROON, True)
    deck.text(page,
              "These results validate the implemented software mapping in unloaded, "
              "single-axis simulation. They do not yet validate material behavior, "
              "payload response, physical torque requirements, or real strain gauges.",
              22.0, 7.65, 9.0, 4.7, 14, DARK)

    page = deck.new_slide("What Changed in the Latest Iteration", "latest update")
    deck.text(page, "Previous development state", 1.4, 3.0, 13.5, 0.7,
              19, MID, True)
    deck.rectangle(page, 1.3, 4.0, 14.6, 8.8, WHITE, 0xD7D2CC, True)
    deck.bullet_list(page, [
        "Torque range temporarily extended to 1.5 N·m",
        "Accumulated bend measurement demonstrated 270°",
        "Useful as historical topology evidence",
        "Outside the intended active research range",
    ], 2.0, 4.8, 13.0, 1.65, 15)
    deck.text(page, "Current active working state", 17.5, 3.0, 13.5, 0.7,
              19, MAROON, True)
    deck.rectangle(page, 17.4, 4.0, 14.6, 8.8, WHITE, MAROON, True)
    deck.bullet_list(page, [
        "Maximum bend restored to ±180°",
        "Torque command capped at ±1.0 N·m",
        "Over-limit commands clamp safely and visibly",
        "Verifier checks requested vs. applied torque",
        "README, validation log, and research document synchronized",
    ], 18.1, 4.8, 13.0, 1.52, 15)
    deck.text(page,
              "Decision: retain the 270° result only as historical pre-cap evidence; "
              "use 180° as the active experimental boundary.",
              4.0, 14.25, 26.0, 1.45, 16, WHITE, True, align=1, fill=MAROON)

    page = deck.new_slide("Third Objective: Three Modules in Series", "soft_3_working")
    deck.image(page, ASSETS / "soft_3_working_snake_pose.png",
               1.1, 2.35, 13.7, 13.7)
    deck.text(page, "soft_3_working architecture", 15.7, 2.55, 14.8, 0.7,
              19, MAROON, True)
    deck.bullet_list(page, [
        "Three serial six-disc modules: 18 physical discs",
        "Five actuated intervals per module: 15 total",
        "Two fixed connectors transmit pose between modules",
        "The first disc is fixed in air; the chain hangs under gravity",
        "Three independent φ-torque commands",
        "Three independent θ bending directions",
        "Same synchronized constant-curvature target inside every module",
        "Source, launch, control, plotting, verification, and USD files use the soft_3_working name",
    ], 15.7, 3.55, 15.8, 1.30, 13.2)
    deck.text(page,
              "Live Isaac Sim capture using the Grey Studio light rig; overlapping "
              "discs reflect the commanded compound three-dimensional pose.",
              15.8, 14.2, 15.2, 1.5, 11, MID)

    page = deck.new_slide("Independent Three-Module Control: Verified", "validation")
    deck.metric(page, "90.0047°", "Module 1: 0.50 N·m, θ = 0°",
                1.2, 2.7, 9.8, MAROON)
    deck.metric(page, "134.9999°", "Module 2: 0.75 N·m, θ = 90°",
                11.7, 2.7, 9.8, TEAL)
    deck.metric(page, "179.9999°", "Module 3: 1.00 N·m, θ = −90°",
                22.2, 2.7, 9.8, GOLD)
    deck.text(page, "12-second headless regression", 1.4, 6.05, 13.0, 0.7,
              18, MAROON, True)
    deck.bullet_list(page, [
        "All five first-step interval targets matched within each module.",
        "Final-second ranges: 0.0001642°, 0.0000568°, 0.0000713°.",
        "Connector residual bends: 0.0000672° and 0.0000394°.",
        "Detected all 18 gauge publishers.",
        "Overall verifier result: PASS.",
    ], 1.4, 7.15, 20.0, 1.55, 14.5)
    deck.rectangle(page, 22.4, 6.1, 9.5, 8.2, WHITE, 0xD7D2CC, True)
    deck.text(page, "Research boundary", 23.0, 6.7, 8.2, 0.6,
              17, MAROON, True)
    deck.text(page,
              "This validates independent software commands, synchronized interval "
              "targets, connector rigidity, and numerical stability. It does not "
              "calibrate physical torque, material response, or payload capacity.",
              23.0, 7.8, 8.1, 4.8, 13.2, DARK)

    page = deck.new_slide("Minimum-Energy Horizontal Tip Circle", "Linked branch")
    deck.metric(page, "212.34 mm", "measured mean radius",
                1.2, 2.7, 9.8, MAROON)
    deck.metric(page, "0.71 mm", "height peak-to-peak",
                11.7, 2.7, 9.8, TEAL)
    deck.metric(page, "0.1254 N·m", "maximum module command",
                22.2, 2.7, 9.8, GOLD)
    deck.text(page, "Constrained constant-curvature solution", 1.4, 6.1,
              15.0, 0.7, 18, MAROON, True)
    deck.bullet_list(page, [
        "Minimizes 0.5 Σ wᵢτᵢ² subject to requested tip radius and height.",
        "All 15 bending intervals and both rigid connector spans are modeled.",
        "Optimized torques stay constant while the common bending plane rotates.",
        "Default radius: 212.5 mm = half of the 425 mm modeled limb length.",
        "Path height is 0.40 m; the 54 s period preserves the prior tip speed.",
        "Full revolution passed: 0.76 mm radial and 0.71 mm height variation.",
    ], 1.4, 7.15, 20.0, 1.4, 13.6)
    deck.rectangle(page, 22.4, 6.1, 9.5, 8.2, WHITE, 0xD7D2CC, True)
    deck.text(page, "Research boundary", 23.0, 6.7, 8.2, 0.6,
              17, MAROON, True)
    deck.text(page,
              "Energy is a normalized squared-command/elastic proxy. It is not "
              "electrical consumption, actuator work, or calibrated mechanical "
              "energy in joules.",
              23.0, 7.8, 8.1, 4.8, 13.2, DARK)

    page = deck.new_slide("Eighteen-Channel Bridge-Voltage Instrumentation", "sensing")
    deck.image(page, ASSETS / "soft_3_working_18_voltage.png",
               0.9, 2.25, 24.9, 14.25)
    deck.text(page, "Repeated sensing layout", 26.25, 2.55, 5.8, 0.75,
              15.5, MAROON, True)
    deck.bullet_list(page, [
        "Six gauges per module",
        "Two axial stations",
        "Three gauges per ring",
        "18 bridge-voltage channels",
        "18 validity flags",
        "Quarter bridge: 350 Ω, GF 2.0, 2.5 V excitation",
        "All 18 voltage graphs open automatically with the simulation launch",
        "Pose-derived readings remain available without a command",
    ], 26.1, 3.6, 6.0, 1.34, 11.5)
    deck.text(page,
              "Live voltage graph captured from /soft_snake/module_N/sensors/... topics.",
              26.2, 14.8, 5.8, 1.2, 10.5, MID)

    page = deck.new_slide("Twelve-Disc Quadruped: Hybrid Control", "integration")
    deck.metric(page, "5", "12-disc robot members", 1.2, 2.7, 9.8, MAROON)
    deck.metric(page, "11", "physical intervals per member",
                11.7, 2.7, 9.8, TEAL)
    deck.metric(page, "1 N·m → 180°", "command-admittance scale",
                22.2, 2.7, 9.8, GOLD)
    deck.text(page, "Stable original assembly with a new control interface", 1.4, 6.05,
              19.5, 0.7, 18, MAROON, True)
    deck.bullet_list(page, [
        "Working simulator regenerated directly from the old grounded quadruped.",
        "USD, height, positions, ±90° placement, attachments, and camera are retained.",
        "Left legs: net 180° axial rotation; right legs: net 360° after an additional 180°.",
        "Joint limits, friction, solver settings, and body lock are unchanged.",
        "Phi is a bounded torque-admittance command; theta remains held position.",
        "The base gets 1/11 of total bend; each later A/B joint gets 1/22.",
        "Torque is limited to ±1.0 N·m and slewed at 10.0 N·m/s.",
        "Immense-stiffness gains: roll 20M/42k; yaw 30M/51k.",
        "Rigid-body damping is 5.0 linear / 25.0 angular; neutral theta is 0°.",
        "The downward-bending standing posture uses positive torque on all four legs.",
        "Disc-orientation PI feedback compensates load: Kp 1.2, Ki 4.0/s.",
        "Responsive playback: 120 Hz physics, render every 2 steps (60 Hz), 24/6 solver.",
        "+0.50 N·m: 5 s near target; 10 s final bends 89.34° to 89.95°.",
    ], 1.4, 7.1, 21.0, 1.15, 12.8)
    deck.rectangle(page, 23.3, 6.1, 8.6, 8.2, WHITE, 0xD7D2CC, True)
    deck.text(page, "Validation boundary", 23.9, 6.7, 7.4, 0.6,
              17, MAROON, True)
    deck.text(page,
              "The 1 N·m → 180° law is a simulated command mapping, not raw or "
              "calibrated physical torque. Leg-angle tracking is simulation-only; "
              "hardware validation, broad contact stability, and locomotion remain "
              "future tests.",
              23.9, 7.8, 7.2, 4.8, 13.0, DARK)

    page = deck.new_slide("Next Research Steps", "forward plan")
    steps = [
        ("01", "Physical gauge selection", "Choose a commercial gauge, substrate, adhesive, and temperature compensation strategy."),
        ("02", "Bench calibration", "Measure real strain and bridge voltage against known curvature and loading."),
        ("03", "Independent shape validation", "Compare reconstructed curvature with motion capture or vision—not the same pose source."),
        ("04", "Loaded and directional trials", "Repeat across θ, negative torque, payloads, disturbances, and repeated cycles."),
        ("05", "Uncertainty and statistics", "Report repeatability, hysteresis, drift, confidence intervals, and failure limits."),
    ]
    for index, (number, title, detail) in enumerate(steps):
        y = 2.65 + index * 2.75
        deck.rectangle(page, 1.3, y, 2.0, 1.75, MAROON, MAROON, True)
        deck.text(page, number, 1.35, y + 0.38, 1.9, 0.7, 21, WHITE, True, align=1)
        deck.text(page, title, 3.8, y + 0.12, 7.2, 1.25,
                  14.5, MAROON, True)
        deck.text(page, detail, 11.4, y + 0.12, 19.9, 1.3,
                  12.0, DARK)

    page = deck.new_slide("Current Outcome", "conclusion")
    deck.text(page, "A reproducible six-disc research platform now connects",
              2.0, 3.1, 29.5, 0.9, 25, MAROON, True, align=1)
    outcomes = [
        ("CONTROL", "Position + torque admittance + minimum-energy circle"),
        ("SHAPE", "Synchronized five-interval constant-curvature target"),
        ("SENSING", "Six pose-derived strain channels + bridge voltages"),
        ("INTEGRATION", "Verified limb reused in a five-member quadruped"),
        ("EVIDENCE", "Automated tests and research documentation"),
    ]
    for index, (heading, detail) in enumerate(outcomes):
        y = 5.0 + index * 2.05
        deck.text(page, heading, 4.0, y, 6.0, 0.55, 12, TEAL, True)
        deck.text(page, detail, 10.2, y - 0.08, 19.0, 0.7, 17, DARK)
        if index < len(outcomes) - 1:
            deck.line(page, 4.0, y + 1.05, 29.2, y + 1.05, 0xD7D2CC, 0.02)
    deck.text(page,
              "Active boundary: ±1.0 N·m and ±180°  •  Next milestone: physical sensor calibration",
              3.0, 16.3, 27.5, 1.0, 16, WHITE, True, align=1, fill=MAROON)


def main():
    required = [
        ASSETS / "isaac_straight_working.png",
        ASSETS / "isaac_180deg_torque_working.png",
        ASSETS / "strain_six_channel_working.png",
        ASSETS / "bridge_voltage_six_channel_working.png",
        ASSETS / "soft_3_working_snake_pose.png",
        ASSETS / "soft_3_working_18_voltage.png",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing captured assets: " + ", ".join(missing))

    make_validation_figures()
    process, context = connect_office()
    document = None
    try:
        desktop = context.ServiceManager.createInstanceWithContext(
            "com.sun.star.frame.Desktop", context
        )
        document = desktop.loadComponentFromURL(
            "private:factory/simpress", "_blank", 0, ()
        )
        build_deck(document, context)
        document.storeAsURL(
            uno.systemPathToFileUrl(str(ODP_PATH.resolve())),
            (prop("FilterName", "impress8"),),
        )
        document.storeToURL(
            uno.systemPathToFileUrl(str(PPTX_PATH.resolve())),
            (prop("FilterName", "Impress Office Open XML"),),
        )
        document.storeToURL(
            uno.systemPathToFileUrl(str(PDF_PATH.resolve())),
            (prop("FilterName", "impress_pdf_Export"),),
        )
        print(f"Created {PPTX_PATH}")
        print(f"Created {ODP_PATH}")
        print(f"Created {PDF_PATH}")
    finally:
        if document is not None:
            document.close(True)
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()


if __name__ == "__main__":
    main()
