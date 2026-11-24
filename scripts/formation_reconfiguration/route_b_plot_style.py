"""Publication-grade IEEE / AST plotting style, colorblind-safe palettes, and helpers
for formation reconfiguration and swarm search figures.

Enforces TrueType (Type 42) font embedding in PDF/PS outputs, consistent
single/two-column IEEE/Elsevier sizing, Okabe-Ito & Tol colorblind-safe palettes,
clean uncertainty-ellipse rendering, and automated text-clipping checks.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, FancyArrowPatch, Polygon

# Standard IEEE / Elsevier two-column widths (inches)
SINGLE_COL_WIDTH = 3.50
DOUBLE_COL_WIDTH = 7.16

# Muted publication figure palette. Legacy keys are kept so every script
# inherits the same restrained family: steel blue / ochre / moss / slate / crimson.
PALETTE = {
    "navy": "#334155",        # Anchors, axes, primary structural text (deep slate)
    "blue": "#1f628e",        # Proposed / Switch / recommended (steel blue)
    "teal": "#2d6a4f",        # Secondary certified / degraded Switch (moss)
    "vermilion": "#b36a27",   # Baseline / Stay / constraint (ochre)
    "amber": "#d9a35f",       # Light ochre: degraded Stay / auxiliary
    "purple": "#9e363a",      # Crimson accent: serial / CRLB / failure bound
    "crimson": "#9e363a",
    "sky": "#8fb3cc",         # Light steel tint
    "slate": "#64748b",       # Secondary lines / annotations
    "gray_mid": "#a3adb8",    # Inactive edges / construction lines
    "gray_light": "#e2e8f0",  # Subtle fills
    "sea_bg": "#F5F9FC",
    "land_bg": "#ECE7DC",
    "land_edge": "#7D7668",
    "white": "#FFFFFF",
    "black": "#1e293b",
}

# Forbidden invented / internal terminology guard (v2.4.0 + reviewer-facing standard)
FORBIDDEN_TERMS = (
    "SRATC",
    "FIM-safe",
    "T1/",
    "T2/",
    "T3/",
    " T1 ",
    " T2 ",
    " T3 ",
    "Pinball",
    "pinball",
    "Make-Before-Break",
    "make-before-break",
    "Ladder",
    "ladder",
    "TDD-",
    "REFIND",
    "29.99",
    "bUAV",
    "sUAV",
    "DL3",
    "DL6",
    "DL7",
    "L12",
    "L14",
    "P6",
    "seed",
    "Seed",
    "1001",
    "1016",
    "FIM",
    "Fisher",
    "CRLB",
    "anchor",
    "Anchor",
)


def apply_ieee_style(base_font_size: float = 8.0) -> None:
    """Configure Matplotlib rcParams for IEEE / AST Times New Roman + STIX serif quality."""
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "Times", "STIXGeneral"],
            "mathtext.fontset": "stix",
            "font.size": base_font_size,
            "axes.titlesize": base_font_size + 0.5,
            "axes.titleweight": "bold",
            "axes.labelsize": base_font_size,
            "axes.labelweight": "normal",
            "xtick.labelsize": base_font_size - 0.8,
            "ytick.labelsize": base_font_size - 0.8,
            "legend.fontsize": base_font_size - 1.0,
            "legend.title_fontsize": base_font_size - 0.8,
            "legend.frameon": False,
            "legend.handlelength": 1.8,
            "legend.borderaxespad": 0.3,
            "text.color": PALETTE["black"],
            "axes.labelcolor": PALETTE["black"],
            "xtick.color": PALETTE["navy"],
            "ytick.color": PALETTE["navy"],
            "axes.edgecolor": PALETTE["navy"],
            "axes.linewidth": 0.6,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": False,
            "grid.color": "#D4DCE3",
            "grid.linestyle": "--",
            "grid.linewidth": 0.5,
            "grid.alpha": 0.7,
            "xtick.direction": "out",
            "ytick.direction": "out",
            "xtick.major.width": 0.65,
            "ytick.major.width": 0.65,
            "xtick.major.size": 3.0,
            "ytick.major.size": 3.0,
            "lines.linewidth": 1.5,
            "lines.markersize": 4.5,
            "patch.linewidth": 0.8,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "path.simplify": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "savefig.dpi": 300,
        }
    )


def check_terminology(texts: Iterable[str]) -> list[str]:
    """Return any offending occurrences of forbidden non-standard terms."""
    hits: list[str] = []
    for txt in texts:
        for bad in FORBIDDEN_TERMS:
            if bad in txt:
                hits.append(f"{bad!r} in {txt!r}")
    return hits


def check_text_clipping(fig: plt.Figure, pad_px: float = 1.0) -> list[str]:
    """Return list of visible text labels whose bounding boxes extend outside the figure."""
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    texts = list(fig.texts)
    for ax in fig.axes:
        texts.extend(ax.texts + [ax.title, ax.xaxis.label, ax.yaxis.label])
        if ax.axison:
            x0, x1 = sorted(ax.get_xlim())
            y0, y1 = sorted(ax.get_ylim())
            for tk, lbl in zip(ax.get_xticks(), ax.get_xticklabels()):
                if x0 - 1e-9 <= tk <= x1 + 1e-9:
                    texts.append(lbl)
            for tk, lbl in zip(ax.get_yticks(), ax.get_yticklabels()):
                if y0 - 1e-9 <= tk <= y1 + 1e-9:
                    texts.append(lbl)
        leg = ax.get_legend()
        if leg is not None:
            texts.extend(leg.get_texts())
            if leg.get_title() is not None:
                texts.append(leg.get_title())
    for leg in fig.legends:
        texts.extend(leg.get_texts())
        if leg.get_title() is not None:
            texts.append(leg.get_title())

    clipped: list[str] = []
    for t in texts:
        if t.get_visible() and t.get_text().strip():
            bbox = t.get_window_extent(renderer)
            if (
                bbox.x0 < -pad_px
                or bbox.y0 < -pad_px
                or bbox.x1 > fig.bbox.width + pad_px
                or bbox.y1 > fig.bbox.height + pad_px
            ):
                clipped.append(t.get_text())
    return clipped


def save_figure_pair(
    fig: plt.Figure,
    out_dir: str | Path,
    stem: str,
    dpi: int = 300,
    strict_clip_check: bool = True,
) -> tuple[Path, Path]:
    """Save both vector PDF (Type 42 fonts) and 300-DPI PNG, checking text bounds."""
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    # Terminology audit across all rendered strings
    all_strings: list[str] = [t.get_text() for t in fig.texts if t.get_text()]
    for ax in fig.axes:
        for t in ax.texts + [ax.title, ax.xaxis.label, ax.yaxis.label]:
            if t.get_text():
                all_strings.append(t.get_text())
        leg = ax.get_legend()
        if leg is not None:
            all_strings.extend(t.get_text() for t in leg.get_texts() if t.get_text())
            if leg.get_title() is not None and leg.get_title().get_text():
                all_strings.append(leg.get_title().get_text())
    bad_terms = check_terminology(all_strings)
    if bad_terms:
        raise ValueError(f"Forbidden terminology detected in {stem}: {bad_terms}")

    clipped = check_text_clipping(fig)
    if clipped and strict_clip_check:
        raise AssertionError(f"Clipped text labels in {stem}: {clipped}")

    pdf_path = out_path / f"{stem}.pdf"
    png_path = out_path / f"{stem}.png"

    fig.savefig(
        pdf_path,
        format="pdf",
        bbox_inches="tight",
        pad_inches=0.04,
        metadata={"CreationDate": None, "ModDate": None},
    )
    fig.savefig(
        png_path,
        format="png",
        dpi=dpi,
        bbox_inches="tight",
        pad_inches=0.04,
    )
    plt.close(fig)
    return pdf_path, png_path


def draw_coastal_band(
    ax: plt.Axes,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    shore_y: float = 0.0,
    zorder_base: int = 0,
) -> None:
    """Render clean shoreline land strip below shore_y and soft sea tint above."""
    ax.axhspan(shore_y, y_max, color=PALETTE["sea_bg"], zorder=zorder_base)
    ax.axhspan(y_min, shore_y, color=PALETTE["land_bg"], zorder=zorder_base)
    ax.axhline(shore_y, color=PALETTE["land_edge"], lw=1.1, zorder=zorder_base + 1)


def draw_beacons(
    ax: plt.Axes,
    beacons: dict[object, tuple[float, float]] | Sequence[tuple[float, float]],
    labels: Sequence[str] | None = None,
    size: float = 34.0,
    y_offset_pts: float = -10.5,
    fontsize: float = 7.0,
    zorder: int = 8,
) -> None:
    """Plot coastal fixed beacon UAVs (bUAVs) as crisp square markers with labels."""
    if isinstance(beacons, dict):
        items = list(beacons.items())
    else:
        lbls = labels or [f"$b_{i}$" for i in range(len(beacons))]
        items = list(zip(lbls, beacons))

    xs = [pt[0] for _, pt in items]
    ys = [pt[1] for _, pt in items]
    ax.scatter(
        xs,
        ys,
        marker="s",
        s=size,
        facecolor=PALETTE["navy"],
        edgecolor=PALETTE["white"],
        linewidths=0.7,
        zorder=zorder,
    )
    for lbl, (x, y) in items:
        txt = f"${lbl[0]}_{{{lbl[1:]}}}$" if isinstance(lbl, str) and lbl.startswith("b") and lbl[1:].isdigit() else str(lbl)
        ax.annotate(
            txt,
            (x, y),
            xytext=(0, y_offset_pts),
            textcoords="offset points",
            ha="center",
            va="top",
            fontsize=fontsize,
            color=PALETTE["navy"],
            fontweight="bold",
            zorder=zorder + 1,
        )


def draw_covariance_ellipse(
    ax: plt.Axes,
    center: tuple[float, float],
    cov_2x2: np.ndarray,
    kappa: float = 3.0,
    scale_display: float = 1.0,
    edgecolor: str = "#0072B2",
    facecolor: str = "#0072B2",
    fill_alpha: float = 0.16,
    lw: float = 1.3,
    ls: str = "-",
    zorder: int = 5,
    label: str | None = None,
) -> tuple[float, float, float]:
    """Draw a 2D covariance ellipse at kappa-sigma contour (optionally magnified for legibility).

    Returns (semi_major_m, semi_minor_m, angle_deg) in unscaled physical meters.
    """
    vals, vecs = np.linalg.eigh(0.5 * (cov_2x2 + cov_2x2.T))
    order = np.argsort(vals)[::-1]
    vals = np.maximum(vals[order], 1e-12)
    vecs = vecs[:, order]
    a_phys = float(kappa * math.sqrt(vals[0]))
    b_phys = float(kappa * math.sqrt(vals[1]))
    angle_deg = float(math.degrees(math.atan2(vecs[1, 0], vecs[0, 0])))

    ell_fill = Ellipse(
        xy=center,
        width=2.0 * a_phys * scale_display,
        height=2.0 * b_phys * scale_display,
        angle=angle_deg,
        facecolor=facecolor,
        edgecolor="none",
        alpha=fill_alpha,
        zorder=zorder,
    )
    ell_edge = Ellipse(
        xy=center,
        width=2.0 * a_phys * scale_display,
        height=2.0 * b_phys * scale_display,
        angle=angle_deg,
        facecolor="none",
        edgecolor=edgecolor,
        linewidth=lw,
        linestyle=ls,
        zorder=zorder + 0.1,
        label=label,
    )
    ax.add_patch(ell_fill)
    ax.add_patch(ell_edge)
    return a_phys, b_phys, angle_deg


def draw_arrow_segment(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    color: str,
    lw: float = 1.5,
    ls: str = "-",
    mutation_scale: float = 9.0,
    shrink_a: float = 2.0,
    shrink_b: float = 2.0,
    zorder: int = 6,
    alpha: float = 1.0,
) -> FancyArrowPatch:
    """Draw a high-contrast directional arrow between two 2D points."""
    arrow = FancyArrowPatch(
        posA=start,
        posB=end,
        arrowstyle="-|>",
        mutation_scale=mutation_scale,
        linewidth=lw,
        linestyle=ls,
        color=color,
        alpha=alpha,
        shrinkA=shrink_a,
        shrinkB=shrink_b,
        zorder=zorder,
    )
    ax.add_patch(arrow)
    return arrow


def draw_polygon_domain(
    ax: plt.Axes,
    vertices: Sequence[tuple[float, float]],
    edgecolor: str = PALETTE["blue"],
    facecolor: str = PALETTE["sky"],
    fill_alpha: float = 0.14,
    lw: float = 1.4,
    ls: str = "--",
    zorder: int = 2,
    label: str | None = None,
) -> Polygon:
    """Render a convex search domain polygon with light translucent fill and clean boundary."""
    poly = Polygon(
        list(vertices),
        closed=True,
        facecolor=facecolor,
        edgecolor="none",
        alpha=fill_alpha,
        zorder=zorder,
    )
    edge = Polygon(
        list(vertices),
        closed=True,
        facecolor="none",
        edgecolor=edgecolor,
        linewidth=lw,
        linestyle=ls,
        zorder=zorder + 0.2,
        label=label,
    )
    ax.add_patch(poly)
    ax.add_patch(edge)
    return edge


def add_panel_label(
    ax: plt.Axes,
    tag: str,
    title: str = "",
    x: float = 0.0,
    y: float = 1.03,
    fontsize: float = 8.5,
) -> None:
    """Place a panel label: bold tag such as '(a)' followed by a regular-weight title."""
    ax.text(
        x,
        y,
        tag,
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=fontsize,
        fontweight="bold",
        color=PALETTE["black"],
    )
    if title:
        ax.annotate(
            title,
            xy=(x, y),
            xycoords="axes fraction",
            xytext=(fontsize * 0.42 * len(tag) + 3.0, 0.0),
            textcoords="offset points",
            ha="left",
            va="bottom",
            fontsize=fontsize,
            color=PALETTE["black"],
        )
