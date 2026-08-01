"""golden_trace.py -- stakeholder-approved golden traces for the square-tour
system test (see clasi/issues/square-tour-is-the-one-system-test-sim-bench-
playfield.md).

A run produces one trace per signal (each wheel speed, each commanded
velocity, each measured velocity, the X-Y path, robot location over time).
Eric looks at the rendered images and, when they show what he wants, promotes
them to GOLDEN. Every later run is then scored against the golden.

Why the gate is analytic and not a pixel operation
--------------------------------------------------
The original sketch was: render the golden with a deliberately widened stroke,
then XOR/AND the new run's raster against it and count pixels. That works, but
it makes the verdict depend on things that have nothing to do with the robot --
figure size, DPI, antialiasing, and the exact rasterization of whatever
matplotlib version happens to be installed. A renderer point-release that
shifts a line by one pixel silently re-scores every golden in the repo.

So the GATE here is analytic: for every sample in the new run, the shortest
distance to the golden polyline, measured in DATA units (mm, mm/s). A sample
is out of band when that distance exceeds the tolerance. This is exact,
resolution-independent, and unchanged by any rendering library.

The raster comparison is still produced, because a picture is what a human
actually verifies and because the pixel counts were asked for -- but it is
reported ALONGSIDE the analytic verdict, not as the verdict.

Why not a plain XOR
-------------------
XOR conflates two different things:

  - new-run ink OUTSIDE the golden band  -- a real failure
  - golden band NOT covered by the new run -- often legitimate (a shorter or
    faster run simply visits less of the band)

Summing them into one number means a run can fail for having been short. This
module reports the two terms separately: ``outside_pixels`` is the failure
metric, ``uncovered_pixels`` is a coverage signal.

Units: tolerances and axis limits are in the signal's own data units. Per
``.claude/rules/coding-standards.md`` no identifier carries a unit; units live
in ``# [unit]`` trailing comments at the call site that knows them.
"""
from __future__ import annotations

import json
import pathlib
from dataclasses import dataclass, asdict

import numpy as np


# Pixel centres are sampled at (i + 0.5) so a mask is symmetric about the
# axis limits rather than biased toward the origin corner.
_PIXEL_CENTRE = 0.5


@dataclass(frozen=True)
class PlotSpec:
    """Canonical, PINNED render geometry for one signal.

    Axis limits are mandatory and never autoscaled: two runs with different
    data ranges would otherwise produce visually similar but incomparable
    images, and the comparison would pass or fail for reasons unrelated to the
    robot. Storing the limits with the golden is what makes a later run
    comparable at all.
    """

    x_low: float
    x_high: float
    y_low: float
    y_high: float
    width: int = 640   # [px]
    height: int = 480  # [px]

    def __post_init__(self) -> None:
        if self.x_high <= self.x_low or self.y_high <= self.y_low:
            raise ValueError(f"degenerate axis limits: {self}")
        if self.width < 2 or self.height < 2:
            raise ValueError(f"degenerate raster size: {self}")

    def pixel_centres(self) -> "tuple[np.ndarray, np.ndarray]":
        """(xs, ys) of every pixel centre, in data units. ys descend so row 0
        is the top of the image, matching how the PNG is written."""
        xs = self.x_low + (np.arange(self.width) + _PIXEL_CENTRE) * (
            self.x_high - self.x_low) / self.width
        ys = self.y_high - (np.arange(self.height) + _PIXEL_CENTRE) * (
            self.y_high - self.y_low) / self.height
        return xs, ys


@dataclass(frozen=True)
class Comparison:
    """Outcome of scoring one run against one golden."""

    sample_count: int
    outside_count: int          # samples farther than tolerance from the golden
    # UNITLESS multiples of tolerance: 1.0 is exactly on the acceptance
    # ellipse, 2.0 is twice as far out as allowed. With a per-axis tolerance
    # there is no single unit to report a distance in, and normalizing makes
    # severity comparable across signals.
    max_deviation: float        # [x tolerance] worst sample
    rms_deviation: float        # [x tolerance]
    outside_pixels: int         # raster: new-run ink outside the band (FAILURE)
    uncovered_pixels: int       # raster: band the run never visited (coverage)
    band_pixels: int
    tolerance_x: float          # [x-axis data units]
    tolerance_y: float          # [y-axis data units]

    @property
    def outside_fraction(self) -> float:
        return self.outside_count / self.sample_count if self.sample_count else 0.0

    def passed(self, max_outside_fraction: float = 0.0) -> bool:
        """Default is strict: no sample may leave the band. Loosen only with a
        stated reason -- a tolerance that absorbs a real regression is worse
        than no test."""
        return self.outside_fraction <= max_outside_fraction

    def summary(self) -> str:
        verdict = "PASS" if self.passed() else "FAIL"
        return (
            f"[{verdict}] {self.outside_count}/{self.sample_count} samples outside "
            f"band (tol x={self.tolerance_x:g} y={self.tolerance_y:g}); "
            f"max {self.max_deviation:.3g}x rms {self.rms_deviation:.3g}x tol; "
            f"raster outside={self.outside_pixels} "
            f"uncovered={self.uncovered_pixels}/{self.band_pixels}"
        )


@dataclass(frozen=True)
class AxisTolerance:
    """Per-axis tolerance, each in ITS OWN axis's data units.

    A single scalar tolerance is only meaningful when both axes carry the same
    unit -- true for ``xy_trace`` (mm vs mm), false for every time series, where
    x is seconds and y is mm/s, mm, rad or ms. A scalar there measures Euclidean
    distance through a blended space, silently equating one second with one mm/s.

    Measured consequence (circle tour, 2026-08-01): at the shipped scalar
    tolerances the band covered 96.8% of the wheel-speed plot, 81% of x_t/y_t
    and 100% of both cycle-timing plots -- and since ``compare()`` scored with
    the same metric, a run displaced by 10 SECONDS still sat inside a "15"
    tolerance. The band was not merely ugly; the gate could barely fail.

    Geometrically the acceptance region is the trace dilated by an ELLIPSE with
    semi-axes (x, y) -- an oval kernel swept along the polyline. Dividing each
    axis by its own tolerance maps that ellipse onto the unit circle, so the
    test stays exact (not rasterized) and costs nothing over the scalar form.
    """

    x: float
    y: float

    def __post_init__(self) -> None:
        if self.x <= 0.0 or self.y <= 0.0:
            raise ValueError(f"tolerance must be positive on both axes: {self}")


def as_axis_tolerance(tolerance: "float | AxisTolerance") -> AxisTolerance:
    """Accept a scalar (same tolerance on both axes) or an explicit pair.

    Scalars stay valid for same-unit plots -- not deprecated, just insufficient
    for mixed-unit ones."""
    if isinstance(tolerance, AxisTolerance):
        return tolerance
    return AxisTolerance(float(tolerance), float(tolerance))


def _segment_distance(
    px: np.ndarray, py: np.ndarray, ax: float, ay: float, bx: float, by: float
) -> np.ndarray:
    """Shortest distance from each point (px, py) to segment a->b.

    Degenerate (zero-length) segments fall back to point distance, which is
    what a stopped robot produces -- and this test deliberately includes
    planned stops, so that case is normal, not exceptional.
    """
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return np.hypot(px - ax, py - ay)
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / length_squared, 0.0, 1.0)
    return np.hypot(px - (ax + t * dx), py - (ay + t * dy))


def distance_to_polyline(
    px: np.ndarray, py: np.ndarray, gx: np.ndarray, gy: np.ndarray
) -> np.ndarray:
    """Shortest distance from each query point to the polyline (gx, gy).

    A single golden point (no segments) degenerates to point distance.
    """
    px = np.asarray(px, dtype=float)
    py = np.asarray(py, dtype=float)
    gx = np.asarray(gx, dtype=float)
    gy = np.asarray(gy, dtype=float)
    if gx.size == 0:
        raise ValueError("golden polyline is empty")
    if gx.size == 1:
        return np.hypot(px - gx[0], py - gy[0])

    best = np.full(px.shape, np.inf)
    for i in range(gx.size - 1):
        np.minimum(
            best,
            _segment_distance(px, py, gx[i], gy[i], gx[i + 1], gy[i + 1]),
            out=best,
        )
    return best


def band_mask(
    gx: np.ndarray, gy: np.ndarray, spec: PlotSpec,
    tolerance: "float | AxisTolerance",
) -> np.ndarray:
    """Boolean raster of the tolerance band around the golden polyline.

    True where a pixel centre lies within ``tolerance`` (data units) of the
    polyline. Computed per segment over that segment's own bounding box, so
    cost scales with the trace length rather than with width*height*segments.
    """
    tol = as_axis_tolerance(tolerance)
    xs, ys = spec.pixel_centres()
    mask = np.zeros((spec.height, spec.width), dtype=bool)
    gx = np.asarray(gx, dtype=float)
    gy = np.asarray(gy, dtype=float)
    if gx.size == 0:
        return mask

    x_step = (spec.x_high - spec.x_low) / spec.width
    y_step = (spec.y_high - spec.y_low) / spec.height

    for i in range(max(gx.size - 1, 1)):
        ax, ay = gx[i], gy[i]
        bx, by = (gx[i + 1], gy[i + 1]) if gx.size > 1 else (gx[i], gy[i])

        # Bounding box from each axis's OWN tolerance -- the ellipse's extent,
        # not one radius reused on both axes.
        col_low = int(np.floor((min(ax, bx) - tol.x - spec.x_low) / x_step)) - 1
        col_high = int(np.ceil((max(ax, bx) + tol.x - spec.x_low) / x_step)) + 1
        row_low = int(np.floor((spec.y_high - (max(ay, by) + tol.y)) / y_step)) - 1
        row_high = int(np.ceil((spec.y_high - (min(ay, by) - tol.y)) / y_step)) + 1

        col_low, col_high = max(col_low, 0), min(col_high, spec.width)
        row_low, row_high = max(row_low, 0), min(row_high, spec.height)
        if col_low >= col_high or row_low >= row_high:
            continue

        grid_x, grid_y = np.meshgrid(xs[col_low:col_high], ys[row_low:row_high])
        # Scaling each axis by its own tolerance maps the acceptance ELLIPSE
        # onto the unit circle, so one distance test covers both axes exactly.
        near = _segment_distance(
            grid_x / tol.x, grid_y / tol.y,
            ax / tol.x, ay / tol.y, bx / tol.x, by / tol.y,
        ) <= 1.0
        mask[row_low:row_high, col_low:col_high] |= near

    return mask


def normalized_distance_to_polyline(
    px: np.ndarray, py: np.ndarray, gx: np.ndarray, gy: np.ndarray,
    tolerance: "float | AxisTolerance",
) -> np.ndarray:
    """Distance to the golden polyline in UNITS OF TOLERANCE, per axis.

    Unitless: <= 1.0 is inside the acceptance ellipse, 2.0 is twice as far out
    as allowed. This is what the verdict uses -- a raw geometric distance cannot
    be tested against one threshold when the axes carry different units.

    It also makes signals commensurable: "1.4x tolerance" means the same
    severity on a wheel-speed plot and a heading plot, where "0.09" would not.
    """
    tol = as_axis_tolerance(tolerance)
    return distance_to_polyline(
        np.asarray(px, dtype=float) / tol.x, np.asarray(py, dtype=float) / tol.y,
        np.asarray(gx, dtype=float) / tol.x, np.asarray(gy, dtype=float) / tol.y,
    )


def compare(
    new_x: np.ndarray,
    new_y: np.ndarray,
    golden_x: np.ndarray,
    golden_y: np.ndarray,
    spec: PlotSpec,
    tolerance: "float | AxisTolerance",
) -> Comparison:
    """Score a run against a golden.

    The verdict comes from the analytic per-sample distance; the raster terms
    are reported for the human-facing artifact and are deliberately split into
    a failure metric and a coverage metric (see the module docstring).
    """
    new_x = np.asarray(new_x, dtype=float)
    new_y = np.asarray(new_y, dtype=float)
    if new_x.size != new_y.size:
        raise ValueError("new trace x/y length mismatch")
    if new_x.size == 0:
        raise ValueError("new trace is empty -- a run that produced no samples "
                         "must never score as a clean pass")

    tol = as_axis_tolerance(tolerance)
    deviation = normalized_distance_to_polyline(
        new_x, new_y, golden_x, golden_y, tol)

    band = band_mask(golden_x, golden_y, spec, tolerance)
    # A hairline raster of the new run: tolerance of one pixel diagonal, so the
    # ink is thin enough that "outside the band" means genuinely outside.
    x_step = (spec.x_high - spec.x_low) / spec.width
    y_step = (spec.y_high - spec.y_low) / spec.height
    hairline = AxisTolerance(x_step * 0.75, y_step * 0.75)
    ink = band_mask(new_x, new_y, spec, hairline)

    return Comparison(
        sample_count=int(new_x.size),
        outside_count=int(np.count_nonzero(deviation > 1.0)),
        max_deviation=float(deviation.max()),
        rms_deviation=float(np.sqrt(np.mean(deviation ** 2))),
        outside_pixels=int(np.count_nonzero(ink & ~band)),
        uncovered_pixels=int(np.count_nonzero(band & ~ink)),
        band_pixels=int(np.count_nonzero(band)),
        tolerance_x=float(tol.x),
        tolerance_y=float(tol.y),
    )


def suggest_tolerance(
    runs: "list[tuple[np.ndarray, np.ndarray]]",
    golden_x: np.ndarray,
    golden_y: np.ndarray,
    margin: float = 1.5,
) -> float:
    """Derive a starting tolerance from observed run-to-run spread.

    Take several runs with NO code change and let the measured variation set
    the band, rather than picking a number that happens to make the first run
    pass. Returns the worst deviation seen across the runs, times ``margin``.
    """
    if not runs:
        raise ValueError("need at least one run to derive a tolerance")
    worst = 0.0
    for rx, ry in runs:
        worst = max(worst, float(distance_to_polyline(rx, ry, golden_x, golden_y).max()))
    return worst * margin


# --- persistence ---------------------------------------------------------
# The image is what a human verifies; the dataset is what makes a failure
# diagnosable and lets a later re-score at a different tolerance happen
# without re-running the robot.

def suggest_axis_tolerance(
    runs: "list[tuple[np.ndarray, np.ndarray]]",
    golden_x: np.ndarray, golden_y: np.ndarray,
    base: AxisTolerance, margin: float = 1.5,
) -> AxisTolerance:
    """Scale a per-axis tolerance to the observed run-to-run spread.

    ``base`` supplies the SHAPE of the acceptance ellipse -- the ratio between
    the two axes, which encodes their unrelated units and cannot be discovered
    from the data. Repeat runs with no code change then supply the SIZE: the
    worst normalized deviation seen, times ``margin``.

    Splitting it this way keeps both halves honest. Deriving the ratio from
    measured spread would let one noisy axis silently reshape the ellipse;
    deriving the size by hand would let the band be whatever makes today's run
    pass.
    """
    if not runs:
        raise ValueError("need at least one run to derive a tolerance")
    worst = 0.0
    for rx, ry in runs:
        deviation = normalized_distance_to_polyline(rx, ry, golden_x, golden_y, base)
        worst = max(worst, float(deviation.max()))
    scale = max(worst, 1e-9) * margin
    return AxisTolerance(base.x * scale, base.y * scale)


def save_golden(
    directory: pathlib.Path, name: str, x: np.ndarray, y: np.ndarray,
    spec: PlotSpec, tolerance: "float | AxisTolerance",
) -> pathlib.Path:
    """Write the raw series plus its spec. Promotion to golden is a human act;
    this only records the candidate."""
    directory.mkdir(parents=True, exist_ok=True)
    data = directory / f"{name}.csv"
    np.savetxt(data, np.column_stack([x, y]), delimiter=",", header="x,y", comments="")
    (directory / f"{name}.json").write_text(
        json.dumps({"spec": asdict(spec),
                    "tolerance": asdict(as_axis_tolerance(tolerance))},
                   indent=2) + "\n"
    )
    return data


def load_golden(
    directory: pathlib.Path, name: str
) -> "tuple[np.ndarray, np.ndarray, PlotSpec, AxisTolerance]":
    rows = np.loadtxt(directory / f"{name}.csv", delimiter=",", skiprows=1, ndmin=2)
    meta = json.loads((directory / f"{name}.json").read_text())
    raw = meta["tolerance"]
    # Goldens blessed before per-axis tolerances stored a bare number.
    tol = AxisTolerance(**raw) if isinstance(raw, dict) else as_axis_tolerance(raw)
    return rows[:, 0], rows[:, 1], PlotSpec(**meta["spec"]), tol


def render_png(
    path: pathlib.Path, x: np.ndarray, y: np.ndarray, spec: PlotSpec,
    tolerance: "float | AxisTolerance", label: str = "",
) -> None:
    """The acceptance image: the band, the trace, nothing else.

    NO axes, ticks, title, margins or legend (stakeholder, 2026-08-01). Every
    adornment is another thing that can move between runs for reasons unrelated
    to the robot -- a matplotlib release that renders a tick label one pixel
    wider shifts the plot area, which does not change the verdict but does stop
    two images being comparable by eye. The picture is the trace and its band;
    the label belongs in the FILENAME.

    Axes fill the figure exactly, so the raster is in direct correspondence with
    the data: pixel (0,0) is (spec.x_low, spec.y_high), and because the spec is
    pinned rather than autoscaled, the same data always lands on the same pixels.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    band = band_mask(x, y, spec, tolerance)
    fig = plt.figure(figsize=(spec.width / 100, spec.height / 100), dpi=100)
    ax = fig.add_axes((0.0, 0.0, 1.0, 1.0))
    ax.imshow(
        band, cmap="Greys", alpha=0.30, origin="upper",
        extent=(spec.x_low, spec.x_high, spec.y_low, spec.y_high),
        aspect="auto", interpolation="nearest",
    )
    ax.plot(x, y, linewidth=1.0, color="#1f77b4")
    ax.set_xlim(spec.x_low, spec.x_high)   # never autoscale
    ax.set_ylim(spec.y_low, spec.y_high)
    ax.set_axis_off()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=100)
    plt.close(fig)
