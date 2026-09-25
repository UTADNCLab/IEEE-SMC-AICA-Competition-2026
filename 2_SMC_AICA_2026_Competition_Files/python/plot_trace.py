"""curb penalty report -- renders a completed run over the drivable area.

PURPOSE
-------
Produces the visual record behind a run's curb penalty, showing where the
QCar2 left the road surface and which of those departures were scored.
Intended both as a review tool for teams and as supporting evidence if a
penalty is queried.

OUTPUT
------
A table of every scored hit, plus two images:

    trace_overview.png   the whole run
    trace_zoom.png       a detail view, centred on ZOOM_AT below

On each image, every 10 Hz sample of the run is plotted:

    green dot    all four wheels on the road surface
    pink dot     at least one wheel off the road, that sample
    red cross    a scored hit

Contact is recorded per sample; hits are recorded per excursion, so a
sustained departure appears as a run of pink samples carrying fewer
crosses. The number of crosses matches the hit count on the score sheet.

USAGE
-----
    python plot_trace.py
    python plot_trace.py <x> <y>     also reports on a specific location,
                                     and centres the detail view there

drivable_grid.npz and curb_trace.csv must be present alongside this script.
curb_trace.csv is written by game_finals.py when curb_TRACE is enabled.

The detection constants below must match those in game_finals.py; if they
differ, this report will not agree with the score awarded.
"""

from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent

# Centre and width of the detail view, in world metres. Defaults to the
# pickup; override from the command line to inspect any other location.
ZOOM_AT = (-2.50305, 29.6703)
ZOOM_SPAN = 20.0

# Detection constants. These MUST match game_finals.py -- they determine
# which samples are marked as scored hits, and a mismatch would make this
# report disagree with the score awarded.
curb_MIN_WHEELS = 1
curb_MIN_DEPTH = 0.10
curb_MIN_TICKS = 1
HIT_CLEAR_TIME = 0.60
curb_REPEAT_TIME = 1.0

# Pickup and drop-off points, as defined in game_finals.py. Drawn on both
# images for orientation.
PAD_POINTS = [
    ("Pickup",    -2.50305, 29.6703),
    ("Common 1",  11.2739, -10.84655),
    ("Common 2",  22.435,    1.392),
    ("Common 3",  22.5478,  29.6703),
    ("Common 4",   0.0,      44.9735),
    ("Common 5", -19.84125, 29.6703),
    ("Large 1",  -12.8205,  -4.5991),
    ("Large 2",    8.975,   37.099),
    ("Large 3",    8.367,   10.853),
]


def main():
    import sys

    # Optional coordinate argument: centres the detail view on that point
    # and reports the grid's state there. Use to examine a specific
    # location, for example one identified with the Coordinate Helper.
    global ZOOM_AT
    query = None
    if len(sys.argv) >= 3:
        try:
            query = (float(sys.argv[1]), float(sys.argv[2]))
            ZOOM_AT = query
            print(f"  querying ({query[0]:.2f}, {query[1]:.2f})")
        except ValueError:
            print("  usage: python plot_trace.py [x y]")

    grid_path = ROOT / "drivable_grid.npz"
    trace_path = ROOT / "curb_trace.csv"

    if not grid_path.exists():
        print(f"No {grid_path.name} here. Run build_drivable_grid.py first.")
        raise SystemExit(1)
    if not trace_path.exists():
        print(f"No {trace_path.name} here. Run the game with curb_TRACE = True.")
        raise SystemExit(1)

    data = np.load(grid_path)
    mask = data["mask"]
    x0, y0, cell = float(data["x0"]), float(data["y0"]), float(data["cell"])
    h, w = mask.shape

    tr = np.genfromtxt(trace_path, delimiter=",", names=True)
    x = np.atleast_1d(tr["x"])
    y = np.atleast_1d(tr["y"])
    off = np.atleast_1d(tr["wheels_off"]).astype(int)
    t = np.atleast_1d(tr["t"])
    if "depth" in (tr.dtype.names or ()):
        depth = np.atleast_1d(tr["depth"])
    else:
        depth = np.where(off > 0, 999.0, 0.0)   # older trace, no depth column

    scored = find_scored_hits(t, off, depth)

    report_hits(t, x, y, off, depth, scored)

    # Reported only when a location is supplied on the command line.
    if query is not None:
        try:
            import cv2
            dist = (cv2.distanceTransform(mask.astype(np.uint8),
                                          cv2.DIST_L2, 5) * cell)
            report_query(query, mask, dist, x0, y0, cell, w, h, x, y, off, t)
        except ImportError:
            pass

    draw(mask, x0, y0, w, h, cell, x, y, off, scored, None,
         "trace_overview.png", "Whole run")
    draw(mask, x0, y0, w, h, cell, x, y, off, scored, (ZOOM_AT, ZOOM_SPAN),
         "trace_zoom.png", "Pickup area")


def find_scored_hits(t, off, depth):
    """Identify which samples were recorded as hits.

    Reproduces the scoring logic in game_finals.py: a hit is recorded on
    first qualifying contact of an excursion, and again for each further
    curb_REPEAT_TIME the vehicle remains off the road. The vehicle must
    return to the road surface for HIT_CLEAR_TIME before a new excursion
    can begin.

    This distinction matters for the report: contact is present in every
    sample of a departure, but only these samples correspond to points
    deducted.
    """
    scored = np.zeros(len(t), dtype=bool)
    armed, quiet_since, run, last_hit_at = True, None, 0, None
    for k in range(len(t)):
        qualifies = off[k] >= curb_MIN_WHEELS and depth[k] >= curb_MIN_DEPTH
        run = run + 1 if qualifies else 0
        if run >= curb_MIN_TICKS:
            quiet_since = None
            if armed or (last_hit_at is not None
                         and t[k] - last_hit_at >= curb_REPEAT_TIME):
                scored[k] = True
                armed = False
                last_hit_at = t[k]
        else:
            if quiet_since is None:
                quiet_since = t[k]
            elif t[k] - quiet_since >= HIT_CLEAR_TIME:
                armed = True
    return scored


def report_hits(t, x, y, off, depth, scored):
    """Tabulate every scored hit with its supporting measurements.

    Columns:

      t, x, y  time and position at which the hit was recorded.
      depth    distance the furthest wheel travelled past the road edge.
               At the current grid resolution 0.10 m is the smallest
               representable value, so a reading of 0.10 indicates a wheel
               immediately across the boundary.
      ticks    duration of the excursion in 0.1 s samples.
    """
    idx = np.flatnonzero(scored)
    if not len(idx):
        return

    # Label each contiguous run of off-road samples, so each hit can report
    # the duration of its excursion.
    runs = np.zeros(len(t), dtype=int)
    rid, prev = 0, False
    for k in range(len(t)):
        now = off[k] > 0
        if now and not prev:
            rid += 1
        runs[k] = rid if now else 0
        prev = now

    print("\n  scored hits:")
    print(f"  {'#':>3} {'t':>7} {'x':>8} {'y':>8} {'depth':>7} {'ticks':>6}")
    for n, k in enumerate(idx, 1):
        r = runs[k]
        length = int((runs == r).sum()) if r else 1
        print(f"  {n:>3} {t[k]:7.1f} {x[k]:8.2f} {y[k]:8.2f} "
              f"{depth[k]:7.2f} {length:6d}")
    print()


def report_query(query, mask, dist, x0, y0, cell, w, h, x, y, off, t):
    """Report the grid's classification of a specific location.

    The margin is the distance from that point to the nearest non-drivable
    cell. A small margin places the point close to the road edge; a large
    one places it well inside open road, in which case no adjustment to the
    detection thresholds would register contact there.

    Also lists any samples of the run passing within 3 m, with the number
    of wheels off the road at each.
    """
    qx, qy = query
    i = int(np.clip(round((qx - x0) / cell), 0, w - 1))
    j = int(np.clip(round((qy - y0) / cell), 0, h - 1))

    print()
    print("================= QUERY =================")
    print(f"  point            : ({qx:.2f}, {qy:.2f})")
    print(f"  classification   : "
          f"{'drivable road surface' if mask[j, i] else 'off road'}")
    if mask[j, i]:
        print(f"  margin to edge   : {dist[j, i]:.2f} m")
        if dist[j, i] > 2.0:
            print("     Well inside the road surface; no obstacle is")
            print("     represented at this location in the grid.")
        elif dist[j, i] > 0.5:
            print("     Inside the road surface, near the edge. Reducing")
            print("     EDGE_GRACE in build_drivable_grid.py would move")
            print("     the boundary inward.")
        else:
            print("     On the boundary; within the resolution of the")
            print("     grid and the vehicle footprint.")

    near = np.hypot(x - qx, y - qy) <= 3.0
    if near.any():
        print(f"\n  samples within 3 m ({int(near.sum())}):")
        print(f"  {'t':>8}  {'x':>8}  {'y':>8}  {'wheels off':>10}")
        for tt, xx, yy, oo in zip(t[near], x[near], y[near], off[near]):
            print(f"  {tt:8.2f}  {xx:8.2f}  {yy:8.2f}  {oo:10d}")
    else:
        print("\n  no samples within 3 m of this location.")
    print("=========================================")


def draw(mask, x0, y0, w, h, cell, x, y, off, scored, zoom, name, label):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available -- cannot draw.")
        return

    x1 = x0 + (w - 1) * cell
    y1 = y0 + (h - 1) * cell

    fig, ax = plt.subplots(figsize=(11, 11))
    ax.imshow(mask, origin="lower", extent=[x0, x1, y0, y1],
              cmap="Greys", alpha=0.75, interpolation="nearest")

    clean = off == 0
    ax.plot(x[clean], y[clean], ".", color="#00a03c", markersize=2.0,
            zorder=3, label="all wheels on road")
    if (~clean).any():
        ax.plot(x[~clean], y[~clean], ".", color="#f0a0a0", markersize=5.0,
                zorder=4, label="in contact (per tick)")
    if scored.any():
        # Samples corresponding to a recorded hit; the count matches the
        # curb hits reported at the end of the run.
        ax.plot(x[scored], y[scored], "X", color="#d00000", markersize=11,
                markeredgecolor="black", markeredgewidth=0.9, zorder=8,
                linestyle="none",
                label=f"SCORED HIT ({int(scored.sum())})")

    ax.plot(x[0], y[0], "o", color="#0060d0", markersize=9,
            markeredgecolor="black", zorder=5, label="start")

    for pname, pxx, pyy in PAD_POINTS:
        ax.plot(pxx, pyy, "o", color="#ff6a00", markersize=8,
                markeredgecolor="black", markeredgewidth=0.8, zorder=6)
        ax.annotate(pname, (pxx, pyy), textcoords="offset points",
                    xytext=(8, 6), fontsize=8, color="#b03c00",
                    fontweight="bold", zorder=7)

    if zoom:
        (cx, cy), span = zoom
        ax.set_xlim(cx - span / 2, cx + span / 2)
        ax.set_ylim(cy - span / 2, cy + span / 2)

    ax.set_title(f"{label} -- QCar2 path over the drivable area\n"
                 "pink = wheel off the road (per sample);  "
                 "X = recorded hit", fontsize=12)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(alpha=0.2, linewidth=0.5)

    out = ROOT / name
    try:
        if out.exists():
            out.unlink()
        fig.savefig(out, dpi=130, bbox_inches="tight")
        print(f"Saved {out.name}")
    except OSError:
        print(f"  could not write {name}; the file may be open elsewhere.")


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        try:
            input("\nPress Enter to close...")
        except EOFError:
            pass