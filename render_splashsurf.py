"""
MAGIC 2026 - splashsurf surface reconstruction + offline render

Turns the per-frame SPH particle dumps from magic_2026_crm_rassor_rig.py
(--dump) into the smooth, continuous regolith surface seen in SBEL's CRM
videos, instead of a cloud of spheres.

WHY THIS IS A SEPARATE OFFLINE PASS
Chrono can do this internally (ChFsiSplashsurfSPH / WriteReconstructedSurface,
and SphVisualizationSettings.use_splashsurf), but it is gated behind the
CHRONO_HAS_SPLASHSURF compile flag and this conda build of PyChrono was built
without it -- Chrono_fsisph.dll contains only the "splashsurf not available"
branch. So the reconstruction is done here with the pysplashsurf wheel, which
is the same splashsurf library, and rendered with PyVista.

The reconstruction parameters match the ones Chrono's own demos pass:
smoothing_length 2.0, cube_size 0.3, surface_threshold 0.6 (all in multiples
of the particle radius).

Usage:
    python render_splashsurf.py --dir=Rassor_results_wheel_bp1fine
    python render_splashsurf.py --dir=... --cube=0.5 --smooth=5 --fps=30
"""

import glob
import os
import sys

import numpy as np
import pysplashsurf
import pyvista as pv
import trimesh


def _cli(flag, default, cast=str):
    for a in sys.argv:
        if a.startswith(flag + "="):
            return cast(a.split("=", 1)[1])
    return default


HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, _cli("--dir", "Rassor_results_wheel_bp1fine"))
PART_DIR = os.path.join(OUT_DIR, "particles")
FRAME_DIR = os.path.join(OUT_DIR, "surface_frames")
os.makedirs(FRAME_DIR, exist_ok=True)

SPACING = _cli("--spacing", 0.005, float)
PARTICLE_RADIUS = SPACING * 0.5
# wider kernel + lower iso threshold than Chrono's defaults: at 2.0/0.6 the
# sparse churned ejecta behind the wheel surfaces as isolated blobs instead of
# a continuous disturbed surface
SMOOTHING_LENGTH = _cli("--smoothing", 4.0, float)
CUBE_SIZE = _cli("--cube", 0.6, float)
THRESHOLD = _cli("--threshold", 0.35, float)
SMOOTH_ITERS = _cli("--smooth", 5, int)
# airborne single-particle blobs are their own connected components, so keeping
# only the largest one removes them and leaves the bed + attached churn intact
# Size threshold, NOT extract_largest(): whether a churn clump counts as
# connected to the bed flips between frames, so "keep only the biggest piece"
# makes clumps pop in and out. Thresholding on component size is stable --
# a big clump stays above it either way, a lone particle never reaches it.
DROP_EJECTA = "--dropejecta" in sys.argv
MIN_BLOB_PTS = _cli("--minblob", 4000, int)
# A component is "floating" if the whole thing sits above the bed surface.
# Filtering on that keeps every clump resting in or on the soil (which is real
# churned material) and removes only airborne specks -- unlike a size
# threshold, which also deletes small clumps lying on the ground.
SURFACE_Z = _cli("--surfacez", 0.0, float)
FLOAT_TOL = _cli("--floattol", 0.004, float)
FPS = _cli("--fps", 30, int)
WHEEL_OBJ = os.path.join(HERE, "RigidWheelSimTest1_mesh.obj")

# lunar regolith albedo is low (~0.1-0.15): it is dark grey, not light tan
REGOLITH_GREY = _cli("--soilcolor", "#5d5954")

frames = sorted(glob.glob(os.path.join(PART_DIR, "sph_*.npy")))
if not frames:
    raise FileNotFoundError(f"no particle dumps in {PART_DIR} -- run the sim with --dump")
START = _cli("--start", 0, int)
LIMIT = _cli("--limit", 0, int)
frames = frames[START:]
if LIMIT:
    frames = frames[:LIMIT]
print(f"{len(frames)} particle frames in {PART_DIR}")

wheel_base = trimesh.load(WHEEL_OBJ, force="mesh", process=False)
print(f"wheel mesh: {len(wheel_base.faces)} faces")

pv.OFF_SCREEN = True


def quat_to_matrix(e0, e1, e2, e3):
    """Chrono quaternion (e0=w, e1..e3=xyz) -> 3x3 rotation."""
    return np.array([
        [1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2)],
        [2 * (e1 * e2 + e0 * e3), 1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1)],
        [2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1), 1 - 2 * (e1 * e1 + e2 * e2)],
    ])

setup_plot = False


p = pv.Plotter(off_screen=True, window_size=(1280, 720), lighting="none")
for i, fpath in enumerate(frames):
    
    pts = np.load(fpath).astype(np.float32)
    body = np.load(os.path.join(PART_DIR, f"body_{START + i:05d}.npy"))

    rec = pysplashsurf.reconstruct_surface(
        pts,
        particle_radius=PARTICLE_RADIUS,
        smoothing_length=SMOOTHING_LENGTH,
        cube_size=CUBE_SIZE,
        iso_surface_threshold=THRESHOLD,
    )
    mesh = rec.mesh
    v = np.asarray(mesh.vertices, dtype=np.float64)
    # force int64: splashsurf hands back unsigned indices, and hstacking those
    # with the VTK "3" prefix column promotes the whole array to float
    f = np.asarray(mesh.triangles, dtype=np.int64)
    faces_pv = np.hstack([np.full((len(f), 1), 3, dtype=np.int64), f]).ravel()
    soil = pv.PolyData(v, faces_pv)
    if SMOOTH_ITERS > 0:
        # Taubin rather than pysplashsurf's laplacian_smoothing_parallel (which
        # needs a prebuilt vertex-connectivity struct) -- and it does not shrink
        # the surface the way plain Laplacian does.
        soil = soil.smooth_taubin(n_iter=SMOOTH_ITERS, pass_band=0.05)
    if DROP_EJECTA:
        conn = soil.connectivity()
        rid = np.asarray(conn.point_data["RegionId"])
        zs = np.asarray(conn.points)[:, 2]
        nreg = rid.max() + 1
        # lowest point of each connected component
        minz = np.full(nreg, np.inf)
        np.minimum.at(minz, rid, zs)
        grounded = np.nonzero(minz <= SURFACE_Z + FLOAT_TOL)[0]
        soil = conn.extract_points(np.isin(rid, grounded),
                                   adjacent_cells=False).extract_surface(algorithm='dataset_surface')
    soil.compute_normals(inplace=True, auto_orient_normals=True)

    R = quat_to_matrix(body[3], body[4], body[5], body[6])
    wv = wheel_base.vertices @ R.T + body[0:3]
    wf = np.asarray(wheel_base.faces, dtype=np.int64)
    wheel = pv.PolyData(np.asarray(wv, dtype=np.float64),
                        np.hstack([np.full((len(wf), 1), 3, dtype=np.int64), wf]).ravel())

    cx, cy, cz = float(body[0]), float(body[1]), float(body[2])
    if not setup_plot:
        p.set_background("#0b0d10")
        p.add_light(pv.Light(position=(cx - 1.2, cy - 1.1, cz + 0.55),
                                focal_point=(cx, cy, cz - 0.15), intensity=1.05))
        p.add_light(pv.Light(position=(cx + 0.9, cy + 0.9, cz + 0.30),
                                focal_point=(cx, cy, cz - 0.15), intensity=0.22))
        # low sun raking across the surface, which is what makes a rut read
        p.camera.up = (0, 0, 1)
        setup_plot = True
    
    p.camera.position = (cx - 0.66, cy - 0.60, cz + 0.38)
    p.camera.focal_point = (cx + 0.06, cy, cz - 0.20)
    ground = p.add_mesh(soil, color=REGOLITH_GREY, smooth_shading=True,
               ambient=0.10, diffuse=0.95, specular=0.02, specular_power=4)
    wheel = p.add_mesh(wheel, color="#7f858c", smooth_shading=True, metallic=0.6,
               roughness=0.45, ambient=0.10, diffuse=0.65,
               specular=0.30, specular_power=25)
    
    p.screenshot(os.path.join(FRAME_DIR, f"surf_{START + i:05d}.png"))
    p.remove_actor(ground)
    p.remove_actor(wheel)

    print(f"frame {i}/{len(frames)}: {len(pts)} particles -> "
            f"{len(f)} triangles")

print("assembling video...")
try:
    import imageio.v2 as imageio
    out = os.path.join(OUT_DIR, "regolith_surface.mp4")
    with imageio.get_writer(out, fps=FPS, codec="libx264", quality=8) as w:
        for i in range(len(frames)):
            fp = os.path.join(FRAME_DIR, f"surf_{START + i:05d}.png")
            if os.path.exists(fp):
                w.append_data(imageio.imread(fp))
    print(f"wrote {out}")
except Exception as e:
    print(f"(video assembly skipped: {e})")
