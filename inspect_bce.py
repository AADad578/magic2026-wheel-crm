"""
Render the wheel's BCE markers -- the point cloud that is the ONLY thing the
SPH soil actually sees. The visual mesh is irrelevant to the physics; if this
cloud has gaps or is too thin, soil tunnels straight through the rim.

Chrono's rule of thumb is >= 3 marker layers across any wall for the SPH kernel
to have proper support. CreatePointsMesh fills the mesh interior on a grid at
initial_spacing, so a thin-walled wheel can end up with 1-2 layers and leak.

    python inspect_bce.py --spacing=0.0035
"""

import os
import sys

import numpy as np
import pychrono as chrono
import pychrono.fsi as fsi
import pyvista as pv


def _cli(flag, default, cast=str):
    for a in sys.argv:
        if a.startswith(flag + "="):
            return cast(a.split("=", 1)[1])
    return default


HERE = os.path.dirname(os.path.abspath(__file__))
SPACING = _cli("--spacing", 0.0035, float)
MESH_BCE = os.path.join(HERE, "RigidWheelSimTest1_mesh_bce.obj")

sysSPH = fsi.ChFsiFluidSystemSPH()
sph_params = fsi.SPHParameters()
sph_params.initial_spacing = SPACING
sph_params.d0_multiplier = 1.0
sysSPH.SetSPHParameters(sph_params)

mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(MESH_BCE, False, False)
bce = sysSPH.CreatePointsMesh(mesh)
pts = np.array([[p.x, p.y, p.z] for p in bce])
print(f"spacing {SPACING*1000:.1f} mm -> {len(pts)} BCE markers")

# axle is local X, so radius is in the Y-Z plane
r = np.sqrt(pts[:, 1] ** 2 + pts[:, 2] ** 2)
print(f"radial extent: {r.min()*1000:.1f} .. {r.max()*1000:.1f} mm")
print("\nradial marker histogram (layer count = bins with markers):")
edges = np.arange(0, r.max() + SPACING, SPACING)
hist, _ = np.histogram(r, bins=edges)
for h, e in zip(hist, edges[:-1]):
    if h:
        print(f"  r {e*1000:6.1f} mm : {'#' * min(60, h // 20):60s} {h}")

rim = r[(r > 0.11) & (r < 0.145)]
if len(rim):
    layers = len(np.unique(np.round(rim / SPACING)))
    print(f"\nrim band 110-145 mm: {len(rim)} markers across {layers} distinct layers")
    print("  >= 3 layers needed for proper SPH support; fewer than that leaks")

cloud = pv.PolyData(pts)
p = pv.Plotter(off_screen=True, window_size=(1400, 900), shape=(1, 2))
p.subplot(0, 0)
p.add_text("BCE markers (what the soil sees)", font_size=10)
p.add_mesh(cloud, color="#ff9040", point_size=6, render_points_as_spheres=True)
p.camera_position = "yz"
p.subplot(0, 1)
p.add_text("cutaway: markers with |x| < 5 mm", font_size=10)
slab = pts[np.abs(pts[:, 0]) < 0.005]
p.add_mesh(pv.PolyData(slab), color="#ff9040", point_size=9, render_points_as_spheres=True)
p.camera_position = "yz"
out = os.path.join(HERE, f"bce_markers_{int(SPACING*10000)}.png")
p.screenshot(out)
print(f"\nwrote {out}")
