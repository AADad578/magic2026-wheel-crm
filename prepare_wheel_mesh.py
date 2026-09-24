"""
MAGIC 2026 - STEP -> watertight OBJ mesh preprocessor for RigidWheelSimTest1

WHAT THIS IS
One-time preprocessing step that turns the CAD STEP file for the grousered
test wheel into a watertight triangle mesh that PyChrono/FSI can consume
(via ChFsiFluidSystemSPH.CreatePointsMesh, which needs a closed mesh to do its
inside/outside ray-parity test when generating BCE boundary markers).

WHY IT'S SEPARATE FROM THE SIM SCRIPT
This uses cascadio (OpenCASCADE's own BRepMesh tessellator) + trimesh +
pymeshfix, none of which live in the `chrono` conda env's simulation stack
(no pychrono.cascade wheel is published, and gmsh's own remesher chokes on
this model's periodic/revolved surfaces with "Impossible to mesh periodic
surface"). Those three packages were pip-installed into the `chrono` env
so this can run there too; if you rebuild the env, `pip install cascadio
trimesh pymeshfix networkx scipy` again.

WHAT IT DOES
1. Tessellate the STEP B-rep directly with OpenCASCADE (cascadio) -> glTF.
   This bypasses gmsh's 2D remesher entirely, which is what dodges the
   periodic-surface failure.
2. Clean up the tessellation (merge coincident vertices, drop degenerate/
   duplicate faces).
3. Run pymeshfix (joining nearby open seams) to force a single watertight
   manifold shell -- required for correct SPH BCE generation.
4. Export the fine mesh as Wavefront OBJ (in meters; the glTF stage already
   converts out of the STEP file's native mm) for visualization.
5. Also emit a decimated, still-watertight, low-poly copy for BCE marker
   generation. ChFsiFluidSystemSPH.CreatePointsMesh does a brute-force
   O(num_grid_points * num_faces) ray-parity test with no spatial
   acceleration structure -- at the ~300k faces the raw tessellation has,
   that step alone would take hours. The wheel-soil coupling only needs
   marker resolution matching the SPH particle spacing anyway, so a ~10-15k
   face decimation loses no relevant detail there.

Run once:
    python prepare_wheel_mesh.py
Outputs (next to this script):
    RigidWheelSimTest1_mesh.obj       (fine, for visualization)
    RigidWheelSimTest1_mesh_bce.obj   (decimated, for BCE generation)
"""

import os

import cascadio
import trimesh
import pymeshfix

HERE = os.path.dirname(os.path.abspath(__file__))
STEP_IN = os.path.join(HERE, "RigidWheelSimTest1.step")
GLB_TMP = os.path.join(HERE, "_wheel_tmp.glb")
OBJ_OUT = os.path.join(HERE, "RigidWheelSimTest1_mesh.obj")
OBJ_BCE_OUT = os.path.join(HERE, "RigidWheelSimTest1_mesh_bce.obj")

# Tessellation tolerance in mm (the STEP file's native linear unit). 0.15 mm
# linear deflection is fine enough to keep the grouser profile crisp without
# an unmanageable triangle count.
TOL_LINEAR_MM = 0.15
TOL_ANGULAR_RAD = 0.2

# Target face count for the decimated BCE mesh.
BCE_TARGET_FACES = 12000


def main():
    print(f"tessellating {STEP_IN} with OpenCASCADE (tol={TOL_LINEAR_MM} mm)...")
    cascadio.step_to_glb(STEP_IN, GLB_TMP, tol_linear=TOL_LINEAR_MM, tol_angular=TOL_ANGULAR_RAD,
                          tol_relative=False)

    mesh = trimesh.load(GLB_TMP, force="mesh", process=False)
    print(f"raw tessellation: {len(mesh.vertices)} verts, {len(mesh.faces)} faces")

    # Coincident vertices from independently-tessellated adjacent faces don't
    # merge at full float precision; snap to ~1e-5 m before dedup.
    mesh.merge_vertices(digits_vertex=5)
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    print(f"after cleanup: {len(mesh.vertices)} verts, {len(mesh.faces)} faces, "
          f"watertight={mesh.is_watertight}")

    if not mesh.is_watertight:
        print("running pymeshfix to close remaining seams...")
        fixer = pymeshfix.MeshFix(mesh.vertices, mesh.faces)
        fixer.repair(joincomp=True, remove_smallest_components=False)
        mesh = trimesh.Trimesh(vertices=fixer.points, faces=fixer.faces, process=True)

    print(f"final: {len(mesh.vertices)} verts, {len(mesh.faces)} faces, "
          f"watertight={mesh.is_watertight}, bodies={mesh.body_count}, "
          f"volume={mesh.volume:.6e} m^3, extents={mesh.extents}")

    if not mesh.is_watertight:
        raise RuntimeError("mesh is still not watertight after repair -- "
                            "CreatePointsMesh's ray-parity test needs a closed shell")

    mesh.export(OBJ_OUT)
    print(f"wrote {OBJ_OUT}")

    print(f"decimating to ~{BCE_TARGET_FACES} faces for BCE generation...")
    bce_mesh = mesh.simplify_quadric_decimation(face_count=BCE_TARGET_FACES)
    bce_mesh.merge_vertices(digits_vertex=5)
    bce_mesh.update_faces(bce_mesh.nondegenerate_faces())
    bce_mesh.update_faces(bce_mesh.unique_faces())
    bce_mesh.remove_unreferenced_vertices()
    print(f"decimated: {len(bce_mesh.vertices)} verts, {len(bce_mesh.faces)} faces, "
          f"watertight={bce_mesh.is_watertight}, bodies={bce_mesh.body_count}")

    if not bce_mesh.is_watertight:
        print("decimation broke watertightness -- re-running pymeshfix...")
        fixer = pymeshfix.MeshFix(bce_mesh.vertices, bce_mesh.faces)
        fixer.repair(joincomp=True, remove_smallest_components=False)
        bce_mesh = trimesh.Trimesh(vertices=fixer.points, faces=fixer.faces, process=True)
        print(f"re-fixed: {len(bce_mesh.vertices)} verts, {len(bce_mesh.faces)} faces, "
              f"watertight={bce_mesh.is_watertight}, bodies={bce_mesh.body_count}")

    if not bce_mesh.is_watertight or bce_mesh.body_count != 1:
        raise RuntimeError("decimated BCE mesh is not a single watertight shell")

    bce_mesh.export(OBJ_BCE_OUT)
    print(f"wrote {OBJ_BCE_OUT} (volume={bce_mesh.volume:.6e} m^3, "
          f"vs fine mesh {mesh.volume:.6e} m^3)")

    os.remove(GLB_TMP)


if __name__ == "__main__":
    main()
