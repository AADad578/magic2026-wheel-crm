# Working in this repo

## PyChrono / Chrono questions
- Use the `chrono-rag` MCP server (`search_chrono`, `chrono_digest`) for anything
  Chrono-specific: API names, signatures, soil/material parameters, demo patterns.
  Do not guess API names or physical constants from memory.
- This install tracks Chrono `main`, not the 10.0 docs: e.g. `SoilProperties`
  (not `ElasticMaterialProperties`), `SetCrmSPH` (not `SetElasticSPH`),
  `AddRigidBody` (not `AddFsiBody`). Verify against rag when in doubt.

## Responses
- Short and direct. No preamble, no restating the question, no summary of what
  you just did unless asked.
- Answer the question that was asked. Don't volunteer adjacent analysis.
- Recommend one approach with its tradeoff; don't survey options.

## Verification
- Don't run exhaustive checks by default. Make the reasonable call and proceed.
- Verify only when: the check is cheap and the failure is expensive (e.g. a long
  GPU run), the user asks, or you're about to report something as working.
- Don't re-read files you just wrote. Don't re-derive facts already established.

## Code
- Edit existing files over creating new ones. No new .md docs unless asked.
- No comments unless the *why* is non-obvious. Never narrate what the code does.
- No defensive error handling for cases that can't happen, no backwards-compat
  shims, no abstractions built for hypothetical future needs.

## Environment
- Run Python via `C:\Users\avani\miniforge3\envs\chrono\python.exe` with
  `<env>\Library\bin` on PATH, or the FSI/VSG/CUDA DLLs fail to load.
- Use `python -u` for long runs so the log is readable while running.
- GPU is a 6 GB RTX 4050. Budget ~100-300k SPH particles. Run anything long in
  the background and watch the log; don't block on it.

## CRM terrain gotchas (learned the hard way here)
- Prefer `veh.CRMTerrain` + `Construct()` + `Initialize()` over hand-rolling the
  bed with `ChGridSampler` + `AddSPHParticle`. The hand-rolled path has to be
  kept consistent with `d0_multiplier` / `free_surface_threshold` by hand;
  mixing one demo's seeding with another's SPH numerics blows up (a 33x contact
  force spike that launched the wheel, in one case).
- Cohesion is load-bearing, not cosmetic. mu(I) yields at `tau = mu*p + c`, and
  `p -> 0` at a free surface, so `cohesion_coeff = 0` gives the top of the bed
  zero strength and anything resting on it sinks without limit. Chrono's own
  lunar demos use 5 kPa. The mu(I) guide says to add cohesion exactly when
  low-pressure strength is under-predicted.
- BCE marker spacing must equal `initial_spacing`. The shipped RASSOR drum cloud
  (`robot/rassor/bce/single_drum.csv`) is built on a 4 mm grid, so that sim is
  locked to `spacing = 0.004`.
- The bed must be bigger than the body. Markers outside the computational domain
  segfault inside the spatial hashing with no useful error.
- `initial_spacing` (SPH discretization, what you see rendered) is not
  `average_diam` (mu(I) grain size). Coarse-looking particles = spacing.
- Raising `Young_modulus` without shrinking the time step is unstable: 10x E at
  the same dt ejected particles out of the domain immediately.

## What exists for RASSOR
There is no RASSOR rover model in Chrono. `pychrono.robot` has Viper, Curiosity,
Turtlebot, RoboSimian, copters — the `RS_*` classes are RoboSimian, not RASSOR.
RASSOR ships only as a single drum (`robot/rassor/obj/single_drum.obj` +
`bce/single_drum.csv`) used by `demo_FSI-SPH_RassorDrum.cpp`.

## STEP CAD -> Chrono
`prepare_wheel_mesh.py` handles it: cascadio (OpenCASCADE tessellation) ->
trimesh cleanup -> pymeshfix to force watertight -> OBJ. gmsh's remesher fails
on this geometry ("Impossible to mesh periodic surface"). `CreatePointsMesh`
needs a watertight shell and is O(gridpoints x faces) brute force, so it gets a
decimated copy while visualization keeps the fine mesh.
