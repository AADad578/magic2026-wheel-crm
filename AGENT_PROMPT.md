# Prompt for your coding agent

Copy everything below the line into your coding agent (Claude Code, Cursor, etc.)
after unzipping this archive and opening the folder.

---

I've unzipped a PyChrono simulation package into this folder. I want you to get it
running on my machine and write me a `SETUP.md` that explains both how to run it and
what it actually simulates, so I can hand it to someone else later.

Work through this in order. Don't skip the chrono-rag step — it's the thing that makes
you accurate about Chrono, and this project's `CLAUDE.md` requires it.

## 1. Read what's already here first

- `CLAUDE.md` — project rules, plus a "CRM terrain gotchas" section written from things
  that already went wrong on this sim. Read it before changing anything.
- `magic_2026_crm_rassor_rig.py` — the simulation. Its module docstring explains the rig.
- `render_splashsurf.py` — the offline surface renderer.
- `prepare_wheel_mesh.py` — STEP CAD → watertight OBJ. Already run; its outputs ship
  in this zip, so you only need it if I replace the wheel.
- `inspect_bce.py` — diagnostic that renders the BCE marker cloud.
- `reference_run/` — a completed run to check my results against.

## 2. Set up chrono-rag (do this before writing any Chrono code)

chrono-rag is a local RAG index over Chrono's source, demos and docs, exposed to you as
an MCP server. Chrono's Python API drifts between releases and the online docs lag the
code, so guessing API names from memory produces code that doesn't run. Use it for every
Chrono-specific question: API names, signatures, soil parameters, demo patterns.

```bash
git clone https://github.com/uwsbel/chrono-rag.git
cd chrono-rag
./setup.sh                  # Windows PowerShell: .\setup.ps1
conda activate chrono-rag
chrono-rag search "CRM terrain SPH lunar regolith"   # confirm it works
chrono-rag mcp-config                                 # prints the MCP entry for this machine
```

`setup.ps1` creates a conda env named `chrono-rag` and downloads the prebuilt index
(~90 MB). Use the **default `main` channel** — do *not* run `get-index --channel 10.0.0`.
The conda `pychrono` package is labelled 10.0.0 but the build used here (`py312_1187`)
carries main-branch APIs: `SoilProperties` not `ElasticMaterialProperties`, `SetCrmSPH`
not `SetElasticSPH`, `AddRigidBody` not `AddFsiBody`. The 10.0.0 index will tell you the
wrong names.

Take the output of `chrono-rag mcp-config` and register it. `.mcp.json.example` in this
zip shows the shape (its paths are from the original machine — replace them with yours):

```bash
claude mcp add-json chrono-rag '<the chrono-rag object from mcp-config>'
```

Then restart, and confirm you can call `search_chrono` and `chrono_digest`.
If you can't call those two tools, stop and fix that before continuing.

## 3. Build the simulation environment

Needs an NVIDIA GPU with CUDA — Chrono's SPH solver is GPU-only. The reference run used
a 6 GB RTX 4050; budget roughly 100–300k SPH particles on 6 GB.

```bash
conda create -n chrono -c projectchrono -c conda-forge python=3.12 pychrono
conda activate chrono
pip install numpy pyvista trimesh imageio imageio-ffmpeg pysplashsurf
pip install cascadio pymeshfix          # only needed to re-run prepare_wheel_mesh.py
```

Reference versions: pychrono 10.0.0 build `py312h418371c_1187`, CUDA 12.8, Python 3.12.14,
numpy 2.5.3, pyvista 0.49.0, trimesh 5.1.0, pysplashsurf 0.14.1.0, vtk 9.7.0.

**Windows gotcha:** `<env>\Library\bin` must be on `PATH` or the FSI/VSG/CUDA DLLs fail to
load with an unhelpful "DLL load failed" on `import pychrono.fsi`. Put that in the run
instructions.

Verify before running anything long:

```bash
python -c "import pychrono.fsi as fsi; print(hasattr(fsi, 'RheologyCRM_MCC'))"
```

Must print `True`. If it's `False` the build is too old for the MCC rheology this sim
uses, and you'll need a newer pychrono.

## 4. Reproduce the reference run

This is the exact configuration in `reference_run/`:

```bash
python -u magic_2026_crm_rassor_rig.py --body=wheel --spacing=0.0035 --length=0.7 --width=0.19 --depth=0.13 --load_kg=15 --slip=0.10 --soil=bp1 --density=1430 --mus=0.81 --rheology=mcc --bcethicken=3 --tend=5 --norender --dump --dumpfps=15 --tag=mcc15kg_5s_thick
```

It writes `run_<YYYYMMDD_HHMM>_mcc15kg_5s_thick/` containing `rig_results.csv`,
`rig_summary.png`, and `particles/` (per-frame SPH dumps, ~300 MB).

Expect **~47 minutes of wall time for 4.05 s of simulated time** on an RTX 4050 — about
700× slower than real time, measured, not estimated. Run it in the background and tail the
log; don't block on it. Note the per-step `rtf=` printed in the log is an instantaneous
number and is garbage at the extremes (you'll see 543 next to 94562 on adjacent steps) —
ignore it and measure wall time over the whole run if you care.
  
It stops at **4.05 s**, not 5 s, because the wheel hits the 0.150 m travel limit on a
0.70 m bed. A genuine 5 s run needs `--length=0.8`, which costs more particles.

Then render:

```bash
python -u render_splashsurf.py --dir=run_<stamp>_mcc15kg_5s_thick --spacing=0.0035 --smoothing=2.5 --threshold=0.45 --cube=0.22 --smooth=12 --fps=15 --dropejecta
```

~35 s per frame, 61 frames. Writes `regolith_surface.mp4` in the run folder.

## 5. Check the numbers

Compare against `reference_run/rig_results.csv`. Converged values at the end of the run:

| quantity | value |
|---|---|
| sinkage | 23.1 mm |
| drawbar pull | +69.8 N |
| drive torque | 13.4 N·m |
| vertical load Fz | 148.8 N (target 147.15 N) |
| duration | 4.05 s (travel limit) |

Fz tracking the 147 N target confirms the load path and soil support are working. If Fz
collapses toward zero, the wheel is falling through unsupported soil — see the active-domain
note in `CLAUDE.md`.

## 6. Write `SETUP.md`

Cover install, the run commands, and expected output — and also explain the physics, because
whoever reads it next won't have this context. Use `search_chrono` to verify anything you
state about Chrono's API or models; don't restate my summary below without checking it.

Here's the context to work from:

**What it is.** A single-wheel terramechanics test: a grousered rigid wheel towed through a
bed of lunar regolith simulant at prescribed slip, measuring drawbar pull, drive torque and
sinkage. The kind of test you'd run on a physical soil-bin rig, done numerically.

**The rig** is a three-link kinematic chain:
`ground → [linear speed motor, X] → carriage_x → [free prismatic, Z] → carriage_z → [rotation speed motor, Y] → wheel`.
Tow speed and spin speed are both prescribed to hit a target slip (10% here). The Z joint
is free and carries the dead load, so the wheel finds its own sinkage. Drawbar pull is the
X motor's reaction force; drive torque is the Y motor's reaction.

**The wheel** comes from `RigidWheelSimTest1.step`: R = 0.160 m, width 0.127 m,
volume 5.152e-4 m³, aluminium (2700 kg/m³) → 1.391 kg. The 15 kg load is carriage dead
mass, so the wheel itself is light and the load dominates.

**The soil** is CRM — Chrono's Continuum Representation Model, an elastoplastic continuum
discretized with SPH. Critically, **SPH particles are not grains.** `initial_spacing`
(0.0035 m here) is the numerical discretization; `average_diam` (1e-4 m) is the constitutive
model's representative grain size. Coarse-looking particles in a render mean coarse spacing,
not coarse soil. This confused the original project for a while.

Soil parameters are BP-1 (Black Point-1) lunar regolith simulant in a loose state:
density 1430 kg/m³, cohesion 1 kPa, friction coefficient 0.81 (φ ≈ 39°), E = 1e6 Pa, ν = 0.3.
The script also ships `--soil=chrono`, which is Chrono's own lunar demo soil
(1700 kg/m³, 5 kPa cohesion, μ = 0.7) for comparison — note its cohesion is well above
anything published for BP-1.

**The rheology choice matters more than anything else here.** The script supports two:

- `--rheology=mui` — μ(I), yielding at `tau_max = μ(I)·p + c` with
  `I = χ·d·√(ρ/p)` and `μ = μ_s + (μ_2−μ_s)·I/(I_0+I)`. Pressure-dependent friction, the
  standard choice for granular flow.
- `--rheology=mcc` — Modified Cam-Clay, a critical-state model with compaction hardening
  and a per-particle preconsolidation pressure.

μ(I) **carries no density or compaction state**. Soil under the wheel shears and flows but
never densifies, so the bed relaxes back and **no rut or grouser imprint survives behind the
wheel**. No amount of tuning load, slip, spacing or render settings fixes that — it's
structural to the model. Switching to MCC produced a persistent rut immediately and nearly
doubled drawbar pull. That's why the reference run is MCC.

MCC parameters: `M = 1.593` derived from BP-1's measured friction angle via
`M = 6·sin(φ)/(3−sin(φ))`; `kappa = 0.01`, `lambda = 0.04`, `v_lambda = 2.0`. Be honest in
the doc that kappa and lambda are **not published for BP-1** — they're Chrono's lunar
lander CRM values, carried over. M is the only MCC parameter actually grounded in BP-1 data.

Also worth stating: cohesion is load-bearing, not cosmetic. With `cohesion = 0`, `p → 0` at
the free surface means the top of the bed has zero strength and anything resting on it sinks
without limit. This bit the project early (180 mm of runaway sinkage).

**BCE markers** are the only representation of a solid that the SPH soil sees — the visual
mesh is irrelevant to the physics. They're generated from the decimated OBJ by
`CreatePointsMesh`, a ray-parity interior fill on a grid at `initial_spacing`. Chrono wants
**≥ 3 marker layers across any wall** for the SPH kernel to have support. This wheel's rim
between grousers is thin enough that the fill gave only 1–2 layers, and **soil particles
tunnelled straight through the rim**. `--bcethicken=3` adds an inner annulus of markers
(+23,828, to 35,940 total) and fixes it. Run `inspect_bce.py` to see the cloud and its
radial histogram — the cutaway view is what diagnosed this.

**Scale:** 420,090 SPH particles + 102,333 boundary BCE markers at 3.5 mm spacing.

**Rendering.** PyChrono's conda build is compiled without `CHRONO_HAS_SPLASHSURF`, so
Chrono's internal surface reconstruction (`ChFsiSplashsurfSPH`, `WriteReconstructedSurface`)
is unavailable — the DLL only contains the "not available" branch. `render_splashsurf.py`
is an offline pass that does the same job: `--dump` writes per-frame particle positions to
`.npy`, then pysplashsurf runs marching cubes over the SPH density field and PyVista renders
it, which turns a cloud of spheres into a continuous dusty surface. `--dropejecta` removes
airborne debris by connected component: any component whose lowest point touches the bed is
kept, fully airborne ones are dropped. (An earlier size-threshold version made clumps pop in
and out between frames.)

**What's NOT here:** there is no RASSOR rover model in Chrono, despite the script's name.
`pychrono.robot` has Viper, Curiosity, Turtlebot, RoboSimian and copters — the `RS_*`
classes are RoboSimian, not RASSOR. RASSOR ships only as a single drum
(`robot/rassor/obj/single_drum.obj` + `bce/single_drum.csv`) used by
`demo_FSI-SPH_RassorDrum.cpp`. The script's `--body=drum` mode runs that drum on the same
rig for comparison, and in that mode `initial_spacing` is forced to 0.004 m because the
shipped BCE cloud is built on a 4 mm grid and BCE spacing must equal SPH spacing.

Keep `SETUP.md` practical. Put the gotchas next to the steps they'd bite on, not in a
footnote at the end.
