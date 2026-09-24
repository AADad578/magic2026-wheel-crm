"""
MAGIC 2026 - RASSOR-style single-drum/wheel rig on fine CRM lunar regolith

WHAT THIS IS
The RASSOR excavation drum (or YOUR RigidWheelSimTest1 wheel, swapped in for
it) driven through a bed of CRM lunar regolith at fine resolution, on the same
rig and the same soil so the two can be compared directly.

WHAT SHIPS WITH CHRONO, AND WHAT DOESN'T (read this before looking for
"the RASSOR demo")
There is NO full RASSOR rover model in Chrono. `pychrono.robot` exposes
Viper, Curiosity, Turtlebot, RoboSimian and the copters -- the `RS_*` classes
in that module are RoboSimian parts, not RASSOR. What Chrono actually ships
for RASSOR is a *single drum*:
    data/robot/rassor/obj/single_drum.obj   (visual mesh, 0.43 m dia x 0.098 m)
    data/robot/rassor/bce/single_drum.csv   (34333 pre-made BCE markers)
used by the C++ demo src/demos/fsi/sph/demo_FSI-SPH_RassorDrum.cpp. The full
rover in the SBEL slide is their research code, not a shipped demo. This file
is a Python port of that single-drum CRM rig, generalized so the drum can be
swapped for an arbitrary wheel.

    python magic_2026_crm_rassor_rig.py --body=drum    # the RASSOR drum
    python magic_2026_crm_rassor_rig.py --body=wheel   # YOUR wheel, same soil

PARTICLE SIZE vs GRAIN SIZE (these are different things)
The big spheres in the earlier rover run were the SPH *discretization*, not
grains: initial_spacing was 0.03 m. The RASSOR drum's shipped BCE cloud is
built at 0.004 m, which is the resolution that demo actually runs at -- ~7x
finer, and it looks like soil rather than marbles. `average_diam` is a
separate quantity: the mu(I) rheology's representative grain diameter, which
enters the inertial number I = Chi * d * sqrt(rho/p). Real lunar regolith is
~70 um; Chrono's terramechanics demos all use 0.005 m, so that is the default
here, with --graindiam to explore finer values.

IMPORTANT: in --body=drum mode initial_spacing is FORCED to 0.004 m, because
the shipped BCE cloud was generated at that spacing and BCE markers must match
the SPH spacing. In --body=wheel mode the markers are generated on the fly
from your mesh, so --spacing is free.

SOIL (demo_ROBOT_Viper_CRM.cpp lunar regolith, via chrono-rag)
    density 1700 kg/m^3, cohesion 5 kPa, friction 0.7, E 1e6 Pa, nu 0.3
Cohesion is what makes regolith load-bearing: the mu(I) yield surface is
tau_max = mu*p + c, and p -> 0 at a free surface, so a cohesionless bed has
zero strength at the top and anything resting on it sinks without limit.

RIG
Same three-link single-wheel testbed used in magic_2026_crm_rigid_wheel_test:
    ground --[MotorLinearSpeed, X]--> carriage_x
           --[LockPrismatic,    Z]--> carriage_z (carries the dead load)
           --[MotorRotationSpeed,Y]--> drum/wheel
Tow speed and spin speed are prescribed for a target slip; the X motor's
reaction is drawbar pull, the Y motor's reaction is drive torque.

Flags: --body=drum|wheel --slip=0.3 --load_lb=20 --spacing=0.006 --tend=6
       --graindiam=0.005 --color=grey|velocity --lunar --smoke --tag=name
Outputs -> Rassor_results_<body>[_tag]/ : telemetry CSV, frames/, mp4, plot
"""

import math
import os
import sys

import pychrono as chrono
import pychrono.fsi as fsi
import pychrono.vehicle as veh

try:
    import pychrono.vsg3d as vsg3d
    HAVE_VSG = True
except Exception:
    HAVE_VSG = False


def _cli_float(flag, default):
    for a in sys.argv:
        if a.startswith(flag + "="):
            return float(a.split("=", 1)[1])
    return default


def _cli_str(flag, default):
    for a in sys.argv:
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


SMOKE = "--smoke" in sys.argv
LUNAR_G = "--lunar" in sys.argv
BODY = _cli_str("--body", "wheel")         # "wheel" (yours) or "drum" (RASSOR)
COLOR_MODE = _cli_str("--color", "grey")
NORENDER = "--norender" in sys.argv
TAG = _cli_str("--tag", "")

if BODY not in ("drum", "wheel"):
    raise ValueError("--body must be 'drum' or 'wheel'")

HERE = os.path.dirname(os.path.abspath(__file__))
# timestamped so runs sort chronologically and never silently overwrite
_STAMP = __import__("time").strftime("%Y%m%d_%H%M")
OUT_DIR = os.path.join(HERE, f"run_{_STAMP}_{TAG if TAG else BODY}")
FRAMES_DIR = os.path.join(OUT_DIR, "frames")
os.makedirs(FRAMES_DIR, exist_ok=True)

GRAVITY = 1.62 if LUNAR_G else 9.81
LB_TO_N = 4.4482216153
# --load_kg is the primary: the rig load is a mass carried on the vertical
# slide, so kg is the natural unit. --load_lb kept for the earlier runs.
_load_kg = _cli_float("--load_kg", 0.0)
if _load_kg > 0:
    normal_load_N = _load_kg * GRAVITY
else:
    normal_load_N = _cli_float("--load_lb", 20.0) * LB_TO_N

# ---------------------------------------------------------------------------
# body geometry
# ---------------------------------------------------------------------------
if BODY == "drum":
    DRUM_CSV = chrono.GetChronoDataFile("robot/rassor/bce/single_drum.csv")
    DRUM_OBJ = chrono.GetChronoDataFile("robot/rassor/obj/single_drum.obj")
    RADIUS = 0.215          # m, measured from the shipped assets (0.43 m dia)
    WIDTH = 0.098           # m
    VOLUME = 6.0e-4         # m^3, approximate -- drum is a thin shell + vanes
    # the shipped BCE cloud is built on a 4 mm grid; SPH spacing must match it
    spacing = 0.004
    body_rot = chrono.QUNIT          # drum axle is already along local Y
    MESH_VIS = DRUM_OBJ
else:
    MESH_VIS = os.path.join(HERE, "RigidWheelSimTest1_mesh.obj")
    MESH_BCE = os.path.join(HERE, "RigidWheelSimTest1_mesh_bce.obj")
    for f in (MESH_VIS, MESH_BCE):
        if not os.path.exists(f):
            raise FileNotFoundError(f"{f} not found -- run prepare_wheel_mesh.py first")
    RADIUS = 0.160
    WIDTH = 0.127
    VOLUME = 5.152e-4
    spacing = _cli_float("--spacing", 0.005)
    body_rot = chrono.QuatFromAngleZ(chrono.CH_PI / 2)   # mesh axle is local X
    if "--spacing" not in " ".join(sys.argv):
        pass

BODY_DENSITY = 2700.0     # aluminum

# ---------------------------------------------------------------------------
# terrain / run parameters
# ---------------------------------------------------------------------------
# The bed must actually FIT the body: a box shorter than the body diameter
# puts BCE markers outside the computational domain, which segfaults in the
# spatial hashing rather than raising anything useful. These floors are
# derived from the body, not guessed.
MIN_LENGTH = 2.2 * RADIUS + 0.20     # room to sit, plus travel
MIN_DEPTH = 0.6 * RADIUS             # deep enough not to bottom out
MIN_WIDTH = WIDTH + 0.05

if SMOKE:
    # In drum mode the spacing must stay at 0.004 -- the shipped BCE cloud is
    # built on that grid -- so a fast smoke run shrinks the *box*, never the
    # spacing. Only wheel mode (markers generated on the fly) may coarsen.
    if BODY == "wheel":
        spacing = max(spacing, 0.012)
    bxDim, byDim, bzDim = MIN_LENGTH, MIN_WIDTH, MIN_DEPTH
    settle_time, ramp_duration, tend = 0.15, 0.1, 0.5
    render_fps = 20
else:
    # sized to the travel the run actually needs: every extra cm of bed is
    # spacing^-3 more particles
    bxDim = max(_cli_float("--length", 0.75), MIN_LENGTH)
    byDim = max(_cli_float("--width", WIDTH + 0.09), MIN_WIDTH)
    bzDim = max(_cli_float("--depth", 0.12), MIN_DEPTH)
    settle_time, ramp_duration = 0.4, 0.3
    tend = _cli_float("--tend", 5.0)
    render_fps = 30

step_size = 2.5e-4
exchange_info = 4 * step_size
# Full box size (not half-extents), centred on each registered FSI body. Sized
# to the wheel's actual disturbance region -- a box wider/deeper than the bed
# excludes nothing and buys no speedup. Chrono measured 2.1x (RASSOR drum) to
# 2.9x (MGRU3 wheel) from active domains alone.
active_box = chrono.ChVector3d(
    _cli_float("--abox", 2.2 * RADIUS),
    byDim,
    # z MUST reach from the body centre (a radius above the surface) down
    # through the whole bed, or no soil particle is ever active and the body
    # falls through unsupported. Only x/y buy anything here anyway.
    2.0 * (RADIUS + bzDim + 0.02))
log_every = 6

slip_target = _cli_float("--slip", 0.30)
tow_speed = _cli_float("--tow", 0.08)
ang_speed = tow_speed / (RADIUS * (1.0 - slip_target))

# --------------------------------------------------------------------------
# REGOLITH. Rheology is mu(I): yield at tau_max = mu(I)*p + c, with
#   I = Chi * d * sqrt(rho/p),  mu = mu_s + (mu_2 - mu_s) * I/(I0 + I)
# mu(I) is the right model for traction/bearing (pressure-dependent friction).
# Note it carries no density/compaction state, so sinkage here comes from shear
# failure and flow, not densification -- Chrono's other CRM option (MCC) is the
# one that captures compaction hardening, but BP-1's published data constrains
# friction and cohesion, not MCC's kappa/lambda.
#
# --soil=bp1 (default): BP-1 "Black Point-1" simulant. MEASURED values, from
# the NASA ARES simulant page (https://ares.jsc.nasa.gov/projects/simulants/bp-1.html):
#     bulk density   1.5-1.6 g/cm^3 (min 1.43, max 1.86)  -> 1600 kg/m^3
#     friction angle 39-51 deg (triaxial)                 -> mu_s = tan(45) = 1.0
#     cohesion       0-2.0 kPa (triaxial)                 -> 1.0 kPa (midpoint)
#     mean particle size 100 um                           -> average_diam = 1e-4
# Ranges are wide; each is taken at its midpoint and is overridable.
#
# The rest is NOT measured for BP-1, so it is inherited from the lunar regolith
# Chrono already ships (demo_ROBOT_Viper_CRM.cpp), rather than invented here:
#   Young_modulus = 1e6 Pa, Poisson_ratio = 0.3, mu_I0 = 0.04
#   mu_fric_2 = mu_fric_s -- Chrono's lunar soil also sets the two equal (0.7),
#     i.e. no inertial rate-hardening. That holds even better at BP-1's 100 um
#     grain size, where the inertial number is negligible at rover speeds.
# So the only BP-1-specific departures from Chrono's lunar soil are the four
# measured quantities above: density, mu_s, cohesion, average_diam.
#
# --soil=chrono is Chrono's own lunar demo soil, kept for comparison. Note its
# cohesion (5 kPa) is well above anything measured for BP-1.
SOIL = _cli_str("--soil", "bp1")
if SOIL == "chrono":
    soil_density = 1700.0
    soil_cohesion = _cli_float("--cohesion", 5e3)
    soil_friction_s = 0.7
    soil_friction_2 = 0.7
    soil_mu_I0 = 0.04
    grain_diam = _cli_float("--graindiam", 0.005)
else:
    soil_density = _cli_float("--density", 1600.0)
    soil_cohesion = _cli_float("--cohesion", 1.0e3)
    soil_friction_s = _cli_float("--mus", 1.0)
    soil_friction_2 = _cli_float("--mu2", soil_friction_s)
    soil_mu_I0 = 0.04
    grain_diam = _cli_float("--graindiam", 1e-4)
soil_E = _cli_float("--E", 1e6)
soil_nu = 0.3

est_particles = int(bxDim * byDim * bzDim / spacing ** 3)
print(f"body={BODY}  radius={RADIUS} m  width={WIDTH} m")
print(f"spacing={spacing} m  box={bxDim}x{byDim}x{bzDim} m  ~{est_particles} SPH particles")
print(f"gravity={GRAVITY} m/s^2  load={normal_load_N:.2f} N "
      f"({normal_load_N / GRAVITY:.1f} kg / {normal_load_N / LB_TO_N:.1f} lbf)  "
      f"slip={slip_target * 100:.0f}%  tow={tow_speed} m/s  spin={ang_speed:.3f} rad/s")
print(f"regolith[{SOIL}]: rho={soil_density} cohesion={soil_cohesion:.0f}Pa "
      f"mu_s={soil_friction_s} mu_2={soil_friction_2} mu_I0={soil_mu_I0} "
      f"E={soil_E:.0f}Pa grain_diam={grain_diam} m")

# ---------------------------------------------------------------------------
# system + terrain
# ---------------------------------------------------------------------------
sysMBS = chrono.ChSystemNSC()
sysMBS.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sysMBS.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -GRAVITY))

terrain = veh.CRMTerrain(sysMBS, spacing)
sysFSI = terrain.GetFsiSystemSPH()
sysSPH = terrain.GetFluidSystemSPH()
_gpu_check = getattr(sysSPH, "EnableGPUErrorCheck", None) or getattr(sysSPH, "EnableCudaErrorCheck", None)
if _gpu_check:
    _gpu_check("--gpucheck" in sys.argv)
terrain.SetVerbose(False)
terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -GRAVITY))
terrain.SetStepSizeCFD(step_size)
terrain.SetStepsizeMBD(step_size)

mat_props = fsi.SoilProperties()
mat_props.density = soil_density
mat_props.Young_modulus = soil_E
mat_props.Poisson_ratio = soil_nu
RHEOLOGY = _cli_str("--rheology", "mui")
if RHEOLOGY == "mcc":
    # Modified Cam-Clay: unlike mu(I) this carries a density/consolidation
    # state, so soil compacts under the wheel instead of only shearing and
    # flowing. That compaction is what preserves a grouser imprint.
    # M comes from BP-1's measured friction angle; kappa/lambda are not
    # published for BP-1, so they are Chrono's lunar lander CRM values.
    _phi = math.atan(soil_friction_s)
    mat_props.rheology_model = fsi.RheologyCRM_MCC
    mat_props.mcc_M = _cli_float("--mcc_M", 6 * math.sin(_phi) / (3 - math.sin(_phi)))
    mat_props.mcc_kappa = _cli_float("--mcc_kappa", 0.01)
    mat_props.mcc_lambda = _cli_float("--mcc_lambda", 0.04)
    mat_props.mcc_v_lambda = _cli_float("--mcc_vlambda", 2.0)
    if not (mat_props.mcc_lambda > mat_props.mcc_kappa > 0 and mat_props.mcc_M > 0):
        raise ValueError("MCC requires kappa > 0 and lambda > kappa and M > 0")
    print(f"rheology=MCC  M={mat_props.mcc_M:.3f} (phi={math.degrees(_phi):.1f} deg) "
          f"kappa={mat_props.mcc_kappa} lambda={mat_props.mcc_lambda}")
else:
    mat_props.rheology_model = fsi.RheologyCRM_MU_OF_I
    mat_props.mu_I0 = soil_mu_I0
    mat_props.mu_fric_s = soil_friction_s
    mat_props.mu_fric_2 = soil_friction_2
    mat_props.average_diam = grain_diam
    mat_props.cohesion_coeff = soil_cohesion
terrain.SetCrmSPH(mat_props)

sph_params = fsi.SPHParameters()
sph_params.integration_scheme = fsi.IntegrationScheme_RK2
sph_params.initial_spacing = spacing
sph_params.d0_multiplier = 1.0
sph_params.free_surface_threshold = 0.8
sph_params.artificial_viscosity = 0.5
sph_params.use_consistent_gradient_discretization = False
sph_params.use_consistent_laplacian_discretization = False
sph_params.viscosity_method = fsi.ViscosityMethod_ARTIFICIAL_BILATERAL
sph_params.boundary_method = fsi.BoundaryMethod_ADAMI
sph_params.use_variable_time_step = True
# Persistent neighbour lists: rebuild the neighbour list every N steps instead
# of every step. Chrono reports 1.28-1.36x with no significant accuracy loss at
# N=10; their guidance is to step 1 -> 4 -> 10 and compare sinkage/traction.
sph_params.num_proximity_search_steps = int(_cli_float("--pssteps", 1))
terrain.SetSPHParameters(sph_params)
terrain.SetOutputLevel(getattr(fsi, "OutputLevel_STATE", 0))

PRE_SCALE = _cli_float("--prescale", 2.0)


class _SoilInit(fsi.ParticlePropertiesCallback):
    """Geostatic initial state. MCC needs a per-particle preconsolidation
    pressure; the base class field is `consolidation_pressure` here (an
    absolute pressure), not the `pre_pressure_scale0` the manual names."""

    def __init__(self, zero_height, scale):
        fsi.ParticlePropertiesCallback.__init__(self)
        self.zero_height = zero_height
        self.scale = scale

    def set(self, sysSPH, pos):
        gz = abs(sysSPH.GetGravitationalAcceleration().z)
        # MCC asserts pressure > 0 and consolidation_pressure >= pressure.
        # Geostatic pressure is exactly zero at the free surface, so it needs
        # a small positive floor (tiny next to the ~1.4 kPa at bed bottom).
        depth_p = max(sysSPH.GetDensity() * gz * (self.zero_height - pos.z), 50.0)
        self.p0 = depth_p
        self.rho0 = sysSPH.GetDensity()
        self.mu0 = sysSPH.GetViscosity()
        self.v0 = chrono.ChVector3d(0, 0, 0)
        self.tau_diag = chrono.ChVector3d(-depth_p, -depth_p, -depth_p)
        self.tau_offdiag = chrono.ChVector3d(0, 0, 0)
        # floored: MCC requires p_c > 0, and p0 goes to zero at the surface
        self.consolidation_pressure = max(self.scale * depth_p, depth_p, 100.0)



# ---------------------------------------------------------------------------
# the drum / wheel body
# ---------------------------------------------------------------------------
body_mass = BODY_DENSITY * VOLUME
I_axial = 0.5 * body_mass * RADIUS ** 2
I_lat = body_mass * (3 * RADIUS ** 2 + WIDTH ** 2) / 12.0

# ConstructMovingPatch places the bed with its LOWEST CORNER at the global
# origin (x in [0,Lx], y in [0,Ly], free surface at z = Lz) and always moves
# in +x, whereas plain Construct is centred in x/y with the surface at z = 0.
# The rig frame has to follow whichever one is in use.
MOVING_PATCH = "--movingpatch" in sys.argv
if MOVING_PATCH:
    surface_z = bzDim
    y_center = byDim / 2
    start_x = 1.6 * RADIUS
    patch_buffer = _cli_float("--buffer", 1.5 * RADIUS)
    patch_shift = _cli_float("--shift", 1.2 * RADIUS)
else:
    surface_z = 0.0
    y_center = 0.0
    start_x = -bxDim / 2 + RADIUS + 0.06
# just kissing the surface: dropping the wheel in from a height punches a
# crater and contaminates the first second of the run
GAP = _cli_float("--gap", 0.001)      # negative = start embedded in the bed
start_z = surface_z + RADIUS + GAP

body = chrono.ChBody()
body.SetPos(chrono.ChVector3d(start_x, y_center, start_z))
body.SetRot(body_rot)
body.SetMass(body_mass)
if BODY == "drum":
    body.SetInertiaXX(chrono.ChVector3d(I_lat, I_axial, I_lat))   # axle = local Y
else:
    body.SetInertiaXX(chrono.ChVector3d(I_axial, I_lat, I_lat))   # axle = local X
body.EnableCollision(False)
sysMBS.AddBody(body)

vis_mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(MESH_VIS, True, False)
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(vis_mesh)
vis_shape.SetColor(chrono.ChColor(0.55, 0.55, 0.6))
body.AddVisualShape(vis_shape)

if BODY == "drum":
    print(f"reading shipped RASSOR BCE cloud: {os.path.basename(DRUM_CSV)}")
    bce_pts = chrono.vector_ChVector3d()
    with open(DRUM_CSV) as f:
        f.readline()                                  # header: x,y,z
        for line in f:
            v = line.strip().split(",")
            if len(v) >= 3:
                bce_pts.push_back(chrono.ChVector3d(float(v[0]), float(v[1]), float(v[2])))
    # False, not the demo's True: that prunes SPH particles inside the body,
    # which is for a drum that starts buried. Here it starts above the bed.
    check_embedded = False
else:
    print("generating BCE markers from your wheel mesh (ray-parity test)...")
    bce_mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(MESH_BCE, False, False)
    bce_pts = sysSPH.CreatePointsMesh(bce_mesh)
    # CreatePointsMesh traces the mesh interior, and this wheel's rim wall is
    # only ~2 markers thick between grousers -- below the >=3 layers the SPH
    # kernel needs, so soil tunnels straight through it. Thicken the wall
    # inward with a solid annulus (Chrono's own Viper wheel BCE generator
    # fills the wheel solid for the same reason).
    _thick = _cli_float("--bcethicken", 3)          # extra layers, 0 disables
    if _thick > 0:
        # CreatePointsMesh hands back a tuple, so rebuild as a chrono vector
        _pts = chrono.vector_ChVector3d()
        for _p in bce_pts:
            _pts.push_back(chrono.ChVector3d(_p.x, _p.y, _p.z))
        bce_pts = _pts
        _r_out = 0.126                               # measured inner edge of the
        _r_in = _r_out - _thick * spacing            # mesh-derived marker shell
        _n = 0
        _x = -WIDTH / 2
        while _x <= WIDTH / 2:
            _y = -_r_out
            while _y <= _r_out:
                _z = -_r_out
                while _z <= _r_out:
                    _rr = math.hypot(_y, _z)
                    if _r_in <= _rr <= _r_out:
                        bce_pts.push_back(chrono.ChVector3d(_x, _y, _z))
                        _n += 1
                    _z += spacing
                _y += spacing
            _x += spacing
        print(f"thickened rim wall by {_thick:.0f} layers: +{_n} markers")
    # starting embedded means BCE markers overlap live SPH particles; without
    # pruning them the overlap resolves as a violent ejection at t=0
    check_embedded = GAP < 0

print(f"BCE marker count: {len(bce_pts)}")
if "--nobce" not in sys.argv:
    terrain.AddRigidBody(body, bce_pts,
                         chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         check_embedded)

# An inactive particle is not integrated -- it freezes exactly where it is.
# Soil thrown clear of the active box therefore hangs in mid-air instead of
# falling back, which reads as "the particles don't flow". --noactive trades
# the 2-3x speedup for ejecta that actually obey gravity.
if "--noactive" not in sys.argv:
    terrain.SetActiveDomain(active_box)
# NOT settle_time: the delay FREEZES the soil until it expires, so the body
# rests on inactive particles that behave like a rigid floor and report no FSI
# force. That is meant for a vehicle settling before SPH turns on -- here the
# settle phase is exactly when the wheel should be sinking into live soil.
_delay = getattr(sysSPH, "SetActiveDomainDelay", None)
if _delay:
    _delay(_cli_float("--adelay", 0.0))

print("constructing regolith bed...")
# surface sits at z=bzDim for a moving patch, z=0 otherwise
_soil_init = _SoilInit(bzDim if MOVING_PATCH else 0.0, PRE_SCALE)
terrain.RegisterParticlePropertiesCallback(_soil_init)

if MOVING_PATCH:
    # Soil behind the wheel is relocated in front of it, so bed length stops
    # scaling with travel distance. `buffer` is the look-ahead distance from
    # the sentinel body to the front boundary that triggers a shift.
    print(f"moving patch: buffer={patch_buffer:.3f} m, shift={patch_shift:.3f} m")
    terrain.ConstructMovingPatch(
        chrono.ChVector3d(bxDim, byDim, bzDim), body, patch_buffer, patch_shift)
else:
    # Construct's `pos` is the centre of the BOTTOM face (particles are
    # generated above it), so the bottom goes at -bzDim to put the free
    # surface on z = 0.
    terrain.Construct(
        chrono.ChVector3d(bxDim, byDim, bzDim),
        chrono.ChVector3d(0, 0, -bzDim),
        fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS,
    )

# ---------------------------------------------------------------------------
# rig: tow motor (X) -> free vertical slide (Z) -> spin motor (Y)
# ---------------------------------------------------------------------------
ground = terrain.GetGroundBody()

added_mass = normal_load_N / GRAVITY - body_mass
if added_mass <= 0:
    raise ValueError(f"{BODY} mass {body_mass:.2f} kg already exceeds the requested normal load")
print(f"{BODY} mass = {body_mass:.3f} kg, dead-load mass on carriage = {added_mass:.3f} kg")

carriage_x = chrono.ChBody()
carriage_x.SetPos(chrono.ChVector3d(start_x, y_center, start_z))
carriage_x.SetMass(0.5)
carriage_x.SetInertiaXX(chrono.ChVector3d(1e-3, 1e-3, 1e-3))
carriage_x.EnableCollision(False)
sysMBS.AddBody(carriage_x)

carriage_z = chrono.ChBody()
carriage_z.SetPos(chrono.ChVector3d(start_x, y_center, start_z))
carriage_z.SetMass(added_mass)
carriage_z.SetInertiaXX(chrono.ChVector3d(1e-3, 1e-3, 1e-3))
carriage_z.EnableCollision(False)
sysMBS.AddBody(carriage_z)

rig_frame = chrono.ChVector3d(start_x, y_center, start_z)
q_to_X = chrono.QuatFromAngleY(chrono.CH_PI / 2)    # frame-local Z -> world X
q_to_Y = chrono.QuatFromAngleX(-chrono.CH_PI / 2)   # frame-local Z -> world Y


class RampFunction(chrono.ChFunction):
    """0 until t0, cosine ease to `target` over `ramp`, then constant."""
    def __init__(self, t0, ramp, target):
        super().__init__()
        self.t0, self.ramp, self.target = t0, ramp, target

    def GetVal(self, t):
        if t <= self.t0:
            return 0.0
        if t >= self.t0 + self.ramp:
            return self.target
        frac = (t - self.t0) / self.ramp
        return self.target * 0.5 * (1.0 - math.cos(math.pi * frac))


tow_fn = RampFunction(settle_time, ramp_duration, tow_speed)
spin_fn = RampFunction(settle_time, ramp_duration, ang_speed)

motor_x = chrono.ChLinkMotorLinearSpeed()
motor_x.Initialize(carriage_x, ground, chrono.ChFramed(rig_frame, q_to_X))
motor_x.SetMotorFunction(tow_fn)
sysMBS.AddLink(motor_x)

prismatic_z = chrono.ChLinkLockPrismatic()
prismatic_z.Initialize(carriage_z, carriage_x, chrono.ChFramed(rig_frame, chrono.QUNIT))
sysMBS.AddLink(prismatic_z)

motor_y = chrono.ChLinkMotorRotationSpeed()
motor_y.Initialize(body, carriage_z, chrono.ChFramed(rig_frame, q_to_Y))
motor_y.SetMotorFunction(spin_fn)
sysMBS.AddLink(motor_y)

terrain.Initialize()
print(f"SPH particles: {terrain.GetNumSPHParticles()}, "
      f"boundary BCE: {terrain.GetNumBoundaryBCEMarkers()}")

# ---------------------------------------------------------------------------
# visualization
# ---------------------------------------------------------------------------
vis = None
if HAVE_VSG and not NORENDER:
    visFSI = fsi.ChSphVisualizationVSG(sysFSI)
    visFSI.EnableFluidMarkers(True)
    visFSI.EnableBoundaryMarkers(False)
    visFSI.EnableRigidBodyMarkers(False)
    # Colour must come from a color callback + colormap: SetColorFluidMarkers
    # sets m_sph_color but the GPU render path ignores it, so particles stay
    # Chrono's default blue. There is no greyscale ChColormap; BROWN is the
    # regolith-looking one and is what Chrono's own terramechanics demos use.
    if COLOR_MODE == "velocity":
        visFSI.SetSPHColorCallback(fsi.ParticleVelocityColorCallback(0.0, 0.25),
                                   chrono.ChColormap.Type_FAST)
    else:
        visFSI.SetSPHColorCallback(fsi.ParticleHeightColorCallback(-bzDim, 0.02),
                                   chrono.ChColormap.Type_BROWN)
    vis = vsg3d.ChVisualSystemVSG()
    vis.AttachPlugin(visFSI)
    vis.AttachSystem(sysMBS)
    vis.SetWindowTitle(f"MAGIC 2026 - RASSOR rig ({BODY}) on CRM lunar regolith")
    vis.SetWindowSize(1280, 720)
    vis.AddCamera(chrono.ChVector3d(start_x - 0.15, y_center - 0.95, surface_z + 0.42),
                  chrono.ChVector3d(start_x + 0.2, y_center, surface_z - 0.03))
    vis.SetLightIntensity(0.9)
    vis.Initialize()

# ---------------------------------------------------------------------------
# run
# ---------------------------------------------------------------------------
DUMP = "--dump" in sys.argv
DUMP_FPS = _cli_float("--dumpfps", 15.0)
DUMP_DIR = os.path.join(OUT_DIR, "particles")
if DUMP:
    os.makedirs(DUMP_DIR, exist_ok=True)
    import numpy as _np

travel_limit_x = bxDim / 2 - RADIUS - 0.04
time = 0.0
sim_frame = 0
render_frame = 0
dump_frame = 0
history = []

print(f"running to t_end={tend:.1f}s (stop if x >= {travel_limit_x:.3f} m)...")
while time < tend:
    if vis is not None and time >= render_frame / render_fps:
        if not vis.Run():
            break
        _p = body.GetPos()
        vis.SetCameraPosition(chrono.ChVector3d(_p.x - 0.18, y_center - 0.85,
                                               surface_z + 0.38))
        vis.SetCameraTarget(chrono.ChVector3d(_p.x + 0.04, y_center, surface_z - 0.03))
        vis.Render()
        vis.WriteImageToFile(os.path.join(FRAMES_DIR, f"frame_{render_frame:05d}.png"))
        render_frame += 1

    if DUMP and time >= dump_frame / DUMP_FPS:
        # GetParticlePositions returns a tuple of all markers (numpy-able);
        # GetPositions hands back a raw SWIG vector that cannot be indexed.
        # Fluid markers come first, BCE follow.
        _arr = _np.asarray(sysSPH.GetParticlePositions(),
                           dtype=_np.float32)[:sysSPH.GetNumFluidMarkers()]
        _np.save(os.path.join(DUMP_DIR, f"sph_{dump_frame:05d}.npy"), _arr)
        _q = body.GetRot()
        _p = body.GetPos()
        _np.save(os.path.join(DUMP_DIR, f"body_{dump_frame:05d}.npy"),
                 _np.array([_p.x, _p.y, _p.z, _q.e0, _q.e1, _q.e2, _q.e3], dtype=_np.float64))
        dump_frame += 1

    terrain.Synchronize(time)      # relocates the moving patch when triggered
    terrain.DoStepDynamics(exchange_info)
    time += exchange_info
    sim_frame += 1

    if sim_frame % log_every == 0:
        pos = body.GetPos()
        sinkage = surface_z - (pos.z - RADIUS)
        dbp = -motor_x.GetMotorForce()
        torque = motor_y.GetMotorTorque()
        f_soil = terrain.GetFsiBodyForce(body)
        history.append((time, pos.x, sinkage, dbp, torque, f_soil.z, f_soil.x))
        print(f"t={time:6.3f}s x={pos.x:+.3f} sinkage={sinkage * 1000:7.2f}mm "
              f"DBP={dbp:8.2f}N torque={torque:7.3f}Nm Fz={f_soil.z:8.2f}N "
              f"rtf={terrain.GetRtfCFD():.0f}")

    if not MOVING_PATCH and body.GetPos().x >= travel_limit_x:
        print("reached travel limit, stopping")
        break
    if (surface_z - (body.GetPos().z - RADIUS)) > 0.7 * bzDim:
        print("sinkage exceeded 70% of bed depth -- stopping")
        break

# ---------------------------------------------------------------------------
# outputs
# ---------------------------------------------------------------------------
csv_path = os.path.join(OUT_DIR, "rig_results.csv")
with open(csv_path, "w") as f:
    f.write("time_s,x_m,sinkage_m,drawbar_pull_N,torque_Nm,normal_reaction_N,traction_N\n")
    for row in history:
        f.write(",".join(f"{v:.6f}" for v in row) + "\n")
print(f"wrote {csv_path} ({len(history)} rows)")

try:
    import matplotlib.pyplot as plt
    t = [r[0] for r in history]
    fig, ax = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    ax[0].plot(t, [r[2] * 1000 for r in history], lw=2); ax[0].set_ylabel("sinkage (mm)"); ax[0].grid(True)
    ax[1].plot(t, [r[3] for r in history], lw=2, color="tab:orange"); ax[1].set_ylabel("drawbar pull (N)"); ax[1].grid(True)
    ax[2].plot(t, [r[4] for r in history], lw=2, color="tab:green"); ax[2].set_ylabel("torque (N m)")
    ax[2].set_xlabel("time (s)"); ax[2].grid(True)
    fig.suptitle(f"RASSOR rig ({BODY}) on CRM lunar regolith -- "
                 f"{normal_load_N / LB_TO_N:.0f} lbf, {slip_target * 100:.0f}% slip, "
                 f"spacing {spacing * 1000:.0f} mm")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "rig_summary.png"), dpi=120)
    print("saved rig_summary.png")
except Exception as e:
    print(f"(plot skipped: {e})")

if render_frame > 0:
    try:
        import imageio.v2 as imageio
        video_path = os.path.join(OUT_DIR, f"rassor_{BODY}.mp4")
        with imageio.get_writer(video_path, fps=render_fps, codec="libx264", quality=8) as w:
            for i in range(render_frame):
                p = os.path.join(FRAMES_DIR, f"frame_{i:05d}.png")
                if os.path.exists(p):
                    w.append_data(imageio.imread(p))
        print(f"wrote {video_path} ({render_frame} frames @ {render_fps} fps)")
    except Exception as e:
        print(f"(video assembly skipped: {e})")

print("done")
