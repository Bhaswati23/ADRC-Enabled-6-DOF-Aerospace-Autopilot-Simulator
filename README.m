% =========================================================================
% README.m  —  6-DOF Missile Guidance & Control System
% Active Disturbance Rejection Control + Proportional Navigation
% =========================================================================
%
% Quick start:
%   1. cd to this directory in MATLAB
%   2. Run:  main_simulation
%   3. Run:  validate_model   (optional verification)
%   4. Run:  init_simulink    (for Simulink model setup)
%
% =========================================================================
%
%  FILE STRUCTURE
% ─────────────────────────────────────────────────────────────────────────
%  main_simulation.m    ← Entry point. Runs full 6-DOF simulation loop.
%  sixdof_dynamics.m    ← Newton-Euler EOM for 12 states.
%  aerodynamic_forces.m ← Lift, Drag, Side-force model (body frame).
%  aerodynamic_moments.m← Pitch, Roll, Yaw moment model.
%  adrc_controller.m    ← ADRC pitch controller (calls ESO internally).
%  eso_observer.m       ← Third-order Extended State Observer.
%  guidance_law.m       ← Proportional Navigation guidance law.
%  disturbance_model.m  ← Wind/gust/turbulence disturbance.
%  plot_results.m       ← All visualization (8 figures).
%  validate_model.m     ← 7-test physical consistency validation suite.
%  init_simulink.m      ← Simulink workspace init + build guide.
%
% ─────────────────────────────────────────────────────────────────────────
%
%  MATHEMATICAL OVERVIEW
% ─────────────────────────────────────────────────────────────────────────
%
%  COORDINATE FRAMES
%  ─────────────────
%  Body Frame (B):     Origin at missile CG
%                      x_B: forward (along roll axis)
%                      y_B: right (starboard)
%                      z_B: down (right-hand system)
%
%  Inertial Frame (I): NED (North-East-Down), Earth-fixed
%                      x_I: North
%                      y_I: East
%                      z_I: Down (altitude = -z_I)
%
%  State Vector (12):
%    [u, v, w]         Body velocities (m/s)
%    [p, q, r]         Body angular rates (rad/s)
%    [φ, θ, ψ]         Euler angles: roll, pitch, yaw (rad)
%    [x, y, z]         NED position (m)
%
%  NEWTON-EULER EQUATIONS OF MOTION
%  ──────────────────────────────────
%  Translational (body frame):
%    m·(u̇ + q·w - r·v) = Fx  (total force: aero + thrust + gravity)
%    m·(v̇ + r·u - p·w) = Fy
%    m·(ẇ + p·v - q·u) = Fz
%
%  Rotational (Euler's equations with Ixz cross-coupling):
%    [Ixx  -Ixz] · [ṗ]   [Mx - (Izz-Iyy)·q·r + Ixz·p·q]
%    [-Ixz  Izz]   [ṙ] = [Mz - (Iyy-Ixx)·p·q - Ixz·q·r]
%    Iyy·q̇ = My - (Ixx-Izz)·p·r - Ixz·(p²-r²)
%
%  Euler Kinematics (3-2-1 sequence):
%    φ̇ = p + (q·sin φ + r·cos φ)·tan θ
%    θ̇ = q·cos φ - r·sin φ
%    ψ̇ = (q·sin φ + r·cos φ) / cos θ
%
%  Position (body → inertial via DCM R_BI = transpose(R_IB)):
%    [ẋ, ẏ, ż]ᵀ = R_BI · [u, v, w]ᵀ
%
%  AERODYNAMIC MODEL
%  ─────────────────
%    CD = CD0 + CDa·α²         (drag polar, parabolic)
%    CL = CLa · α              (linear lift curve)
%    CY = CYb · β              (linear side force)
%    Cm = Cma·α + Cmq·(q·d/2V) + Cmde·δe   (pitch moment)
%    Cl = Clp·(p·d/2V)         (roll damping)
%    Cn = Cnr·(r·d/2V)         (yaw damping)
%
%  ADRC CONTROL SYSTEM
%  ────────────────────
%  Plant model (pitch):
%    θ̇ = q
%    q̇ = f(·) + b₀·δe    where f(·) = total disturbance
%
%  Extended State Observer (3rd order):
%    ż₁ = z₂ - β₁·(z₁ - θ)
%    ż₂ = z₃ - β₂·(z₁ - θ) + b₀·δe
%    ż₃ =    - β₃·(z₁ - θ)
%
%  Observer poles: (s + ωo)³ = 0
%    β₁ = 3ωo,  β₂ = 3ωo²,  β₃ = ωo³
%
%  Control Law:
%    u₀ = k₁·(θref - z₁) - k₂·z₂     [PD on estimated states]
%    δe = (u₀ - z₃) / b₀               [disturbance cancellation]
%
%  Controller poles: (s + ωc)² = 0
%    k₁ = ωc²,  k₂ = 2ωc
%
%  PROPORTIONAL NAVIGATION
%  ────────────────────────
%    a_cmd = N · V_c · λ̇
%
%    λ   = LOS elevation angle = atan2(-Δz, √(Δx²+Δy²))
%    λ̇   = dλ/dt  (numerically differentiated, filtered)
%    V_c = -d(R)/dt = closing speed (m/s)
%    N   = 4  (navigation ratio)
%
%  Pitch reference:
%    θref = θ + (a_cmd/V)·dt
%
% ─────────────────────────────────────────────────────────────────────────
%
%  DEFAULT PARAMETERS SUMMARY
% ─────────────────────────────────────────────────────────────────────────
%
%  Missile:
%    Mass = 120 kg,  Iyy = 80 kg·m²
%    Sref = 0.02 m², Lref = 0.16 m
%    Thrust = 5000 N for 8 s
%
%  ADRC:
%    ωc = 8 rad/s,  ωo = 50 rad/s
%    k1 = 64,       k2 = 16
%    β1 = 150,      β2 = 7500,    β3 = 125000
%    b0 = 1/80 = 0.0125
%    δe_max = ±25°
%
%  Guidance:
%    N = 4 (navigation ratio)
%
%  Scenario:
%    Missile launch: [0, 0, -500] m NED at 300 m/s, θ0 = +5°
%    Target: [8000, 500, -500] m NED at -150 m/s (head-on)
%
%  Disturbance:
%    Steady wind: -10 m/s North, +5 m/s East
%    Step gust:   +8 m/s downward from t=5 to t=8 s (1-cosine profile)
%    Turbulence:  σ = 1.5 m/s (Dryden approximation)
%
% ─────────────────────────────────────────────────────────────────────────
%
%  VALIDATION CHECKLIST (run validate_model.m)
% ─────────────────────────────────────────────────────────────────────────
%  ✓ DCM gravity transform at θ = -90° → [g, 0, 0] in body frame
%  ✓ Zero-incidence aerodynamics → pure drag, no lift/side force
%  ✓ Pure pitch kinematics → θ̇ = q exactly
%  ✓ ESO convergence to injected constant disturbance (< 20% error)
%  ✓ ADRC step tracking on 2nd-order plant (< 2% SS error)
%  ✓ ISA atmosphere density at 0 m and 10 km altitude
%  ✓ Elevator sign convention → positive δe → positive My (nose-up)
%
% ─────────────────────────────────────────────────────────────────────────
%
%  KNOWN SIMPLIFICATIONS (acceptable for internship project)
% ─────────────────────────────────────────────────────────────────────────
%  • Aerodynamic coefficients are Mach-independent constants
%    (real missiles use Mach × α lookup tables from wind-tunnel data)
%  • No actuator dynamics in MATLAB sim (only Simulink has servo model)
%  • Euler angle singularity at θ = ±90° (use quaternions for full range)
%  • Guidance is 2D (vertical plane) — extend with azimuth channel for 3D
%  • Turbulence is white-noise-based (proper Dryden filter for MIL-spec)
%  • No warhead/fusing model (range-only termination condition)
%
% ─────────────────────────────────────────────────────────────────────────
%
%  EXTENSION IDEAS
% ─────────────────────────────────────────────────────────────────────────
%  • Add yaw ADRC channel for full 3D guidance
%  • Implement augmented PN (APN) with target acceleration feedforward
%  • Replace constant aero coefficients with Mach × alpha lookup tables
%  • Add seeker model (gimballed IR or radar with noise + gimbal limits)
%  • Model autopilot roll stabilization
%  • Add Monte Carlo analysis over wind and target maneuver uncertainty
%
% =========================================================================

fprintf('\n6-DOF Missile Guidance System — File Structure:\n');
fprintf('  main_simulation.m     → Entry point\n');
fprintf('  sixdof_dynamics.m     → Newton-Euler EOM\n');
fprintf('  aerodynamic_forces.m  → Lift/Drag/Side force\n');
fprintf('  aerodynamic_moments.m → Pitch/Roll/Yaw moments\n');
fprintf('  adrc_controller.m     → ADRC pitch controller\n');
fprintf('  eso_observer.m        → Extended State Observer\n');
fprintf('  guidance_law.m        → Proportional Navigation\n');
fprintf('  disturbance_model.m   → Wind/gust model\n');
fprintf('  plot_results.m        → 8-panel visualization\n');
fprintf('  validate_model.m      → 7-test validation suite\n');
fprintf('  init_simulink.m       → Simulink setup + build guide\n');
fprintf('\nRun: >> main_simulation\n\n');
