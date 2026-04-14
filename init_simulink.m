% =========================================================================
% INIT_SIMULINK.M
% Simulink Model Initialization and Parameter Workspace Setup
%
% PURPOSE:
%   This script initializes all parameters into the MATLAB workspace
%   before running the Simulink model. Run this before opening the
%   Simulink model (missile_guidance_simulink.slx).
%
% SIMULINK MODEL ARCHITECTURE (missile_guidance_simulink.slx):
% =========================================================================
%
%  SIGNAL FLOW DIAGRAM:
%
%  [Target Model] ──────pos_t, vel_t──────────────────────────────┐
%                                                                  │
%  [Missile State] ──pos_m, vel_body, θ, ψ──► [Guidance: PN] ──theta_ref──►┐
%                                                                            │
%  theta ──────────────────────────────────────────────────────► [ADRC]◄────┘
%  q ──────────────────────────────────────────────────────────► [ESO] ──z3──►┐
%                                                                              │
%                                                        ┌──────────────────◄─┘
%                                                        │
%  [ADRC Output: δe] ──► [Actuator: 1st-order lag] ──δe_actual──►┐
%                                                                  │
%  [Disturbance Model] ──wind────────────────────────────────────►┤
%                                                                  │
%  [Aerodynamic Forces  ] ──Fx,Fy,Fz──────────────────────────►  │
%  [Aerodynamic Moments ] ──Mx,My,Mz──────────────────────────►  ▼
%                                                        [6-DOF Dynamics]
%                                                                  │
%                                                                  ▼
%                                               [State: u,v,w,p,q,r,φ,θ,ψ,x,y,z]
%                                                                  │
%                                                    ┌─────────────┘ (feedback)
%                                                    │
%                                         [Coord Transform]
%                                                    │
%                                              [Scope/Logger]
%
% =========================================================================
%
% SIMULINK BLOCK DESCRIPTIONS:
% =========================================================================
%
% 1. TARGET MODEL SUBSYSTEM
%    ─────────────────────────────────────────────────────────────────────
%    Blocks:
%      • Integrator (3 states) — integrates target velocity to position
%      • Constant block — target initial velocity [-150; 0; 0] m/s NED
%      • IC block — target initial position [8000; 500; -500] m NED
%    Outputs: pos_t (3×1), vel_t (3×1)
%
% 2. GUIDANCE SUBSYSTEM (Proportional Navigation)
%    ─────────────────────────────────────────────────────────────────────
%    Blocks:
%      • Math Operations: Subtract (pos_t - pos_m), norm
%      • Trigonometric Functions: atan2, asin for LOS angle computation
%      • Derivative block (with low-pass filter τ=0.25s) — LOS rate
%      • Gain: Navigation constant N=4
%      • Math: V_c * lambda_dot → a_cmd
%      • Divide, Saturation: pitch angle reference conversion
%      • Memory block: lambda_prev for discrete differentiation
%    Inputs:  pos_m, vel_body, theta, psi, pos_t, vel_t
%    Outputs: theta_ref
%
% 3. ADRC CONTROLLER SUBSYSTEM
%    ─────────────────────────────────────────────────────────────────────
%    Sub-blocks:
%      a) ESO (Extended State Observer):
%         • Sum block: e_obs = z1 - theta (observation error)
%         • Three Gain blocks: beta1, beta2, beta3
%         • Three Integrator blocks: z1, z2, z3
%         • Sum blocks for ESO differential equations
%      b) PD Control Law:
%         • Sum: e1 = theta_ref - z1
%         • Gain k1: proportional term
%         • Gain k2: derivative term (acting on z2)
%         • Sum: u0 = k1*e1 - k2*z2
%      c) Disturbance Cancellation:
%         • Sum: u0 - z3
%         • Gain: 1/b0
%         • Saturation: ±de_max
%    Inputs:  theta, q, theta_ref
%    Outputs: de (elevator command), z3 (disturbance estimate)
%
% 4. ACTUATOR SUBSYSTEM
%    ─────────────────────────────────────────────────────────────────────
%    Models servo actuator dynamics (elevator servo response):
%      Transfer function: G_act(s) = 1 / (tau_act*s + 1)
%      tau_act = 0.02s (50 Hz bandwidth — fast servo)
%    Blocks:
%      • Transfer Function: 1/(0.02s+1)
%      • Rate Limiter: ±2.5 rad/s deflection rate limit
%      • Saturation: ±de_max
%    Inputs:  de_cmd
%    Outputs: de_actual
%
% 5. AERODYNAMICS SUBSYSTEM
%    ─────────────────────────────────────────────────────────────────────
%    Sub-subsystems:
%      a) Aerodynamic Angles:
%         • atan2(w, u) → alpha
%         • asin(v/V) → beta
%         • sqrt(u²+v²+w²) → V
%      b) Aerodynamic Forces (aerodynamic_forces.m as S-Function):
%         • Inputs: qbar, alpha, beta
%         • Outputs: Fx, Fy, Fz
%      c) Aerodynamic Moments (aerodynamic_moments.m as S-Function):
%         • Inputs: qbar, alpha, beta, p, q, r, V, de
%         • Outputs: Mx, My, Mz
%    Use MATLAB Function blocks or Interpreted MATLAB Function blocks.
%
% 6. 6-DOF DYNAMICS SUBSYSTEM
%    ─────────────────────────────────────────────────────────────────────
%    Implement sixdof_dynamics.m as a MATLAB Function block:
%      • 12 state integrators (one per state variable)
%      • State vector output fed back to all subsystems
%    Recommended: Use the "6DOF (Euler Angles)" block from
%    Aerospace Blockset if available. Otherwise:
%      • MATLAB Function block calling sixdof_dynamics()
%      • Integrator block with IC from X0
%    Inputs:  All forces [Fx;Fy;Fz], moments [Mx;My;Mz], wind
%    Outputs: X (12-state vector)
%
% 7. COORDINATE TRANSFORMATION SUBSYSTEM
%    ─────────────────────────────────────────────────────────────────────
%    Converts body-frame velocities to inertial frame:
%      • Direction Cosine Matrix (DCM) computation
%      • Matrix multiply: v_inertial = R_BI * v_body
%    Inputs:  phi, theta, psi, u, v, w
%    Outputs: x_dot, y_dot, z_dot (inertial velocity components)
%
% =========================================================================

fprintf('Initializing Simulink workspace parameters...\n\n');

%% ---- Missile parameters (loaded into base workspace) -------------------
sim_params.dt     = 0.005;
sim_params.t_end  = 25.0;

% Missile physical properties
missile.mass  = 120;
missile.Ixx   = 0.5;
missile.Iyy   = 80;
missile.Izz   = 80;
missile.Ixz   = 0.0;
missile.Sref  = 0.02;
missile.Lref  = 0.16;
missile.thrust = 5000;
missile.t_burn = 8.0;

% Aerodynamic coefficients
missile.aero.CD0  = 0.30;
missile.aero.CDa  = 1.80;
missile.aero.CLa  = 12.0;
missile.aero.CYb  = -10.0;
missile.aero.Cmq  = -20.0;
missile.aero.Cmde =  2.5;
missile.aero.Cma  = -2.0;
missile.aero.Clp  = -0.5;
missile.aero.Cnr  = -0.5;

% Assign to base workspace for Simulink access
assignin('base', 'missile', missile);

%% ---- Initial conditions ------------------------------------------------
X0 = [300; 0; 0; 0; 0; 0; 0; deg2rad(5); 0; 0; 0; -500];
assignin('base', 'X0', X0);

%% ---- ADRC parameters ---------------------------------------------------
adrc.b0      = 1/missile.Iyy;
adrc.omega_c = 8.0;
adrc.omega_o = 50.0;
adrc.k1      = adrc.omega_c^2;
adrc.k2      = 2*adrc.omega_c;
adrc.beta1   = 3*adrc.omega_o;
adrc.beta2   = 3*adrc.omega_o^2;
adrc.beta3   = adrc.omega_o^3;
adrc.de_max  = deg2rad(25);
assignin('base', 'adrc', adrc);

% Individual workspace variables for Simulink Gain blocks
assignin('base', 'k1',    adrc.k1);
assignin('base', 'k2',    adrc.k2);
assignin('base', 'b0',    adrc.b0);
assignin('base', 'beta1', adrc.beta1);
assignin('base', 'beta2', adrc.beta2);
assignin('base', 'beta3', adrc.beta3);
assignin('base', 'de_max', adrc.de_max);

%% ---- Guidance parameters -----------------------------------------------
guidance.N = 4.0;
assignin('base', 'Nav_N', guidance.N);

%% ---- Target initial conditions -----------------------------------------
target_pos0 = [8000; 500; -500];
target_vel   = [-150; 0; 0];
assignin('base', 'target_pos0', target_pos0);
assignin('base', 'target_vel',  target_vel);

%% ---- Actuator dynamics -------------------------------------------------
tau_act = 0.02;   % Servo time constant (s)
assignin('base', 'tau_act', tau_act);

%% ---- Gravity -----------------------------------------------------------
g = 9.81;
assignin('base', 'g', g);

%% ---- Summary -----------------------------------------------------------
fprintf('=== Simulink Parameter Summary ===\n');
fprintf('Missile mass:         %.0f kg\n',   missile.mass);
fprintf('Pitch inertia (Iyy):  %.0f kg·m²\n', missile.Iyy);
fprintf('ADRC omega_c:         %.1f rad/s\n', adrc.omega_c);
fprintf('ADRC omega_o:         %.1f rad/s\n', adrc.omega_o);
fprintf('ESO beta1/beta2/beta3: %.0f / %.0f / %.0f\n', adrc.beta1, adrc.beta2, adrc.beta3);
fprintf('PD gains k1/k2:       %.1f / %.1f\n', adrc.k1, adrc.k2);
fprintf('Nav constant N:       %.1f\n', guidance.N);
fprintf('\nAll parameters loaded into base workspace.\n');
fprintf('Open missile_guidance_simulink.slx to run the Simulink model.\n\n');

%% ---- Simulink Construction Guide ----------------------------------------
fprintf('=== SIMULINK CONSTRUCTION CHECKLIST ===\n');
fprintf('1. New Model → Solver: Fixed-step, RK4, dt=0.005s\n');
fprintf('2. Add Subsystems: Guidance | ADRC+ESO | Actuator | Aero | 6DOF | Target\n');
fprintf('3. ESO: 3 Integrators with ICs [X0(8); X0(5); 0]\n');
fprintf('4. 6DOF: MATLAB Function block calling sixdof_dynamics()\n');
fprintf('5. Signal routing: All states feed back from 6DOF output\n');
fprintf('6. Scopes: theta/theta_cmd | de | z3 | miss distance | 3D pos\n');
fprintf('7. Run init_simulink.m before simulating\n');
fprintf('==========================================\n');
