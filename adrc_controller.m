% =========================================================================
% ADRC_CONTROLLER.M
% Active Disturbance Rejection Controller (ADRC) — Pitch Axis
%
% PHILOSOPHY:
%   ADRC treats all unmodeled dynamics, aerodynamic nonlinearities,
%   parameter uncertainty, and external disturbances as a single
%   "total disturbance" f(t). The Extended State Observer (ESO) estimates
%   this disturbance in real time. The control law then cancels it,
%   reducing the plant to a pure double integrator:
%
%     Plant model (pitch):
%       theta_dot  = q
%       q_dot      = f(theta,q,t) + b0*u_e
%
%     where:
%       f(...)  = true total disturbance (unknown)
%       b0      = 1/Iyy (known control gain estimate)
%       u_e     = elevator command (control input)
%
% STRUCTURE:
%   1. ESO estimates [theta, q, f] as [z1, z2, z3]
%   2. PD control law on error e = theta_ref - z1:
%        u0 = k1*(theta_ref - z1) + k2*(0 - z2)
%           [proportional]         [derivative with zero setpoint for q_dot]
%   3. Disturbance compensation:
%        u_e = (u0 - z3) / b0
%
% PARAMETERS (bandwidth parameterization):
%   omega_c — desired closed-loop bandwidth (rad/s)
%   omega_o — ESO bandwidth (rad/s), typically 3-10× omega_c
%   k1 = omega_c²   (PD proportional gain)
%   k2 = 2*omega_c  (PD derivative gain)
%   beta1 = 3*omega_o     (ESO gain 1)
%   beta2 = 3*omega_o²    (ESO gain 2)
%   beta3 = omega_o³      (ESO gain 3)
%   b0    = 1/Iyy          (control gain estimate)
%
% INPUTS:
%   theta     - Current pitch angle (rad)
%   q         - Current pitch rate (rad/s)
%   theta_ref - Commanded pitch angle from guidance (rad)
%   eso_state - ESO state vector [z1; z2; z3]
%   adrc      - Controller parameter struct
%   dt        - Time step (s)
%
% OUTPUTS:
%   de        - Elevator deflection command (rad)
%   eso_state - Updated ESO state vector [z1; z2; z3]
%
% REFERENCE:
%   Han, J. (2009). "From PID to Active Disturbance Rejection Control."
%   IEEE Transactions on Industrial Electronics, 56(3), 900-906.
% =========================================================================

function [de, eso_state] = adrc_controller(theta, q, theta_ref, eso_state, adrc, dt)

    %% ---- Unpack ADRC parameters ----------------------------------------
    k1    = adrc.k1;       % Proportional gain = omega_c²
    k2    = adrc.k2;       % Derivative gain   = 2*omega_c
    b0    = adrc.b0;       % Control gain estimate (1/Iyy)
    de_max = adrc.de_max;  % Elevator deflection limit (rad)

    %% ---- Step 1: Run the ESO (one step ahead) --------------------------
    % The ESO is called separately but we embed a local copy here
    % so ADRC is a self-contained function.
    [de_prev_est, eso_state] = eso_observer(theta, q, eso_state, adrc, dt);

    %% ---- Extract ESO estimates -----------------------------------------
    z1 = eso_state(1);    % Estimated pitch angle  (rad)
    z2 = eso_state(2);    % Estimated pitch rate   (rad/s)
    z3 = eso_state(3);    % Estimated total disturbance (rad/s²)

    %% ---- Step 2: PD Control Law on pitch angle error -------------------
    % Tracking error
    e1 = theta_ref - z1;   % Pitch angle error  (rad)

    % Pitch rate setpoint: we want pitch rate to drive angle to reference.
    % For a simple second-order system: q_ref = k1*e1/k2 = omega_c/2 * e1
    % But for clean ADRC formulation, we use full PD on z1 and z2:
    %   u0 = k1*e1 + k2*(q_ref_dot - z2)
    % With q_ref_dot = 0 (constant reference assumption):
    u0 = k1 * e1 - k2 * z2;

    %% ---- Step 3: Disturbance rejection ---------------------------------
    % Cancel estimated disturbance, scale by b0 to get elevator command.
    % This converts the nonlinear disturbed plant back to a double integrator.
    %
    %   u_e = (u0 - z3) / b0
    %
    % Intuition: if z3 correctly estimates disturbance f, then:
    %   q_dot = f + b0*u_e ≈ f + b0*(u0-f)/b0 = u0
    % So effective dynamics become: q_dot = u0 (pure integrator!)
    de = (u0 - z3) / b0;

    %% ---- Saturate elevator command -------------------------------------
    de = max(-de_max, min(de_max, de));
end
