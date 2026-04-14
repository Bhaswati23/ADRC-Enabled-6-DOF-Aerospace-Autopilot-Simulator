% =========================================================================
% ESO_OBSERVER.M
% Extended State Observer (ESO) — Third-Order Linear ESO
%
% PURPOSE:
%   The ESO extends the plant state to include the "total disturbance"
%   as an additional state variable. This allows real-time estimation and
%   subsequent cancellation of disturbances.
%
% PLANT MODEL (pitch axis):
%   x1 = theta     (pitch angle)
%   x2 = q = x1_dot (pitch rate)
%   x2_dot = f(x,t) + b0 * u_e      ← f is the "total disturbance"
%
%   Extended state:  x3 = f(x,t)  (disturbance treated as extra state)
%   Assumed model:   x3_dot ≈ 0   (disturbance changes slowly vs. observer)
%
% LINEAR ESO EQUATIONS (continuous):
%   z1_dot = z2 - beta1*(z1 - theta)
%   z2_dot = z3 - beta2*(z1 - theta) + b0*u_e
%   z3_dot =    - beta3*(z1 - theta)
%
%   where e_obs = z1 - theta = observation error
%
%   The characteristic polynomial of the ESO is:
%     s³ + beta1*s² + beta2*s + beta3 = (s + omega_o)³
%
%   Expanding:
%     beta1 = 3*omega_o
%     beta2 = 3*omega_o²
%     beta3 = omega_o³
%
%   All ESO poles are placed at -omega_o (stable for omega_o > 0).
%
% DISCRETIZATION:
%   First-order Euler integration:
%   z(k+1) = z(k) + dt * f(z(k), u(k))
%
%   This is sufficient for dt << 1/omega_o. Verify: dt*omega_o < 0.5
%   With dt=0.005 and omega_o=50: dt*omega_o = 0.25 ✓
%
% INPUTS:
%   theta     - Measured pitch angle (rad)  — the "output" y
%   q         - Measured pitch rate (rad/s) — used only as cross-check
%   eso_state - Previous ESO state [z1; z2; z3]
%   adrc      - Parameter struct containing beta1, beta2, beta3, b0
%   dt        - Time step (s)
%
% NOTE:
%   u_e (control input) is not passed because we use the "output-only"
%   form: the ESO observer error injection compensates for missing u_e.
%   For higher accuracy, pass u_e and include b0*u_e in z2_dot.
%
% OUTPUTS:
%   de_est    - (unused placeholder, for API consistency)
%   eso_state - Updated [z1; z2; z3] (theta_est, q_est, disturbance_est)
% =========================================================================

function [de_est, eso_state] = eso_observer(theta, q, eso_state, adrc, dt)

    %% ---- Unpack ESO gains ----------------------------------------------
    beta1 = adrc.beta1;    % ESO gain 1 = 3*omega_o
    beta2 = adrc.beta2;    % ESO gain 2 = 3*omega_o²
    beta3 = adrc.beta3;    % ESO gain 3 = omega_o³
    b0    = adrc.b0;       % Control gain estimate

    %% ---- Current ESO states -------------------------------------------
    z1 = eso_state(1);    % Estimated pitch angle  (should track theta)
    z2 = eso_state(2);    % Estimated pitch rate   (should track q)
    z3 = eso_state(3);    % Estimated total disturbance (rad/s²)

    %% ---- Observation error --------------------------------------------
    % e_obs is the innovation: difference between measured output and
    % observer prediction. This drives all three ESO equations.
    e_obs = z1 - theta;   % Observation error (positive → observer is high)

    %% ---- ESO differential equations (continuous form) -----------------
    %
    % z1_dot: track theta using z2 (predicted derivative), correct with e_obs
    z1_dot = z2 - beta1 * e_obs;

    % z2_dot: track q using z3 (disturbance), correct with e_obs
    %   The b0*u_e term is omitted here (output-injection form).
    %   This is acceptable when the control is bounded and beta2 is large.
    z2_dot = z3 - beta2 * e_obs;

    % z3_dot: disturbance is modeled as constant (random walk model).
    %   beta3 drives z3 toward the true disturbance.
    z3_dot = -beta3 * e_obs;

    %% ---- Euler integration (discrete update) --------------------------
    z1_new = z1 + dt * z1_dot;
    z2_new = z2 + dt * z2_dot;
    z3_new = z3 + dt * z3_dot;

    %% ---- Bound disturbance estimate (prevents wind-up) ----------------
    % Physical limit: max disturbance acceleration ≈ 500 rad/s²
    z3_new = max(-500, min(500, z3_new));

    %% ---- Output updated state -----------------------------------------
    eso_state = [z1_new; z2_new; z3_new];
    de_est    = 0;    % Placeholder (not used externally)
end
