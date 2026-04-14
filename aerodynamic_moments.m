% =========================================================================
% AERODYNAMIC_MOMENTS.M
% Computes aerodynamic moments about the missile CG in the body frame.
%
% MOMENT CONVENTIONS (right-hand rule, body frame):
%   Mx — Roll  moment about x_body (positive: right wing down)
%   My — Pitch moment about y_body (positive: nose up)
%   Mz — Yaw   moment about z_body (positive: nose right)
%
% MOMENT MODEL:
%   Non-dimensional moment coefficients (Cm, Cl, Cn) are multiplied by
%   the moment reference arm:
%     My = qbar * S * Lref * Cm_total
%     Mx = qbar * S * Lref * Cl_total
%     Mz = qbar * S * Lref * Cn_total
%
% PITCH MOMENT (Cm):
%   Cm = Cma * alpha                  (static stability, Cma < 0 = stable)
%      + Cmq * (q * Lref / (2*V))     (pitch damping)
%      + Cmde * delta_e               (elevator/canard control)
%
%   The pitch damping term Cmq*(q*Lref/(2V)) models the restoring moment
%   from tail surfaces as the missile pitches. Cmq < 0 = damping.
%
% ROLL MOMENT (Cl):
%   Cl = Clp * (p * Lref / (2*V))     (roll damping)
%
%   Roll is lightly modeled; most missiles rely on body symmetry.
%
% YAW MOMENT (Cn):
%   Cn = Cnr * (r * Lref / (2*V))     (yaw damping)
%
% INPUTS:
%   qbar     - Dynamic pressure (Pa)
%   alpha    - Angle of attack (rad)
%   beta     - Sideslip angle (rad)
%   p,q,r    - Body angular rates (rad/s)
%   V        - Total airspeed (m/s)
%   de       - Elevator deflection (rad), positive = trailing edge down
%   missile  - Missile parameter struct
%
% OUTPUTS:
%   Mx_aero, My_aero, Mz_aero — Aerodynamic moments in body frame (N·m)
% =========================================================================

function [Mx_aero, My_aero, Mz_aero] = aerodynamic_moments(qbar, alpha, beta, p, q, r, V, de, missile)

    S    = missile.Sref;
    Lref = missile.Lref;
    Cma  = missile.aero.Cma;
    Cmq  = missile.aero.Cmq;
    Cmde = missile.aero.Cmde;
    Clp  = missile.aero.Clp;
    Cnr  = missile.aero.Cnr;

    %% ---- Non-dimensionalize angular rates --------------------------------
    % These are the standard aerodynamic body-rate non-dimensional parameters
    % used in stability and control analysis.
    V_ref = max(V, 1.0);    % Avoid division by zero at launch
    p_hat = p * Lref / (2 * V_ref);    % Non-dimensional roll  rate
    q_hat = q * Lref / (2 * V_ref);    % Non-dimensional pitch rate
    r_hat = r * Lref / (2 * V_ref);    % Non-dimensional yaw   rate

    %% ---- PITCH MOMENT COEFFICIENT (Cm) -----------------------------------
    % Static stability: Cma*alpha generates a restoring nose-down moment
    %   when Cma < 0 (statically stable configuration)
    Cm_static  = Cma * alpha;

    % Pitch damping: opposes pitch rate (Cmq < 0)
    Cm_damp    = Cmq * q_hat;

    % Control effectiveness: elevator/canard deflection
    %   Positive delta_e (trailing edge down) → nose-up pitch moment
    Cm_control = Cmde * de;

    Cm_total = Cm_static + Cm_damp + Cm_control;

    %% ---- ROLL MOMENT COEFFICIENT (Cl) ------------------------------------
    % Roll damping: opposes roll rate (Clp < 0)
    % Add small sideslip-to-roll coupling (Clb) if needed
    Cl_total = Clp * p_hat;

    %% ---- YAW MOMENT COEFFICIENT (Cn) ------------------------------------
    % Yaw damping: opposes yaw rate (Cnr < 0)
    Cn_total = Cnr * r_hat;

    %% ---- Dimensional moments (N·m) ---------------------------------------
    % Moment = qbar * S * Lref * coefficient
    My_aero = qbar * S * Lref * Cm_total;   % Pitch (N·m)
    Mx_aero = qbar * S * Lref * Cl_total;   % Roll  (N·m)
    Mz_aero = qbar * S * Lref * Cn_total;   % Yaw   (N·m)

    %% ---- Sanity bounds ---------------------------------------------------
    My_aero = clamp(My_aero, -50e3, 50e3);
    Mx_aero = clamp(Mx_aero, -10e3, 10e3);
    Mz_aero = clamp(Mz_aero, -10e3, 10e3);
end

function y = clamp(x, lo, hi)
    y = max(lo, min(hi, x));
end
