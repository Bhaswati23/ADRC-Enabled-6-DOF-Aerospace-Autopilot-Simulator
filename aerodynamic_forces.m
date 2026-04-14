% =========================================================================
% AERODYNAMIC_FORCES.M
% Computes aerodynamic forces on the missile in the body frame.
%
% AERODYNAMIC ANGLES:
%   alpha (Angle of Attack): angle between velocity vector projected onto
%         x-z body plane and the x-body axis.
%         alpha = atan2(w, u)
%
%   beta (Sideslip Angle): angle between velocity vector and x-z body plane.
%         beta = asin(v / V)
%
% FORCE MODEL (body frame):
%   The aerodynamic force vector in the wind frame is:
%     Drag (D)  — opposes velocity
%     Lift (L)  — perpendicular to velocity, in x-z body plane
%     Side (Y)  — perpendicular to x-z body plane
%
%   Transformed to body frame:
%     Fx_aero = -D*cos(alpha)*cos(beta) + L*sin(alpha) - Y*cos(alpha)*sin(beta)
%     Fy_aero = -D*sin(beta)             + Y*cos(beta)
%     Fz_aero = -D*sin(alpha)*cos(beta) - L*cos(alpha) - Y*sin(alpha)*sin(beta)
%
% COEFFICIENTS USED:
%   CD = CD0 + CDa*alpha²      (drag polar — parabolic model)
%   CL = CLa * alpha           (linear lift-curve slope)
%   CY = CYb * beta            (linear side-force slope)
%
% NOTE: All forces are in the body frame (Fx forward, Fy right, Fz down).
%       Lift is defined as positive upward (−z_body direction).
%
% INPUTS:
%   qbar    - Dynamic pressure (Pa)
%   alpha   - Angle of attack (rad)
%   beta    - Sideslip angle (rad)
%   missile - Missile parameter struct (Sref, aero coefficients)
%
% OUTPUTS:
%   Fx_aero, Fy_aero, Fz_aero — Aerodynamic forces in body frame (N)
% =========================================================================

function [Fx_aero, Fy_aero, Fz_aero] = aerodynamic_forces(qbar, alpha, beta, missile)

    S   = missile.Sref;
    CD0 = missile.aero.CD0;
    CDa = missile.aero.CDa;
    CLa = missile.aero.CLa;
    CYb = missile.aero.CYb;

    %% ---- Non-dimensional aerodynamic coefficients ----------------------
    % Drag: increases quadratically with angle of attack (induced drag)
    CD = CD0 + CDa * alpha^2;

    % Lift: linear in alpha (valid for small-to-moderate angles of attack)
    % Sign convention: CL positive = force in -z_body (upward) direction
    CL = CLa * alpha;

    % Side force: linear in sideslip
    % CY negative = force in -y_body for positive beta (restoring)
    CY = CYb * beta;

    %% ---- Dimensional forces (wind frame) -------------------------------
    D = qbar * S * CD;    % Drag force (N) — always positive, opposes motion
    L = qbar * S * CL;    % Lift force (N) — positive = upward (−z_body)
    Y = qbar * S * CY;    % Side force (N) — negative for positive beta

    %% ---- Transform from wind frame to body frame -----------------------
    % Using wind-to-body transformation via alpha and beta:
    ca = cos(alpha);  sa = sin(alpha);
    cb = cos(beta);   sb = sin(beta);

    % Body-frame aerodynamic forces
    % The negative drag acts backward; lift acts in -z; side in ±y
    Fx_aero =  -D*ca*cb + L*sa - Y*ca*sb;   % Axial force  (forward = positive)
    Fy_aero =  -D*sb    + Y*cb;             % Side  force  (right   = positive)
    Fz_aero =  -D*sa*cb - L*ca - Y*sa*sb;  % Normal force (down    = positive)

    %% ---- Sanity bounds (flag numerical issues, not physical limiting) --
    Fx_aero = clamp(Fx_aero, -200e3, 200e3);
    Fy_aero = clamp(Fy_aero, -200e3, 200e3);
    Fz_aero = clamp(Fz_aero, -200e3, 200e3);
end

%% ---- Helper: saturate value to [lo, hi] --------------------------------
function y = clamp(x, lo, hi)
    y = max(lo, min(hi, x));
end
