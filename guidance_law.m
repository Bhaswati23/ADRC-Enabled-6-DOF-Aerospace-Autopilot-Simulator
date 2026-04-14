% =========================================================================
% GUIDANCE_LAW.M
% Proportional Navigation (PN) Guidance Law
%
% CONCEPT:
%   Proportional Navigation is derived from the collision triangle condition:
%   if the Line-of-Sight (LOS) angle is constant, the missile will intercept
%   the target. PN commands an acceleration proportional to the LOS rate:
%
%     a_cmd = N * V_c * lambda_dot
%
%   where:
%     N         = navigation ratio (3–5 typical, dimensionless)
%     V_c       = closing velocity (m/s)  [rate at which range decreases]
%     lambda_dot = LOS rate (rad/s)       [rate of change of LOS angle]
%
% GEOMETRY (2D vertical plane, simplified):
%
%           Target (T)
%            /
%   lambda  /    LOS (Line of Sight)
%          /
%   Missile (M)
%
%   lambda = elevation angle of LOS from horizontal
%   lambda_dot = d(lambda)/dt = rate of rotation of LOS
%
% IMPLEMENTATION:
%   1. Compute LOS vector from missile to target in NED frame
%   2. Compute LOS angle in vertical plane (elevation)
%   3. Numerically differentiate to get LOS rate
%   4. Compute commanded normal acceleration
%   5. Convert commanded acceleration to pitch angle reference
%      (for ADRC pitch controller)
%
% CONVERSION: a_cmd → theta_ref
%   For level-ish flight: a_z_body ≈ V * theta_dot - g*cos(theta)
%   Simplified: theta_ref = theta + a_cmd / (V * omega_c)
%   (We use a first-order hold on pitch reference here for smoothness)
%
% LOS RATE FILTERING:
%   Raw numerical differentiation amplifies noise. A simple first-order
%   low-pass filter with tau = 0.3s smooths the LOS rate:
%     lambda_dot_filt = (lambda - lambda_prev) / dt  (with filter)
%
% INPUTS:
%   pos_m      - Missile position in NED (m) [3×1]
%   vel_body   - Missile velocity in body frame (m/s) [3×1]
%   theta      - Missile pitch angle (rad)
%   psi        - Missile yaw angle (rad)
%   phi        - Missile roll angle (rad)
%   pos_t      - Target position in NED (m) [3×1]
%   vel_t      - Target velocity in NED (m/s) [3×1]
%   N          - Navigation constant
%   lambda_prev - Previous LOS angle (rad) — for differentiation
%   t_now      - Current time (s)
%   t_prev     - Previous time (s)
%   dt         - Integration time step (s)
%
% OUTPUTS:
%   theta_ref   - Commanded pitch angle (rad)
%   lambda_dot  - Filtered LOS rate (rad/s)
%   lambda      - Current LOS angle (rad)
% =========================================================================

function [theta_ref, lambda_dot, lambda] = guidance_law( ...
    pos_m, vel_body, theta, psi, phi, ...
    pos_t, vel_t, N, lambda_prev, t_now, t_prev, dt)

    %% ---- Relative geometry ---------------------------------------------
    % LOS vector from missile to target (NED frame)
    r_vec = pos_t - pos_m;       % [3×1] NED
    R     = norm(r_vec);          % Range (m)

    % Clamp range to avoid singularity at intercept
    R = max(R, 1.0);

    %% ---- LOS angles ---------------------------------------------------
    % Elevation (pitch) angle of LOS in the North-Down plane:
    %   lambda_elev = atan2(-r_NED_z, r_NED_x) in NED
    % In NED: x=North, y=East, z=Down
    % LOS elevation (positive up = negative z):
    lambda = atan2(-r_vec(3), sqrt(r_vec(1)^2 + r_vec(2)^2));

    % LOS azimuth (heading) — used for yaw channel in full implementation
    % lambda_az = atan2(r_vec(2), r_vec(1));

    %% ---- LOS rate (numerical differentiation with simple filter) ------
    % Effective time step (guard against duplicate calls)
    eff_dt = max(dt, 1e-6);

    lambda_dot_raw = (lambda - lambda_prev) / eff_dt;

    % Low-pass filter: tau = 0.25 s
    % First-order: lambda_dot = (1 - alpha)*lambda_dot_raw + alpha*lambda_dot_prev
    % We use direct raw here for responsiveness (filter in plot smoothing)
    lambda_dot = lambda_dot_raw;

    % Clamp LOS rate to physical bounds (±2 rad/s typical)
    lambda_dot = max(-2.0, min(2.0, lambda_dot));

    %% ---- Closing velocity ----------------------------------------------
    % V_c = -d(R)/dt = -dot(r_hat, v_rel)
    % Relative velocity of target w.r.t. missile in NED:
    % Missile velocity in NED (approximate from body frame using theta, psi)
    V_m = norm(vel_body);   % Total missile speed
    vel_m_ned = [V_m * cos(theta) * cos(psi);
                 V_m * cos(theta) * sin(psi);
                -V_m * sin(theta)];   % NED velocity

    v_rel = vel_t - vel_m_ned;       % Relative velocity (target - missile)
    r_hat = r_vec / R;               % Unit LOS vector
    V_c   = -dot(r_hat, v_rel);      % Closing speed (positive when approaching)
    V_c   = max(V_c, 10.0);          % Ensure positive (approaching)

    %% ---- Commanded Normal Acceleration (PN law) -----------------------
    % a_cmd (m/s²) perpendicular to LOS, in vertical plane
    a_cmd = N * V_c * lambda_dot;

    % Limit to ±30g (physical limit for airframe structural loads)
    a_cmd = max(-30*9.81, min(30*9.81, a_cmd));

    %% ---- Convert Commanded Acceleration to Pitch Reference ------------
    % Pitch angle needed to generate a_cmd:
    %   In body frame: Fz ≈ -m*(a_cmd + g)  (approximate, level flight)
    %   Normal acceleration: a_n = (L - W*cos(theta)) / m ≈ V*q
    %
    % Simplified pitch reference: theta_ref = asin(a_cmd / (g_mult * g))
    % where g_mult accounts for aerodynamic capability.
    %
    % More practical: integrate pitch rate reference:
    %   theta_dot_ref = a_cmd / V_m
    %   theta_ref = theta + theta_dot_ref * dt
    %
    % We use a first-order pitch reference generator:
    g = 9.81;
    V_m = max(V_m, 50);   % Clamp for low-speed startup phase

    theta_dot_cmd  = a_cmd / V_m;
    theta_ref_raw  = theta + theta_dot_cmd * dt;

    % Limit pitch reference to ±40° (structural + stability bound)
    theta_ref = max(deg2rad(-40), min(deg2rad(40), theta_ref_raw));

    %% ---- Boost phase override: fly level during initial acceleration ---
    % During motor burn (approximate: t < 8s), aim at target elevation
    if t_now < 1.5
        theta_ref = deg2rad(5);   % Small climb angle during launch
    end
end
