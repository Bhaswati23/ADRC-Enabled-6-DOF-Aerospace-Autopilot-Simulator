% =========================================================================
% SIXDOF_DYNAMICS.M
% Newton-Euler 6-DOF Rigid Body Equations of Motion
%
% STATE VECTOR (12 components):
%   X = [u, v, w, p, q, r, phi, theta, psi, xI, yI, zI]'
%
% COORDINATE FRAMES:
%   Body Frame (B):
%     - Origin: missile center of gravity (CG)
%     - x_B: points forward along missile axis (roll axis)
%     - y_B: points right (starboard)
%     - z_B: points down (completing right-hand system)
%
%   Inertial Frame (I) — NED:
%     - x_I: North
%     - y_I: East
%     - z_I: Down (positive downward)
%
% EQUATIONS OF MOTION:
%   Translational (body frame, Newton's 2nd Law):
%     m*(du/dt + q*w - r*v) = Fx   [X-force, forward]
%     m*(dv/dt + r*u - p*w) = Fy   [Y-force, right]
%     m*(dw/dt + p*v - q*u) = Fz   [Z-force, down]
%
%   Rotational (body frame, Euler's equations):
%     Ixx*dp/dt - Ixz*dr/dt + (Izz-Iyy)*q*r - Ixz*p*q = Mx  [roll]
%     Iyy*dq/dt + (Ixx-Izz)*p*r + Ixz*(p²-r²)         = My  [pitch]
%     Izz*dr/dt - Ixz*dp/dt + (Iyy-Ixx)*p*q + Ixz*q*r = Mz  [yaw]
%
%   Kinematics (Euler angle rates):
%     phi_dot   = p + (q*sin(phi) + r*cos(phi))*tan(theta)
%     theta_dot = q*cos(phi) - r*sin(phi)
%     psi_dot   = (q*sin(phi) + r*cos(phi))/cos(theta)
%
%   Position (rotation of body velocities to inertial frame):
%     [xI_dot; yI_dot; zI_dot] = R_BI * [u; v; w]
%     where R_BI is the body-to-inertial Direction Cosine Matrix (DCM)
%
% INPUTS:
%   X        - State vector (12x1)
%   Fx,Fy,Fz - Total forces in body frame (N)  [aero + thrust]
%   Mx,My,Mz - Total moments in body frame (N·m)
%   wind     - Wind velocity in inertial NED frame [wx;wy;wz] (m/s)
%   missile  - Missile parameter struct
%
% OUTPUT:
%   Xdot     - Time derivative of state vector (12x1)
% =========================================================================

function Xdot = sixdof_dynamics(X, Fx, Fy, Fz, Mx, My, Mz, wind, missile)

    %% ---- Extract states ----
    u   = X(1);   v   = X(2);   w   = X(3);   % body velocities (m/s)
    p   = X(4);   q   = X(5);   r   = X(6);   % body angular rates (rad/s)
    phi = X(7);   theta = X(8); psi = X(9);   % Euler angles (rad)
    % X(10:12) = [xI, yI, zI] — position not needed in dynamics

    %% ---- Missile inertia parameters ----
    m   = missile.mass;
    Ixx = missile.Ixx;
    Iyy = missile.Iyy;
    Izz = missile.Izz;
    Ixz = missile.Ixz;

    %% ---- Wind: transform inertial wind to body frame ----
    % Direction Cosine Matrix: Inertial → Body
    R_IB = dcm_inertial_to_body(phi, theta, psi);
    wind_body = R_IB * wind;       % Wind in body frame

    % Relative airspeed components in body frame
    % (subtract wind from body velocity to get airspeed)
    % Note: wind is already accounted for in aerodynamic forces via alpha/beta
    % Here we add wind force effect on the missile body:
    Fx = Fx + m * (wind_body(1) / 20);   % Simplified wind force (gust effect)
    Fz = Fz + m * (wind_body(3) / 20);

    %% ---- Gravity in body frame ----
    % Gravity vector in NED: g_I = [0; 0; g] (g positive downward)
    g = 9.81;   % m/s²
    % Transform gravity from inertial to body frame
    g_body = R_IB * [0; 0; g];
    Fx = Fx + m * g_body(1);
    Fy = Fy + m * g_body(2);
    Fz = Fz + m * g_body(3);

    %% =========================================================
    %  TRANSLATIONAL DYNAMICS (Newton's 2nd Law in body frame)
    %  Coriolis terms arise because body frame is rotating.
    %
    %  F = m*(dV/dt)_body + omega × (m*V)
    %  => dV/dt = F/m - omega × V
    % =========================================================
    du = Fx/m - (q*w - r*v);
    dv = Fy/m - (r*u - p*w);
    dw = Fz/m - (p*v - q*u);

    %% =========================================================
    %  ROTATIONAL DYNAMICS (Euler's equations, with Ixz coupling)
    %
    %  [Ixx -Ixz] [dp/dt]   [Mx - (Izz-Iyy)*q*r + Ixz*p*q]
    %  [-Ixz Izz] [dr/dt] = [Mz - (Iyy-Ixx)*p*q - Ixz*q*r]
    %
    %  Solve 2x2 system for dp, dr:
    % =========================================================
    Gamma = Ixx*Izz - Ixz^2;   % Determinant of inertia sub-matrix

    dp = (Izz*Mx + Ixz*Mz - (Izz*(Izz-Iyy) + Ixz^2)*q*r + Ixz*(Ixx-Iyy+Izz)*p*q) / Gamma;
    dq = (My - (Ixx-Izz)*p*r - Ixz*(p^2 - r^2)) / Iyy;
    dr = (Ixx*Mz + Ixz*Mx + (Ixx*(Iyy-Ixx) + Ixz^2)*p*q + Ixz*(Iyy-Ixx-Izz)*q*r) / Gamma;

    %% =========================================================
    %  KINEMATIC EQUATIONS (Euler angle rates from body rates)
    %
    %  Singularity at theta = ±90°. For a missile this is
    %  typically avoided; a quaternion formulation removes it.
    %
    %  [phi_dot  ]   [1  sin(phi)*tan(theta)  cos(phi)*tan(theta)] [p]
    %  [theta_dot] = [0  cos(phi)            -sin(phi)           ] [q]
    %  [psi_dot  ]   [0  sin(phi)/cos(theta)  cos(phi)/cos(theta)] [r]
    % =========================================================
    cos_theta = cos(theta);
    if abs(cos_theta) < 0.01
        cos_theta = sign(cos_theta)*0.01;   % Clamp near singularity
    end

    dphi   = p + (q*sin(phi) + r*cos(phi)) * tan(theta);
    dtheta = q*cos(phi) - r*sin(phi);
    dpsi   = (q*sin(phi) + r*cos(phi)) / cos_theta;

    %% =========================================================
    %  POSITION KINEMATICS (body → inertial frame)
    %
    %  [xI_dot]             [u]
    %  [yI_dot] = R_BI(3x3) [v]
    %  [zI_dot]             [w]
    %
    %  R_BI = R_BI(phi,theta,psi): Body-to-Inertial DCM (transpose of R_IB)
    % =========================================================
    R_BI = R_IB';    % Body-to-Inertial = transpose of Inertial-to-Body

    vel_inertial = R_BI * [u; v; w];
    dxI = vel_inertial(1);
    dyI = vel_inertial(2);
    dzI = vel_inertial(3);

    %% ---- Assemble state derivative vector ----
    Xdot = [du; dv; dw; dp; dq; dr; dphi; dtheta; dpsi; dxI; dyI; dzI];
end

%% =========================================================================
%  DCM: INERTIAL (NED) → BODY FRAME
%  Sequence: psi (yaw) → theta (pitch) → phi (roll)  [3-2-1 Euler]
%
%  R_IB = Rx(phi) * Ry(theta) * Rz(psi)
% =========================================================================
function R_IB = dcm_inertial_to_body(phi, theta, psi)
    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);

    % Standard 3-2-1 DCM
    R_IB = [cth*cpsi,              cth*spsi,             -sth;
            sphi*sth*cpsi-cphi*spsi, sphi*sth*spsi+cphi*cpsi, sphi*cth;
            cphi*sth*cpsi+sphi*spsi, cphi*sth*spsi-sphi*cpsi, cphi*cth];
end
