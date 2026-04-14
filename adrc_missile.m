clc;
clear;
close all;

% =========================================================
%  MISSILE 6-DOF + ADRC PITCH CONTROLLER  (Pure MATLAB ODE)
%  Run this file directly: generates all 13 required plots
% =========================================================

%% 1. Missile Physical Parameters
m   = 150;       % mass (kg)
Ix  = 10;        % roll  inertia (kg.m^2)
Iy  = 20;        % pitch inertia
Iz  = 25;        % yaw   inertia

rho = 1.225;     % air density (kg/m^3)
S   = 0.50;      % reference wing area (m^2)
b   = 1.0;       % span  (m)
c   = 0.5;       % chord (m)

g   = 9.81;      % gravity (m/s^2)

% Aerodynamic coefficients (linear model)
CLa  =  4.0;     % lift-curve slope (1/rad)
CDa  =  0.10;    % drag vs AoA coefficient
CD0  =  0.02;    % zero-lift drag
Cma  = -1.50;    % pitch moment slope (statically stable)
Cmq  = -8.0;     % pitch-rate damping
Cyb  = -0.80;    % side-force vs sideslip
Cnb  =  0.60;    % yaw moment slope
Clp  = -0.40;    % roll-rate damping
Cm_de =  1.20;   % pitch moment per rad elevator
Cn_dr =  0.50;   % yaw   moment per rad rudder

%% 2. Initial Conditions
U0     = 300;    % forward body velocity (m/s)
V0     = 0;
W0     = 0;
P0     = 0;      % roll  rate (rad/s)
Q0     = 0;      % pitch rate
R0     = 0;      % yaw   rate
phi0   = 0;      % roll  angle (rad)
theta0 = 0;      % pitch angle
psi0   = 0;      % yaw   angle
X0     = 0;      % inertial X (m)
Y0     = 0;      % inertial Y
Z0     = -2000;  % inertial Z  (negative = altitude in NED)

target_X = 5000;
target_Y = 0;
target_Z = -2000;

%% 3. ADRC Pitch-Controller Parameters
wc = 8;          % controller bandwidth (rad/s)
wo = 50;         % observer  bandwidth  (rad/s)

kp = wc^2;
kd = 2*wc;

beta1 = 3*wo;
beta2 = 3*wo^2;
beta3 =   wo^3;

theta_ref = 5*pi/180;   % 5-degree pitch command (rad)

% ESO plant gain: b0 = Cm_de * q_bar * S * c / Iy  (at trim)
b0 = Cm_de * 0.5*rho*U0^2*S*c / Iy;

%% 4. Pack all parameters into struct for ODE function
par.m    = m;   par.Ix = Ix;  par.Iy = Iy;  par.Iz = Iz;
par.rho  = rho; par.S  = S;   par.b  = b;   par.c  = c;
par.g    = g;
par.CLa  = CLa; par.CDa = CDa; par.CD0 = CD0;
par.Cma  = Cma; par.Cmq = Cmq;
par.Cyb  = Cyb; par.Cnb = Cnb; par.Clp = Clp;
par.Cm_de = Cm_de; par.Cn_dr = Cn_dr;
par.kp   = kp;  par.kd = kd;
par.beta1 = beta1; par.beta2 = beta2; par.beta3 = beta3;
par.theta_ref = theta_ref;
par.b0   = b0;

%% 5. State Vector Layout
% Index : 1   2   3   4   5   6    7     8      9    10  11  12   13  14  15
% State : U   V   W   P   Q   R   phi  theta   psi   X   Y   Z   z1  z2  z3
% z1,z2,z3 are the 3rd-order ESO states
%   z1 ~ theta  (pitch angle estimate)
%   z2 ~ Q      (pitch rate estimate)
%   z3 ~ total disturbance estimate

x0 = [U0; V0; W0; P0; Q0; R0; phi0; theta0; psi0; X0; Y0; Z0; theta0; 0; 0];

%% 6. ODE Integration
tspan = [0 40];
opts  = odeset('RelTol',1e-6, 'AbsTol',1e-8, 'MaxStep',0.05);

disp('Running 6-DOF + ADRC simulation ...');
[t, Xst] = ode45(@(t,x) eom6dof(t, x, par), tspan, x0, opts);
fprintf('Done. Integration steps = %d\n', length(t));

%% 7. Extract States
U_v   = Xst(:,1);   V_v    = Xst(:,2);   W_v   = Xst(:,3);
P_v   = Xst(:,4);   Q_v    = Xst(:,5);   R_v   = Xst(:,6);
phi_v = Xst(:,7);   theta_v = Xst(:,8);  psi_v = Xst(:,9);
Xp    = Xst(:,10);  Yp     = Xst(:,11);  Zp    = Xst(:,12);
z1_v  = Xst(:,13);  z2_v   = Xst(:,14);  z3_v  = Xst(:,15);

%% 8. Derived Quantities
airspeed = sqrt(U_v.^2 + V_v.^2 + W_v.^2);
alpha_v  = atan2(W_v, U_v) * 180/pi;           % deg
beta_v   = atan2(V_v, airspeed) * 180/pi;       % deg

% Recompute ADRC outputs from ESO states
e1      = theta_ref - z1_v;
e2      = 0         - z2_v;
u_pd    = par.kp*e1 + par.kd*e2;
delta_e = (u_pd - z3_v) / par.b0;
delta_e = max(min(delta_e, 0.40), -0.40);       % saturation +/-23 deg
delta_e_deg = delta_e * 180/pi;

pitch_error = (theta_ref - theta_v) * 180/pi;   % deg

range_v = sqrt((Xp - target_X).^2 + (Yp - target_Y).^2 + (Zp - target_Z).^2);

eso_dist = z3_v;   % disturbance estimate

%% 9. Plots
LW = 1.8;   % line width

% ----- Figure 1: Body Velocities & Aero Angles ---------------------------
figure('Name','Velocities and Aero Angles','NumberTitle','off',...
       'Position',[40 40 1360 880]);

subplot(3,2,1)
plot(t, U_v, 'b', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('U  (m/s)');
title('Forward Body Velocity'); grid on;

subplot(3,2,2)
plot(t, V_v, 'r', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('V  (m/s)');
title('Lateral Body Velocity'); grid on;

subplot(3,2,3)
plot(t, W_v, 'm', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('W  (m/s)');
title('Vertical Body Velocity'); grid on;

subplot(3,2,4)
plot(t, airspeed, 'k', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('V_T  (m/s)');
title('Total Airspeed'); grid on;

subplot(3,2,5)
plot(t, alpha_v, 'g', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('\alpha  (deg)');
title('Angle of Attack'); grid on;

subplot(3,2,6)
plot(t, beta_v, 'c', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('\beta  (deg)');
title('Sideslip Angle'); grid on;

sgtitle('Missile 6-DOF  |  Velocity & Aerodynamic Angles',...
        'FontSize',13,'FontWeight','bold');

% ----- Figure 2: ADRC Controller Performance -----------------------------
figure('Name','ADRC Controller Performance','NumberTitle','off',...
       'Position',[70 70 1360 880]);

subplot(3,2,1)
plot(t, theta_v*180/pi, 'b', 'LineWidth', LW); hold on;
yline(theta_ref*180/pi, 'r--', 'LineWidth', 1.4);
xlabel('Time (s)'); ylabel('\theta  (deg)');
title('Pitch Angle vs Reference'); grid on;
legend('Actual \theta','Reference','Location','southeast');

subplot(3,2,2)
plot(t, pitch_error, 'r', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('e_\theta  (deg)');
title('Pitch Tracking Error'); grid on;

subplot(3,2,3)
plot(t, delta_e_deg, 'm', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('\delta_e  (deg)');
title('Elevator Command'); grid on;

subplot(3,2,4)
plot(t, eso_dist, 'k', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('z_3');
title('ESO Disturbance Estimate  (z_3)'); grid on;

subplot(3,2,5)
plot(t, Q_v*180/pi, 'b', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('Q  (deg/s)');
title('Pitch Rate'); grid on;

subplot(3,2,6)
plot(t, range_v/1000, 'g', 'LineWidth', LW);
xlabel('Time (s)'); ylabel('Range  (km)');
title('Missile-Target Range'); grid on;

sgtitle('Missile 6-DOF  |  ADRC Controller Performance',...
        'FontSize',13,'FontWeight','bold');

% ----- Figure 3: 3-D Trajectory ------------------------------------------
figure('Name','3D Trajectory','NumberTitle','off',...
       'Position',[100 100 820 640]);

plot3(Xp/1000, Yp/1000, -Zp/1000, 'b', 'LineWidth', 2); hold on;
plot3(0, 0, -Z0/1000, 'go', 'MarkerSize',12, 'MarkerFaceColor','g');
plot3(target_X/1000, target_Y/1000, -target_Z/1000,...
      'r^', 'MarkerSize',14, 'MarkerFaceColor','r');

xlabel('X  (km)'); ylabel('Y  (km)'); zlabel('Altitude  (km)');
title('3-D Missile Trajectory','FontSize',13,'FontWeight','bold');
legend('Trajectory','Launch Point','Target','Location','best');
grid on; view(35, 25);

%% 10. Console Summary
fprintf('\n========== Simulation Summary ==========\n');
fprintf('Final airspeed       : %.1f m/s\n',   airspeed(end));
fprintf('Final pitch angle    : %.3f deg\n',   theta_v(end)*180/pi);
fprintf('Steady-state error   : %.4f deg\n',   pitch_error(end));
fprintf('Final elevator cmd   : %.3f deg\n',   delta_e_deg(end));
fprintf('Final range to target: %.1f m\n',     range_v(end));
fprintf('Final altitude       : %.1f m\n',    -Zp(end));
fprintf('=========================================\n');

% =========================================================
%  LOCAL FUNCTION: 6-DOF + ESO Equations of Motion
%  State: [U V W  P Q R  phi theta psi  X Y Z  z1 z2 z3]
% =========================================================
function dxdt = eom6dof(~, x, par)

    % Unpack states
    U     = x(1);
    V     = x(2);
    W     = x(3);
    P     = x(4);
    Q     = x(5);
    R     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    z1    = x(13);
    z2    = x(14);
    z3    = x(15);

    % Airspeed and aero angles
    Vt    = sqrt(U^2 + V^2 + W^2);
    Vt    = max(Vt, 1.0);
    alpha = atan2(W, U);
    beta  = atan2(V, Vt);
    qbar  = 0.5 * par.rho * Vt^2;

    % ADRC control law
    e1      = par.theta_ref - z1;
    e2      = 0             - z2;
    u_pd    = par.kp*e1 + par.kd*e2;
    delta_e = (u_pd - z3) / par.b0;
    delta_e = max(min(delta_e, 0.40), -0.40);
    delta_r = 0;

    % Aerodynamic coefficients
    CL = par.CLa * alpha;
    CD = par.CD0 + par.CDa * alpha^2;
    CY = par.Cyb * beta;

    % Aerodynamic forces in wind frame, then rotate to body
    Lift = qbar * par.S * CL;
    Drag = qbar * par.S * CD;
    Side = qbar * par.S * CY;

    ca = cos(alpha);  sa = sin(alpha);
    cb = cos(beta);   sb = sin(beta);

    Fax =  -Drag*ca*cb - Side*ca*sb - Lift*sa;
    Fay =  -Drag*sb    + Side*cb;
    Faz =  -Drag*sa*cb - Side*sa*sb + Lift*ca;

    % Gravity components in body frame
    Fgx = -par.m * par.g * sin(theta);
    Fgy =  par.m * par.g * cos(theta) * sin(phi);
    Fgz =  par.m * par.g * cos(theta) * cos(phi);

    % Translational equations of motion
    dU = (Fax + Fgx)/par.m + R*V - Q*W;
    dV = (Fay + Fgy)/par.m + P*W - R*U;
    dW = (Faz + Fgz)/par.m + Q*U - P*V;

    % Aerodynamic moments
    Cm = par.Cma*alpha + par.Cmq*(par.c/(2*Vt))*Q + par.Cm_de*delta_e;
    Cn = par.Cnb*beta  + par.Cn_dr*delta_r;
    Cl = par.Clp*(par.b/(2*Vt))*P;

    Ma = qbar * par.S * par.c * Cm;
    Na = qbar * par.S * par.b * Cn;
    La = qbar * par.S * par.b * Cl;

    % Rotational equations of motion (Euler)
    dP = (La + (par.Iy - par.Iz)*Q*R) / par.Ix;
    dQ = (Ma + (par.Iz - par.Ix)*P*R) / par.Iy;
    dR = (Na + (par.Ix - par.Iy)*P*Q) / par.Iz;

    % Euler angle kinematics
    dphi   = P + (Q*sin(phi) + R*cos(phi)) * tan(theta);
    dtheta = Q*cos(phi) - R*sin(phi);
    dpsi   = (Q*sin(phi) + R*cos(phi)) / max(cos(theta), 0.017);

    % Position kinematics: body velocity -> NED
    cphi = cos(phi);  sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);

    Rib = [ cth*cpsi,               cth*spsi,              -sth;
            sphi*sth*cpsi-cphi*spsi, sphi*sth*spsi+cphi*cpsi, sphi*cth;
            cphi*sth*cpsi+sphi*spsi, cphi*sth*spsi-sphi*cpsi, cphi*cth ];

    vned = Rib' * [U; V; W];
    dX = vned(1);
    dY = vned(2);
    dZ = vned(3);

    % 3rd-order ESO (observes theta, estimates disturbance)
    y_obs = theta;
    eobs  = z1 - y_obs;

    dz1 = z2 - par.beta1 * eobs;
    dz2 = z3 - par.beta2 * eobs + par.b0 * delta_e;
    dz3 =    - par.beta3 * eobs;

    % Assemble derivative vector
    dxdt = [dU; dV; dW;
            dP; dQ; dR;
            dphi; dtheta; dpsi;
            dX; dY; dZ;
            dz1; dz2; dz3];
end