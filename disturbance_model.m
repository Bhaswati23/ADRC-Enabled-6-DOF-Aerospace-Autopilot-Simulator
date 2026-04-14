% =========================================================================
% DISTURBANCE_MODEL.M
% Wind and Atmospheric Disturbance Model
%
% DISTURBANCE COMPONENTS:
%
% 1. STEADY WIND (Constant background wind in NED frame):
%    A constant headwind (negative North component) representing a
%    sustained atmospheric flow field. Affects lift, drag, and moments
%    through changes in effective angle of attack.
%
% 2. STEP GUST (Discrete gust event):
%    A sudden increase in wind velocity at a defined time, sustained
%    for a defined duration. Models atmospheric boundaries (shear layers)
%    or localized turbulence events.
%    Per MIL-SPEC: a 1-cos gust profile is realistic, simplified to step here.
%
% 3. DRYDEN TURBULENCE (Stochastic component):
%    Low-intensity random turbulence modeled as band-limited white noise
%    filtered through a first-order Dryden shaping filter.
%    Spectral density: Φ(ω) = σ²*L/(π*V) * 1/(1 + (L*ω/V)²)
%    Simplified as: noise amplitude σ_turb * randn() per step
%
% PHYSICAL INTERPRETATION FOR ADRC:
%   The wind disturbance changes the missile's aerodynamic angles
%   (alpha, beta) and thus moments. The ADRC/ESO treats this as part
%   of the "total disturbance" f(t) and compensates automatically.
%   This demonstrates ADRC's key advantage: no explicit disturbance
%   model is needed in the controller — ESO estimates it in real time.
%
% COORDINATE FRAME:
%   Wind is expressed in NED inertial frame [wx_N; wy_E; wz_D] (m/s).
%   Positive wx_N = North wind (tailwind for northward missile).
%   Positive wz_D = downward component (unusual — usually negligible).
%
% INPUT:
%   t — current simulation time (s)
%
% OUTPUT:
%   wind — wind velocity in NED frame [wx; wy; wz] (m/s)
%
% DISTURBANCE SCHEDULE:
%   t < 5s:        Steady headwind only (-10 m/s North = headwind)
%   5s ≤ t < 8s:   Step gust added (+15 m/s vertical = downburst)
%   t ≥ 8s:        Return to steady wind + turbulence
% =========================================================================

function wind = disturbance_model(t)

    %% ---- Steady background wind (NED frame) ----------------------------
    % Headwind component: -10 m/s North (opposes northward flight)
    % Cross-wind component: +5 m/s East (lateral disturbance)
    wind_steady = [-10; 5; 0];    % (m/s) NED

    %% ---- Step gust parameters ------------------------------------------
    gust_start    = 5.0;     % s — gust onset time
    gust_duration = 3.0;     % s — gust duration
    gust_end      = gust_start + gust_duration;

    % Gust intensity: downward vertical gust (most challenging for pitch)
    gust_amplitude = [0; 0; 8];   % m/s NED (downward = positive z in NED)

    % 1-cosine gust profile (smoother than pure step, more realistic)
    if t >= gust_start && t < gust_end
        tau = (t - gust_start) / gust_duration;    % 0 → 1 during gust
        gust_shape = 0.5 * (1 - cos(pi * tau));    % smooth 0→1→0 envelope
        wind_gust = gust_amplitude * gust_shape;
    else
        wind_gust = [0; 0; 0];
    end

    %% ---- Dryden turbulence (stochastic component) ----------------------
    % Turbulence intensity (light turbulence: σ ≈ 1–2 m/s)
    sigma_turb = 1.5;      % m/s  (RMS turbulence intensity)
    L_turb     = 200;      % m    (turbulence length scale)
    V_ref      = 300;      % m/s  (reference airspeed for filter)
    dt_ref     = 0.005;    % s    (simulation time step)

    % Approximate Dryden as scaled white noise (acceptable for short simulations)
    % For production code: implement proper shaping filter H(s) = σ*sqrt(2L/πV)/(1 + Ls/V)
    turb_scale = sigma_turb * sqrt(2*L_turb / (pi * V_ref * dt_ref));
    turb_scale = min(turb_scale, sigma_turb * 3);  % Cap at 3σ for stability
    wind_turb = turb_scale * randn(3,1) * dt_ref;

    %% ---- Total wind disturbance ----------------------------------------
    wind = wind_steady + wind_gust + wind_turb;

    %% ---- Bound total wind (physical maximum ~50 m/s) -------------------
    wind = max(-50, min(50, wind));
end
