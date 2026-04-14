%% build_missile_pitch_model.m
% Builds the Missile 6DOF Pitch Control System with ADRC and ESO
% Run this script in MATLAB to auto-generate the Simulink model.
% Requirements: MATLAB + Simulink (R2020a or later recommended)

%% ---- PARAMETERS (tune as needed) ----
Kp   = 2.0;       % Proportional gain
b0   = 1.0;       % Control input gain estimate (1/b0 used in controller)
sat_lo = -0.4;    % Elevator saturation lower limit (rad)
sat_hi =  0.4;    % Elevator saturation upper limit (rad)
beta1  = 30;      % ESO gain 1
beta2  = 300;     % ESO gain 2
beta3  = 1000;    % ESO gain 3
Kd     = 5;       % Derivative-like gain fed from z3 (disturbance rejection)

%% ---- Create / open model ----
mdl = 'Missile_ADRC_Pitch';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);
set_param(mdl, 'SolverType','Fixed-step', 'Solver','ode4', ...
               'FixedStep','0.001', 'StopTime','10');

%% ================================================================
%  HELPER: add_block shortcut
% ================================================================
function h = ab(mdl, lib_path, name, pos, varargin)
    h = add_block(lib_path, [mdl '/' name], 'Position', pos);
    for k = 1:2:length(varargin)
        set_param(h, varargin{k}, varargin{k+1});
    end
end

%% ================================================================
%  SECTION 1 – TOP-LEVEL KINEMATICS (velocity & position)
%  Mirrors the top rows of the diagram
%% ================================================================

% --- Step (Theta_ref) ---
ab(mdl,'simulink/Sources/Step','Theta_ref',...
   [30 360 90 390],...
   'Time','0','Before','0','After','1','SampleTime','0');

% --- U_velocity gain ---
ab(mdl,'simulink/Math Operations/Gain','U_velocity',...
   [160 60 220 90],'Gain','1','ShowName','on');

% --- V_velocity gain ---
ab(mdl,'simulink/Math Operations/Gain','V_velocity',...
   [280 60 340 90],'Gain','1','ShowName','on');

% --- X_position  (1/z discrete integrator) ---
ab(mdl,'simulink/Discrete/Unit Delay','X_position',...
   [400 55 450 95],'SampleTime','0.001');

% --- W_velocity summing junction ---
ab(mdl,'simulink/Math Operations/Sum','W_velocity_sum',...
   [155 150 185 180],'Inputs','++','ShowName','on');

% --- Sum of Elements (sqrt input bus) ---
ab(mdl,'simulink/Math Operations/Math Function','SumOfElements',...
   [220 140 290 180],'Operator','square','ShowName','on');

% NOTE: U²+V²+W² is formed by summing squares then sqrt.
%       We use a Fcn block for clarity.
add_block('simulink/User-Defined Functions/Fcn',...
   [mdl '/SpeedMagnitude'],'Position',[220 140 310 180],...
   'Expr','sqrt(u(1)^2 + u(2)^2 + u(3)^2)');
set_param([mdl '/SpeedMagnitude'],'ShowName','on');
% (Delete the placeholder above and keep Fcn)
delete_block([mdl '/SumOfElements']);

% --- Square Root block (visual alias – already inside Fcn above) ---
%     We add a label-only annotation instead
add_annotation(mdl, 'sqrt( U²+V²+W² )', [220 195]);

% --- Pitch_angle  (1/z) ---
ab(mdl,'simulink/Discrete/Unit Delay','Pitch_angle',...
   [360 140 410 180],'SampleTime','0.001');

% --- alpha = atan(W/U) Fcn ---
ab(mdl,'simulink/User-Defined Functions/Fcn','alpha_calc',...
   [30 230 160 260],'Expr','atan(u(2)/u(1))');
set_param([mdl '/alpha_calc'],'ShowName','on');

% --- alpha Unit Delay ---
ab(mdl,'simulink/Discrete/Unit Delay','alpha_delay',...
   [30 275 90 305],'SampleTime','0.001');

%% ================================================================
%  SECTION 2 – POSITION CHAIN  X → Y → Z  (top right)
%% ================================================================
ab(mdl,'simulink/Discrete/Unit Delay','X_pos_r',...
   [530 50 580 80],'SampleTime','0.001');
ab(mdl,'simulink/Discrete/Unit Delay','Y_pos_r',...
   [620 50 670 80],'SampleTime','0.001');
ab(mdl,'simulink/Discrete/Unit Delay','Z_pos_r',...
   [710 50 760 80],'SampleTime','0.001');

% Invert block (1/x, 1/Y chain)
ab(mdl,'simulink/Math Operations/Math Function','Invert_top',...
   [620 110 670 140],'Operator','reciprocal','ShowName','on');

% Q block
ab(mdl,'simulink/User-Defined Functions/Fcn','Q_block',...
   [620 160 670 190],'Expr','u(1)','ShowName','on');

%% ================================================================
%  SECTION 3 – MISSILE 6DOF DYNAMICS (subsystem)
%% ================================================================
% Create a subsystem to represent the 6DOF plant
add_block('simulink/Ports & Subsystems/Subsystem',...
   [mdl '/Missile_6DOF'],[700 200 900 340]);
set_param([mdl '/Missile_6DOF'],'ShowName','on');

% Open 6DOF subsystem and build internals
open_system([mdl '/Missile_6DOF']);
m6 = [mdl '/Missile_6DOF'];

% Delete default In1→Out1 connection
delete_line(m6,'In1/1','Out1/1');

% Add ports
add_block('simulink/Sources/In1',[m6 '/delta_e'],  'Position',[30 60 60 80]);   % elevator input
add_block('simulink/Sources/In1',[m6 '/z3_in'],    'Position',[30 120 60 140]); % disturbance
add_block('simulink/Sinks/Out1', [m6 '/theta_out'],'Position',[300 60 330 80]);
add_block('simulink/Sinks/Out1', [m6 '/zdot3_out'],'Position',[300 120 330 140]);

% Simple second-order pitch dynamics: θ'' = a*θ + b*δ_e  (linearised)
%   State-space: [θ; θ_dot]  — placeholder gains, replace with real aerodynamics
add_block('simulink/Continuous/State-Space',[m6 '/Pitch_SS'],...
   'Position',[120 60 240 140],...
   'A','[0 1; -1 -0.5]',...   % replace with real A matrix
   'B','[0; 1]',...            % replace with real B matrix
   'C','[1 0; 0 1]',...
   'D','[0; 0]',...
   'X0','[0; 0]');

add_line(m6,'delta_e/1','Pitch_SS/1','autorouting','on');
add_line(m6,'Pitch_SS/1','theta_out/1','autorouting','on');
add_line(m6,'Pitch_SS/2','zdot3_out/1','autorouting','on');

close_system([mdl '/Missile_6DOF']);

%% ================================================================
%  SECTION 4 – PITCH ERROR & SIMPLE Kp LOOP (upper middle path)
%% ================================================================

% Pitch_error sum  (+  -)
ab(mdl,'simulink/Math Operations/Sum','Pitch_error_sum',...
   [170 355 200 385],'Inputs','+-');

% Kp gain
ab(mdl,'simulink/Math Operations/Gain','Kp_upper',...
   [220 355 270 385],'Gain',num2str(Kp),'ShowName','on');

% 1/2 (half) gain after Kp
ab(mdl,'simulink/Math Operations/Gain','Half_upper',...
   [285 355 320 385],'Gain','0.5');

% Invert (Q)
ab(mdl,'simulink/Math Operations/Math Function','Invert_mid',...
   [340 350 390 390],'Operator','reciprocal','ShowName','on');

% 1/b0 gain → Elevator_command
ab(mdl,'simulink/Math Operations/Gain','inv_b0_upper',...
   [405 355 455 385],'Gain',num2str(1/b0),'ShowName','on');

% Saturation  –0.4 to 0.4
ab(mdl,'simulink/Discontinuities/Saturation','Saturate_upper',...
   [520 350 580 390],'LowerLimit',num2str(sat_lo),'UpperLimit',num2str(sat_hi));

% Ze‡ (degrs)  Unit Delay – feeds into elevator path
ab(mdl,'simulink/Discrete/Unit Delay','Ze_deg',...
   [405 280 445 310],'SampleTime','0.001');

%% ================================================================
%  SECTION 5 – ADRC PITCH CONTROLLER (lower subsystem)
%% ================================================================
add_block('simulink/Ports & Subsystems/Subsystem',...
   [mdl '/ADRC_Controller'],[30 480 320 580]);
set_param([mdl '/ADRC_Controller'],'ShowName','on');

open_system([mdl '/ADRC_Controller']);
ac = [mdl '/ADRC_Controller'];
delete_line(ac,'In1/1','Out1/1');

% Ports
add_block('simulink/Sources/In1',[ac '/theta_in'],  'Position',[20 50 50 70]);
add_block('simulink/Sources/In1',[ac '/theta_ref_in'],'Position',[20 100 50 120]);
add_block('simulink/Sources/In1',[ac '/z3_in'],     'Position',[20 150 50 170]);
add_block('simulink/Sinks/Out1', [ac '/u_out'],     'Position',[400 100 430 120]);

% Step inside ADRC (mirrors diagram's "Step" → Theta block)
add_block('simulink/Math Operations/Sum',[ac '/e1_sum'],...
   'Position',[80 95 110 125],'Inputs','+-');
add_block('simulink/Math Operations/Gain',[ac '/Kp_adrc'],...
   'Position',[130 95 175 125],'Gain',num2str(Kp));
add_block('simulink/Discrete/Unit Delay',[ac '/e1_delay'],...
   'Position',[80 45 120 65],'SampleTime','0.001');

% C1 sum: e2 path
add_block('simulink/Math Operations/Sum',[ac '/C1_sum'],...
   'Position',[200 90 230 120],'Inputs','+-+');
% e2 gain  (Kp)
add_block('simulink/Math Operations/Gain',[ac '/Kp_e2'],...
   'Position',[200 140 240 165],'Gain',num2str(Kp));
% Kd * z3 disturbance feed
add_block('simulink/Math Operations/Gain',[ac '/Kd_z3'],...
   'Position',[80 145 120 170],'Gain',num2str(Kd));

% 1/b0 → Saturate
add_block('simulink/Math Operations/Gain',[ac '/inv_b0_adrc'],...
   'Position',[255 90 300 120],'Gain',num2str(1/b0));
add_block('simulink/Discontinuities/Saturation',[ac '/Sat_adrc'],...
   'Position',[320 90 370 120],...
   'LowerLimit',num2str(sat_lo),'UpperLimit',num2str(sat_hi));

% Connect inside ADRC
add_line(ac,'theta_ref_in/1','e1_sum/1','autorouting','on');
add_line(ac,'theta_in/1','e1_sum/2','autorouting','on');
add_line(ac,'e1_sum/1','Kp_adrc/1','autorouting','on');
add_line(ac,'Kp_adrc/1','C1_sum/1','autorouting','on');
add_line(ac,'e1_sum/1','e1_delay/1','autorouting','on');
add_line(ac,'e1_delay/1','Kp_e2/1','autorouting','on');
add_line(ac,'Kp_e2/1','C1_sum/2','autorouting','on');
add_line(ac,'z3_in/1','Kd_z3/1','autorouting','on');
add_line(ac,'Kd_z3/1','C1_sum/3','autorouting','on');
add_line(ac,'C1_sum/1','inv_b0_adrc/1','autorouting','on');
add_line(ac,'inv_b0_adrc/1','Sat_adrc/1','autorouting','on');
add_line(ac,'Sat_adrc/1','u_out/1','autorouting','on');

close_system([mdl '/ADRC_Controller']);

%% ================================================================
%  SECTION 6 – EXTENDED STATE OBSERVER (ESO subsystem)
%% ================================================================
add_block('simulink/Ports & Subsystems/Subsystem',...
   [mdl '/ESO'],[540 470 820 580]);
set_param([mdl '/ESO'],'ShowName','on');

open_system([mdl '/ESO']);
eso = [mdl '/ESO'];
delete_line(eso,'In1/1','Out1/1');

% Ports
add_block('simulink/Sources/In1',[eso '/e3_in'], 'Position',[20 80 50 100]);  % error e3 input
add_block('simulink/Sinks/Out1', [eso '/z1_out'],'Position',[380 50 410 70]);
add_block('simulink/Sinks/Out1', [eso '/z2_out'],'Position',[380 90 410 110]);  % theta_hat
add_block('simulink/Sinks/Out1', [eso '/z3_out'],'Position',[380 130 410 150]); % disturbance

% ESO equations (discrete):
%   z1(k+1) = z1 + Ts*(z2 - beta1*e)
%   z2(k+1) = z2 + Ts*(z3 - beta2*e + b0*u)
%   z3(k+1) = z3 + Ts*(-beta3*e)
%   e = z1 - y  (y is the output, fed in as e3_in already processed)

% Use Discrete State-Space to implement ESO compactly
% State vector: [z1; z2; z3]
Ts = 0.001;
A_eso = [1-beta1*Ts,  Ts,      0;
         -beta2*Ts,   1,       Ts;
         -beta3*Ts,   0,       1];
B_eso = [0; b0*Ts; 0];   % u input (not connected here – simplified)
C_eso = eye(3);
D_eso = zeros(3,1);

A_str = mat2str(A_eso);
B_str = mat2str(B_eso);
C_str = mat2str(C_eso);
D_str = mat2str(D_eso);

add_block('simulink/Discrete/Discrete State-Space',[eso '/ESO_SS'],...
   'Position',[120 70 280 160],...
   'A',A_str,'B',B_str,'C',C_str,'D',D_str,...
   'X0','[0;0;0]','SampleTime',num2str(Ts));

% Connect
add_line(eso,'e3_in/1','ESO_SS/1','autorouting','on');
add_line(eso,'ESO_SS/1','z1_out/1','autorouting','on');
add_line(eso,'ESO_SS/2','z2_out/1','autorouting','on');
add_line(eso,'ESO_SS/3','z3_out/1','autorouting','on');

% Labels
add_block('simulink/Signal Attributes/Signal Specification',...
   [eso '/label_z2'],'Position',[300 85 370 105]);
set_param([eso '/label_z2'],'ShowName','on');
add_line(eso,'ESO_SS/2','label_z2/1','autorouting','on');

close_system([mdl '/ESO']);

%% ================================================================
%  SECTION 7 – TOP-LEVEL CONNECTIONS
%% ================================================================

% Theta_ref → Pitch_error_sum port 1
add_line(mdl,'Theta_ref/1','Pitch_error_sum/1','autorouting','on');

% Missile_6DOF theta → Pitch_error_sum port 2
add_line(mdl,'Missile_6DOF/1','Pitch_error_sum/2','autorouting','on');

% Pitch_error_sum → Kp_upper
add_line(mdl,'Pitch_error_sum/1','Kp_upper/1','autorouting','on');

% Kp_upper → Half_upper
add_line(mdl,'Kp_upper/1','Half_upper/1','autorouting','on');

% Half_upper → Invert_mid
add_line(mdl,'Half_upper/1','Invert_mid/1','autorouting','on');

% Invert_mid → inv_b0_upper
add_line(mdl,'Invert_mid/1','inv_b0_upper/1','autorouting','on');

% inv_b0_upper → Saturate_upper port 1 (sum with z3 first)
% (Saturate combines the elevator command + z3 disturbance rejection)
add_block('simulink/Math Operations/Sum',[mdl '/elev_sum'],...
   'Position',[480 350 510 380],'Inputs','+-');
add_line(mdl,'inv_b0_upper/1','elev_sum/1','autorouting','on');
% z3 from ESO → elev_sum port 2
add_line(mdl,'ESO/3','elev_sum/2','autorouting','on');
add_line(mdl,'elev_sum/1','Saturate_upper/1','autorouting','on');

% Saturate_upper → Missile_6DOF elevator input
add_line(mdl,'Saturate_upper/1','Missile_6DOF/1','autorouting','on');

% ADRC controller outputs → also feed Missile 6DOF (parallel path)
% Theta_ref → ADRC port 2
add_line(mdl,'Theta_ref/1','ADRC_Controller/2','autorouting','on');
% Missile theta → ADRC port 1
add_line(mdl,'Missile_6DOF/1','ADRC_Controller/1','autorouting','on');
% ESO z3 → ADRC port 3
add_line(mdl,'ESO/3','ADRC_Controller/3','autorouting','on');

% Missile zdot3 → ESO e3 input
add_line(mdl,'Missile_6DOF/2','ESO/1','autorouting','on');

% ADRC out → also connected back to upper Saturate (as second elevator source)
% In the diagram both paths converge; wire ADRC to elev_sum through 1/b0 already done above.

%% ================================================================
%  SECTION 8 – SCOPES / LOGGING
%% ================================================================
add_block('simulink/Sinks/Scope',[mdl '/Scope_Theta'],...
   'Position',[950 250 1000 290],'NumInputPorts','2');
set_param([mdl '/Scope_Theta'],'ShowName','on');
add_line(mdl,'Missile_6DOF/1','Scope_Theta/1','autorouting','on');
add_line(mdl,'Theta_ref/1','Scope_Theta/2','autorouting','on');

add_block('simulink/Sinks/Scope',[mdl '/Scope_ESO'],...
   'Position',[950 310 1000 350],'NumInputPorts','3');
set_param([mdl '/Scope_ESO'],'ShowName','on');
add_line(mdl,'ESO/1','Scope_ESO/1','autorouting','on');
add_line(mdl,'ESO/2','Scope_ESO/2','autorouting','on');
add_line(mdl,'ESO/3','Scope_ESO/3','autorouting','on');

%% ================================================================
%  SECTION 9 – ARRANGE & SAVE
%% ================================================================
Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl);
fprintf('\n✅  Model "%s.slx" saved successfully.\n', mdl);
fprintf('   Open it with:  open_system(''%s'')\n\n', mdl);
fprintf('⚠️  IMPORTANT NEXT STEPS:\n');
fprintf('   1. Replace the State-Space A,B matrices in "Missile_6DOF"\n');
fprintf('      with your real aerodynamic coefficients.\n');
fprintf('   2. Tune Kp, b0, beta1/2/3, Kd in the PARAMETERS section.\n');
fprintf('   3. Verify signal dimensions (especially the 6DOF state vector).\n');
fprintf('   4. Run simulation and check Scope_Theta for tracking.\n');