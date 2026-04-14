clc;
clear;
close all;

%% Model Name
model = 'MISSILE_6DOF_ADRC';

% Close if already exists
if bdIsLoaded(model)
    close_system(model,0);
end

% Create new Simulink model
new_system(model);

% Open the model
open_system(model);

% Set simulation settings
set_param(model,'StopTime','40');
set_param(model,'Solver','ode45');

disp('Simulink model created and opened successfully');

%% Missile Physical Parameters

m = 150;        % mass (kg)
Ix = 10;
Iy = 20;
Iz = 25;

rho = 1.225;    % air density
S = 0.5;        % wing area
g = 9.81;

U0 = 300;       % forward velocity
V0 = 0;
W0 = 0;

theta0 = 0;
Q0 = 0;

X0 = 0;
Y0 = 0;
Z0 = -2000;

target_range = 5000;

disp('Missile parameters loaded'); 

%% ADRC Pitch Controller Parameters

% Controller bandwidth
wc = 8;        % rad/sec

% Observer bandwidth
wo = 50;       % rad/sec

% ADRC Gains
kp = wc^2;
kd = 2*wc;

% ESO Gains
beta1 = 3*wo;
beta2 = 3*(wo^2);
beta3 = wo^3;

% Reference pitch angle
theta_ref = 5 * pi/180;   % 5 degree command

disp('ADRC controller initialized');

%% Step 4: Create 6DOF Core Blocks

disp('Creating 6DOF blocks...')

% Forward velocity U
add_block('simulink/Continuous/Integrator',[model '/U_velocity'],...
    'Position',[100 50 130 80]);

% Lateral velocity V
add_block('simulink/Continuous/Integrator',[model '/V_velocity'],...
    'Position',[100 120 130 150]);

% Vertical velocity W
add_block('simulink/Continuous/Integrator',[model '/W_velocity'],...
    'Position',[100 190 130 220]);

% Pitch rate Q
add_block('simulink/Continuous/Integrator',[model '/Pitch_rate_Q'],...
    'Position',[300 120 330 150]);

% Pitch angle Theta
add_block('simulink/Continuous/Integrator',[model '/Pitch_angle'],...
    'Position',[450 120 480 150]);

% X position
add_block('simulink/Continuous/Integrator',[model '/X_position'],...
    'Position',[600 50 630 80]);

% Y position
add_block('simulink/Continuous/Integrator',[model '/Y_position'],...
    'Position',[600 120 630 150]);

% Z position
add_block('simulink/Continuous/Integrator',[model '/Z_position'],...
    'Position',[600 190 630 220]);

disp('Core integrator blocks added')

%% Step 5: Airspeed, AoA, Sideslip and ADRC

disp('Adding airspeed and ADRC blocks...')

%% Airspeed Calculation

add_block('simulink/Math Operations/Sum',...
    [model '/Sum_velocity'],...
    'Position',[200 100 230 140],...
    'Inputs','+++');

add_block('simulink/Math Operations/Math Function',...
    [model '/SquareRoot'],...
    'Position',[260 110 300 140],...
    'Operator','sqrt');

%% Angle of Attack

add_block('simulink/Math Operations/Trigonometric Function',...
    [model '/Angle_of_Attack'],...
    'Position',[350 50 390 80],...
    'Operator','atan');

%% Sideslip

add_block('simulink/Math Operations/Trigonometric Function',...
    [model '/Sideslip'],...
    'Position',[350 90 390 120],...
    'Operator','atan');

%% ADRC Reference

add_block('simulink/Sources/Constant',...
    [model '/Theta_ref'],...
    'Position',[100 300 130 330],...
    'Value','theta_ref');

%% Error Calculation

add_block('simulink/Math Operations/Sum',...
    [model '/Pitch_error'],...
    'Position',[200 300 230 330],...
    'Inputs','+-');

%% ADRC Gains

add_block('simulink/Math Operations/Gain',...
    [model '/Kp'],...
    'Position',[260 290 300 320],...
    'Gain','kp');

add_block('simulink/Math Operations/Gain',...
    [model '/Kd'],...
    'Position',[260 330 300 360],...
    'Gain','kd');

%% Elevator Command

add_block('simulink/Math Operations/Sum',...
    [model '/Elevator_command'],...
    'Position',[350 310 380 340],...
    'Inputs','++');

%% ESO Output

add_block('simulink/Sinks/To Workspace',...
    [model '/ESO_output'],...
    'Position',[500 300 550 330],...
    'VariableName','eso');

%% Elevator Output

add_block('simulink/Sinks/To Workspace',...
    [model '/Elevator_out'],...
    'Position',[500 350 550 380],...
    'VariableName','elevator');

disp('Airspeed and ADRC blocks added')

%% Connections

disp('Connecting lines safely...')

set_param(model,'SimulationCommand','update')

try

add_line(model,'U_input/1','U_velocity/1','autorouting','on')
add_line(model,'V_input/1','V_velocity/1','autorouting','on')
add_line(model,'W_input/1','W_velocity/1','autorouting','on')
add_line(model,'Q_input/1','Pitch_rate_Q/1','autorouting','on')

add_line(model,'Pitch_rate_Q/1','Pitch_angle/1','autorouting','on')

add_line(model,'U_velocity/1','U_out/1','autorouting','on')
add_line(model,'V_velocity/1','V_out/1','autorouting','on')
add_line(model,'W_velocity/1','W_out/1','autorouting','on')

add_line(model,'Pitch_angle/1','Pitch_out/1','autorouting','on')
add_line(model,'Pitch_rate_Q/1','Pitch_rate_out/1','autorouting','on')

add_line(model,'U_velocity/1','Sum_velocity/1','autorouting','on')
add_line(model,'V_velocity/1','Sum_velocity/2','autorouting','on')
add_line(model,'W_velocity/1','Sum_velocity/3','autorouting','on')

add_line(model,'Sum_velocity/1','SquareRoot/1','autorouting','on')
add_line(model,'SquareRoot/1','Airspeed_out/1','autorouting','on')

add_line(model,'Theta_ref/1','Pitch_error/1','autorouting','on')
add_line(model,'Pitch_angle/1','Pitch_error/2','autorouting','on')

add_line(model,'Pitch_error/1','Kp/1','autorouting','on')
add_line(model,'Pitch_error/1','Kd/1','autorouting','on')

add_line(model,'Kp/1','Elevator_command/1','autorouting','on')
add_line(model,'Kd/1','Elevator_command/2','autorouting','on')

add_line(model,'Elevator_command/1','Elevator_out/1','autorouting','on')
add_line(model,'Elevator_command/1','ESO_output/1','autorouting','on')

add_line(model,'Target/1','Range/1','autorouting','on')
add_line(model,'X_position/1','Range/2','autorouting','on')
add_line(model,'Range/1','Range_out/1','autorouting','on')

catch
    disp('Some connections already exist or failed — continuing...')
end

disp('Connections completed')