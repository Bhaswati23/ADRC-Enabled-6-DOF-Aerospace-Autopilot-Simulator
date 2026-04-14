clc;
clear;
close all;

model = 'MISSILE_SIM_ADRC';

% Create new model
new_system(model);
open_system(model);

%% TARGET BLOCK
add_block('simulink/Sources/Constant',[model '/Target_Velocity']);
set_param([model '/Target_Velocity'],'Value','600');

add_block('simulink/Continuous/Integrator',[model '/Target_Position']);

%% MISSILE BLOCK
add_block('simulink/Sources/Constant',[model '/Missile_Input']);
set_param([model '/Missile_Input'],'Value','1');

add_block('simulink/Continuous/Integrator',[model '/Missile_Velocity']);
add_block('simulink/Continuous/Integrator',[model '/Missile_Position']);

%% ADRC CONTROLLER

% Tracking Differentiator
add_block('simulink/Continuous/Transfer Fcn',[model '/Tracking_Differentiator']);
set_param([model '/Tracking_Differentiator'], ...
    'Numerator','[1]', ...
    'Denominator','[1 1]');

% ESO
add_block('simulink/Math Operations/Gain',[model '/ESO_Gain']);
set_param([model '/ESO_Gain'],'Gain','4.5');

% Control Gain
add_block('simulink/Math Operations/Gain',[model '/Control_Gain']);
set_param([model '/Control_Gain'],'Gain','14.3');

%% SUM BLOCK
add_block('simulink/Math Operations/Sum',[model '/Error']);
set_param([model '/Error'],'Inputs','+-');

%% SCOPE
add_block('simulink/Sinks/Scope',[model '/Scope']);

%% CONNECTIONS

add_line(model,'Target_Velocity/1','Target_Position/1');

add_line(model,'Missile_Input/1','Missile_Velocity/1');
add_line(model,'Missile_Velocity/1','Missile_Position/1');

add_line(model,'Target_Position/1','Error/1');
add_line(model,'Missile_Position/1','Error/2');

add_line(model,'Error/1','Tracking_Differentiator/1');
add_line(model,'Tracking_Differentiator/1','ESO_Gain/1');
add_line(model,'ESO_Gain/1','Control_Gain/1');
add_line(model,'Control_Gain/1','Scope/1');

%% SAVE MODEL
save_system(model);

disp('ADRC Missile Simulink Model Created Successfully')