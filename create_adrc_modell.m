clc
clear
close all

model = 'ADRC_Missile_Pitch';
new_system(model)
open_system(model)

%% =========================
% STEP INPUT (Theta_ref)
%% =========================
add_block('simulink/Sources/Step',[model '/Theta_ref'],...
    'Position',[30 100 60 130])

%% =========================
% SUM (Pitch error)
%% =========================
add_block('simulink/Math Operations/Sum',...
    [model '/Pitch_error'],...
    'Inputs','+-',...
    'Position',[120 100 140 130])

%% =========================
% KP
%% =========================
add_block('simulink/Math Operations/Gain',...
    [model '/Kp'],...
    'Gain','kp',...
    'Position',[180 100 220 130])

%% =========================
% 1/b0
%% =========================
add_block('simulink/Math Operations/Gain',...
    [model '/1_b0'],...
    'Gain','1/b0',...
    'Position',[260 100 300 130])

%% =========================
% Saturation
%% =========================
add_block('simulink/Discontinuities/Saturation',...
    [model '/Saturation'],...
    'UpperLimit','0.4',...
    'LowerLimit','-0.4',...
    'Position',[330 100 360 130])

%% =========================
% Missile Dynamics (Subsystem)
%% =========================
add_block('simulink/Ports & Subsystems/Subsystem',...
    [model '/Missile_6DOF_Dynamics'],...
    'Position',[420 60 550 180])

open_system([model '/Missile_6DOF_Dynamics'])

add_block('simulink/Sources/In1',...
    [model '/Missile_6DOF_Dynamics/Input'],...
    'Position',[30 60 60 80])

add_block('simulink/Continuous/Integrator',...
    [model '/Missile_6DOF_Dynamics/z2'],...
    'Position',[100 60 130 90])

add_block('simulink/Continuous/Integrator',...
    [model '/Missile_6DOF_Dynamics/z3'],...
    'Position',[160 60 190 90])

add_block('simulink/Sinks/Out1',...
    [model '/Missile_6DOF_Dynamics/Theta'],...
    'Position',[230 60 260 80])

add_line([model '/Missile_6DOF_Dynamics'],'Input/1','z2/1')
add_line([model '/Missile_6DOF_Dynamics'],'z2/1','z3/1')
add_line([model '/Missile_6DOF_Dynamics'],'z3/1','Theta/1')

close_system([model '/Missile_6DOF_Dynamics'])

%% =========================
% ESO Subsystem
%% =========================
add_block('simulink/Ports & Subsystems/Subsystem',...
    [model '/ESO'],...
    'Position',[420 220 550 340])

open_system([model '/ESO'])

add_block('simulink/Sources/In1',...
    [model '/ESO/e3'],...
    'Position',[30 60 60 80])

add_block('simulink/Continuous/Integrator',...
    [model '/ESO/z1'],...
    'Position',[100 40 130 70])

add_block('simulink/Continuous/Integrator',...
    [model '/ESO/z2'],...
    'Position',[150 40 180 70])

add_block('simulink/Continuous/Integrator',...
    [model '/ESO/z3'],...
    'Position',[200 40 230 70])

add_block('simulink/Sinks/Out1',...
    [model '/ESO/Disturbance'],...
    'Position',[260 40 290 60])

add_line([model '/ESO'],'e3/1','z1/1')
add_line([model '/ESO'],'z1/1','z2/1')
add_line([model '/ESO'],'z2/1','z3/1')
add_line([model '/ESO'],'z3/1','Disturbance/1')

close_system([model '/ESO'])

%% =========================
% Connections
%% =========================

add_line(model,'Theta_ref/1','Pitch_error/1')
add_line(model,'Pitch_error/1','Kp/1')
add_line(model,'Kp/1','1_b0/1')
add_line(model,'1_b0/1','Saturation/1')
add_line(model,'Saturation/1','Missile_6DOF_Dynamics/1')

%% Feedback
add_line(model,'Missile_6DOF_Dynamics/1','Pitch_error/2')

%% ESO connection
add_line(model,'Saturation/1','ESO/1')

save_system(model)

disp('ADRC Model Created Successfully')