clc;
clear;
close all;

model = 'MISSILE_SIM_ADRC';

% Create model
new_system(model);
open_system(model);

%% TARGET

add_block('simulink/Sources/Constant',[model '/Target_Position'],...
    'Position',[30 50 80 80]);
set_param([model '/Target_Position'],'Value','20000');

add_block('simulink/Continuous/Integrator',[model '/Target_Velocity'],...
    'Position',[120 50 170 80]);

%% MISSILE

add_block('simulink/Sources/Constant',[model '/Missile_Input'],...
    'Position',[30 200 80 230]);
set_param([model '/Missile_Input'],'Value','1');

add_block('simulink/Continuous/Integrator',[model '/Missile_Velocity'],...
    'Position',[120 200 170 230]);

add_block('simulink/Continuous/Integrator',[model '/Missile_Position'],...
    'Position',[200 200 250 230]);

%% ERROR

add_block('simulink/Math Operations/Sum',[model '/Error'],...
    'Position',[300 120 330 150]);

set_param([model '/Error'],'Inputs','+-');

%% TRACKING DIFFERENTIATOR

add_block('simulink/Continuous/Transfer Fcn',[model '/TD'],...
    'Position',[380 120 450 150]);

set_param([model '/TD'],...
    'Numerator','[1]',...
    'Denominator','[1 1]');

%% ESO

add_block('simulink/Math Operations/Gain',[model '/ESO_Z1'],...
    'Position',[500 100 550 130]);
set_param([model '/ESO_Z1'],'Gain','4.5');

add_block('simulink/Math Operations/Gain',[model '/ESO_Z2'],...
    'Position',[500 140 550 170]);
set_param([model '/ESO_Z2'],'Gain','14.3');

add_block('simulink/Math Operations/Gain',[model '/ESO_Z3'],...
    'Position',[500 180 550 210]);
set_param([model '/ESO_Z3'],'Gain','-0.37');

%% CONTROL LAW

add_block('simulink/Math Operations/Sum',[model '/Control_Sum'],...
    'Position',[600 130 630 160]);

set_param([model '/Control_Sum'],'Inputs','+++');

%% CONTROL OUTPUT

add_block('simulink/Sinks/Scope',[model '/Scope'],...
    'Position',[700 130 750 160]);

%% CONNECTIONS

add_line(model,'Target_Position/1','Target_Velocity/1');

add_line(model,'Missile_Input/1','Missile_Velocity/1');
add_line(model,'Missile_Velocity/1','Missile_Position/1');

add_line(model,'Target_Velocity/1','Error/1');
add_line(model,'Missile_Position/1','Error/2');

add_line(model,'Error/1','TD/1');

add_line(model,'TD/1','ESO_Z1/1');
add_line(model,'TD/1','ESO_Z2/1');
add_line(model,'TD/1','ESO_Z3/1');

add_line(model,'ESO_Z1/1','Control_Sum/1');
add_line(model,'ESO_Z2/1','Control_Sum/2');
add_line(model,'ESO_Z3/1','Control_Sum/3');

add_line(model,'Control_Sum/1','Scope/1');

%% SAVE

save_system(model);

disp('MISSILE_SIM_ADRC Model Created Successfully');