clc;
clear;
close all;

model = 'missile_ADRC_complete';

new_system(model)
open_system(model)
set_param(model,'StopTime','40')

%% =========================
% Theta Reference
%% =========================

add_block('simulink/Sources/Step',...
    [model '/Theta_ref'],...
    'Position',[30 120 60 150],...
    'After','5*pi/180');

%% =========================
% ADRC Controller
%% =========================

add_block('simulink/User-Defined Functions/MATLAB Function',...
    [model '/ADRC'],...
    'Position',[120 100 240 170]);

set_param([model '/ADRC'],'Script',sprintf([...
'function delta_e = ADRC(theta_ref,z1,z2,z3)\n' ...
'wc=8;kp=wc^2;kd=2*wc;\n' ...
'rho=1.225;U0=300;S=0.5;c=0.5;Iy=20;Cm_de=1.2;\n' ...
'b0=Cm_de*0.5*rho*U0^2*S*c/Iy;\n' ...
'e1=theta_ref-z1;\n' ...
'e2=-z2;\n' ...
'u=kp*e1+kd*e2;\n' ...
'delta_e=(u-z3)/b0;\n' ...
'if delta_e>0.4,delta_e=0.4;end\n' ...
'if delta_e<-0.4,delta_e=-0.4;end\n' ...
'end']));

%% =========================
% Missile EOM
%% =========================

add_block('simulink/User-Defined Functions/MATLAB Function',...
    [model '/Missile_EOM'],...
    'Position',[350 60 550 220]);

set_param([model '/Missile_EOM'],'Script','disp(''Insert missile code here'')');

%% =========================
% ESO
%% =========================

add_block('simulink/User-Defined Functions/MATLAB Function',...
    [model '/ESO'],...
    'Position',[650 100 800 180]);

set_param([model '/ESO'],'Script',sprintf([...
'function [z1dot,z2dot,z3dot]=ESO(theta,delta_e,z1,z2,z3)\n' ...
'wo=50;beta1=3*wo;beta2=3*wo^2;beta3=wo^3;\n' ...
'rho=1.225;U0=300;S=0.5;c=0.5;Iy=20;Cm_de=1.2;\n' ...
'b0=Cm_de*0.5*rho*U0^2*S*c/Iy;\n' ...
'e=z1-theta;\n' ...
'z1dot=z2-beta1*e;\n' ...
'z2dot=z3-beta2*e+b0*delta_e;\n' ...
'z3dot=-beta3*e;\n' ...
'end']));

%% =========================
% State Integrators
%% =========================

states={'U','V','W','P','Q','R','phi','theta','psi','X','Y','Z'};

x=350;
y=260;

for i=1:12

    add_block('simulink/Continuous/Integrator',...
        [model '/' states{i}],...
        'Position',[x y x+30 y+30]);

    set_param([model '/' states{i}],'InitialCondition','0');

    y=y+45;

end

%% Initial conditions

set_param([model '/U'],'InitialCondition','300');
set_param([model '/Z'],'InitialCondition','-2000');

%% =========================
% ESO Integrators
%% =========================

add_block('simulink/Continuous/Integrator',[model '/z1'],...
    'Position',[850 100 880 130]);

add_block('simulink/Continuous/Integrator',[model '/z2'],...
    'Position',[850 140 880 170]);

add_block('simulink/Continuous/Integrator',[model '/z3'],...
    'Position',[850 180 880 210]);

%% Scope

add_block('simulink/Sinks/Scope',...
    [model '/Scope'],...
    'Position',[950 130 980 160]);

%% =========================
% Connections
%% =========================

add_line(model,'Theta_ref/1','ADRC/1')

add_line(model,'z1/1','ADRC/2')
add_line(model,'z2/1','ADRC/3')
add_line(model,'z3/1','ADRC/4')

add_line(model,'ADRC/1','Missile_EOM/1')

add_line(model,'Missile_EOM/13','ESO/1')
add_line(model,'ADRC/1','ESO/2')

add_line(model,'ESO/1','z1/1')
add_line(model,'ESO/2','z2/1')
add_line(model,'ESO/3','z3/1')

add_line(model,'Missile_EOM/13','Scope/1')

save_system(model)
open_system(model)

disp('Complete model created successfully')