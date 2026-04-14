clc;
close all;
clearvars;   % better than clear all

% S.I units
d2r = pi/180;

% TARGET
V_T  = 600;
X_T0 = 100;
Z_T0 = 20000;
L_T0 = -2.5*d2r;   % small 'l'

% MISSILE
X_M0 = 2000;
Z_M0 = 0;
L_M0 = 80*d2r;

% Control loop gains
Kdc = 1.1;
Ka  = 4.5;
Ki  = 14.3;
Kr  = -0.37;

% file opening and operation
modelName = 'MISSILE_SIM';

if exist([modelName '.slx'], 'file')
    open_system(modelName);
    sim(modelName);
else
    error('Simulink model MISSILE_SIM.slx not found in current folder');
end