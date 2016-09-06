%---------------------------------------------------%
% Walking Test Gait                                 %
%---------------------------------------------------%
%MAIN_Walk
clc; clear; addpath ../computerGeneratedCode; addpath ../shared;

%%%% Physical parameters %%%%
INPUT.physical.leg_length = [0.7; 0.9];
INPUT.physical.total_mass = 8;  %(kg) total robot mass
INPUT.physical.hip_mass_fraction = 0.85;
INPUT.physical.gravity = 9.81;
INPUT.physical.coeff_friction = 0.8;
INPUT.physical.actuator_leg_saturate = 4;   % (_)*(Mass*Gravity)
INPUT.physical.actuator_hip_saturate = 0.5; % (_)*(Length*Mass*Gravity)
INPUT.physical.actuator_ank_saturate = 0.5; % (_)*(Length*Mass*Gravity)

%%%% Constraints %%%%
INPUT.constraint.duration_single_stance = [0.05; 2];
INPUT.constraint.duration_double_stance = [0.05; 2];
INPUT.constraint.speed = [0.2;2.5];
INPUT.constraint.step_distance = [0.2; 0.7];
INPUT.constraint.clearance = 0.00;  %Swing foot clearance

%%%% Optimization %%%%
INPUT.optimize.solver = 'ipopt';   %{'ipopt', 'snopt'}
INPUT.optimize.tol_mesh = 1e-2;
INPUT.optimize.tol_opt = 1e-3;
INPUT.optimize.max_mesh_iter = 2;
INPUT.optimize.max_iter = 100;

%%%% Cost Function %%%%
INPUT.cost.actuator_weight = 1e-12; 
INPUT.cost.actuator_rate_weight = 1e-12; 
INPUT.cost.method = 'CoT';  %{'Work','CoT'}
INPUT.cost.actuator_leg_scale = 1 /INPUT.physical.actuator_leg_saturate;
INPUT.cost.actuator_hip_scale = 1 /INPUT.physical.actuator_hip_saturate;
INPUT.cost.actuator_ank_scale = 1 /INPUT.physical.actuator_ank_saturate; 

% These should all be == 1 for CoT to make any sense!
INPUT.cost.weight.ankle = 100;
INPUT.cost.weight.hip = 1;
INPUT.cost.weight.legOne = 1;
INPUT.cost.weight.legTwo = 1;

%%%% Input / Output parameters %%%%
INPUT.io.loadPrevSoln = true;
INPUT.io.saveSolution = false;
INPUT.io.createPlots = true;
INPUT.io.runAnimation = true;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Run Trajectory Optimization:                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

[output, plotInfo] = Trajectory_Walk(INPUT);

P_Dyn = output.result.setup.auxdata.dynamics;
save('DATA_Walk.mat','plotInfo','P_Dyn');
