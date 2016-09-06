%---------------------------------------------------%
% Running Test Gait                                 %
%---------------------------------------------------%
%MAIN_Run
clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

%%%% Physical parameters %%%%
INPUT.physical.leg_length = [0.6; 1.0];
INPUT.physical.total_mass = 8;  %(kg) total robot mass
INPUT.physical.hip_mass_fraction = 0.9;
INPUT.physical.gravity = 9.81;
INPUT.physical.coeff_friction = 0.8;
INPUT.physical.actuator_leg_saturate = 4;   % (_)*(Mass*Gravity)
INPUT.physical.actuator_hip_saturate = 0.5; % (_)*(Length*Mass*Gravity)
INPUT.physical.actuator_ank_saturate = 0.05; % (_)*(Length*Mass*Gravity)

%%%% Constraints %%%%
INPUT.constraint.duration_single_stance = [0.02; 2];
INPUT.constraint.duration_flight = [0.02; 2];
INPUT.constraint.speed = [1.2; 2.0];
INPUT.constraint.step_distance = [0.05; 1.0];
INPUT.constraint.center_clearance = 0.02;

%%%% Optimization %%%%
INPUT.optimize.solver = 'ipopt';   %{'ipopt', 'snopt'}
INPUT.optimize.tol_mesh = 1e-3;
INPUT.optimize.tol_opt = 1e-6;
INPUT.optimize.max_mesh_iter = 5;

%%%% Cost Function %%%%
INPUT.cost.actuator_weight = 1e-12;   %1e-3
INPUT.cost.actuator_rate_weight = 1e-12;   %1e-3
INPUT.cost.method = 'CoT';  %{'Work','CoT'}
INPUT.cost.actuator_leg_scale = 1 /INPUT.physical.actuator_leg_saturate;
INPUT.cost.actuator_hip_scale = 1 /INPUT.physical.actuator_hip_saturate;
INPUT.cost.actuator_ank_scale = 1 /INPUT.physical.actuator_ank_saturate; 

%%%% Input / Output parameters %%%%
INPUT.io.loadPrevSoln = true;
INPUT.io.saveSolution = false;
INPUT.io.createPlots = true;
INPUT.io.runAnimation = true;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Run Trajectory Optimization:                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

[output, plotInfo] = Trajectory_Run(INPUT);

P_Dyn = output.result.setup.auxdata.dynamics;
save('DATA_Run.mat','plotInfo','P_Dyn');
