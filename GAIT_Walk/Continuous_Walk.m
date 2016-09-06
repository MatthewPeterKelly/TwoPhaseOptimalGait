function output = Continuous_Walk(input)

%Check MAIN_Walk for index definitions throughout this file

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 1  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;

IdxState = 1:4;
IdxAct = 5:6;
IdxActRate = 1:2;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).state(:,IdxAct);
Actuator_Rate = input.phase(iphase).control(:,IdxActRate);
P_Dyn = input.auxdata.dynamics;

STEP_DIST = input.phase(iphase).parameter;
P_Dyn.x2 = -STEP_DIST;
P_Dyn.y2 = 0;

[dStates, contactForces] = dynamics_double(States, Actuators, P_Dyn);
output(iphase).dynamics = [dStates, Actuator_Rate];

output(iphase).path = zeros(size(dStates,1),6);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
H2 = contactForces(:,3);
V2 = contactForces(:,4);
u = P_Dyn.u;
output(iphase).path(:,1) = H1 + u*V1;
output(iphase).path(:,2) = u*V1 - H1;
output(iphase).path(:,3) = H2 + u*V2;
output(iphase).path(:,4) = u*V2 - H2;

IdxAbsPos = 3:4;
IdxAbsNeg = 5:6;
AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

P_Cost = input.auxdata.cost;
[cost, path] = costFunc(States, Actuators, Actuator_Rate,...
    P_Dyn, P_Cost, AbsPos, AbsNeg, 'D', STEP_DIST);
output(iphase).path(:,5:6) = path;
output(iphase).integrand = cost;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 2  --  Single Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2;

IdxState = 1:8;
IdxAct = 9:12;
IdxActRate = 1:4;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).state(:,IdxAct);
Actuator_Rate = input.phase(iphase).control(:,IdxActRate);

[dStates, contactForces] = dynamics_single(States, Actuators, P_Dyn);
output(iphase).dynamics = [dStates, Actuator_Rate];

output(iphase).path = zeros(size(dStates,1),7);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
output(iphase).path(:,1) = H1 + u*V1;
output(iphase).path(:,2) = u*V1 - H1;

%Foot height above ground
Position = getPosVel_single(States);
STEP_DIST = input.phase(iphase).parameter;
Bnd = [-STEP_DIST,STEP_DIST];
Clr = footClearance(Position.footTwo.x,Bnd,input.auxdata.footClearance);
output(iphase).path(:,3) =  Position.footTwo.y - Clr;

IdxAbsPos = 5:8;
IdxAbsNeg = 9:12;
AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

STEP_DIST = input.phase(iphase).parameter;
[cost, path] = costFunc(States, Actuators, Actuator_Rate,...
    P_Dyn, P_Cost, AbsPos, AbsNeg, 'S', STEP_DIST);
output(iphase).path(:,4:7) = path;
output(iphase).integrand = cost;

end