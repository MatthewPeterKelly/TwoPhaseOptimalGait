function output = Continuous_Run(input)

%Check MAIN_Walk for index definitions throughout this file

n1 = 0;  % surface normal = angle from vertical

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 1  --  Flight                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;

IdxState = 1:12;
IdxAct = 13:15;
IdxActRate = 1:3;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).state(:,IdxAct);
Actuator_Rate = input.phase(iphase).control(:,IdxActRate);
P_Dyn = input.auxdata.dynamics;

dStates = dynamics_flight(States, Actuators, P_Dyn);
output(iphase).dynamics = [dStates, Actuator_Rate];

IdxAbsPos = 4:6;
IdxAbsNeg = 7:9;
AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

output(iphase).path = zeros(size(dStates,1),5);
Position = getPosVel_flight(States);

%%%% HACK %%%% January 21, 2014
% The clearance constraint is not enforced in flight, only during single
% stance. The should be fine for low speed running, but for high-speed
% running this will effectivly remove that constraint.
STEP_DIST = input.phase(iphase).parameter;
y1 = 0;  % ground height
output(iphase).path(:,1) = ... %Foot height above ground
    Position.footOne.y - y1;
y2 = 0;  % ground height
output(iphase).path(:,2) = ... %Foot height above ground
    Position.footTwo.y - y2;
%
%%%% DONE %%%%

P_Cost = input.auxdata.cost;
[cost, path] = costFunc(States, Actuators, Actuator_Rate,...
    P_Dyn, P_Cost, AbsPos, AbsNeg, 'F', STEP_DIST);
output(iphase).path(:,3:5) = path;
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

output(iphase).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
output(iphase).path(:,1) = atan2(H1,V1) + n1;

Position = getPosVel_single(States);
ground_height = 0;
STEP_DIST = input.phase(iphase).parameter;
xBnd = STEP_DIST*[-1,1];
hClr = input.auxdata.ground.clearance;
clearance = footClearance(Position.footTwo.x,xBnd,hClr);
output(iphase).path(:,2) = ... %Foot height above ground
    Position.footTwo.y - (ground_height+clearance);

IdxAbsPos = 5:8;
IdxAbsNeg = 9:12;
AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

[cost, path] = costFunc(States, Actuators, Actuator_Rate,...
    P_Dyn, P_Cost, AbsPos, AbsNeg, 'S', STEP_DIST);
output(iphase).path(:,3:6) = path;
output(iphase).integrand = cost;

end