function output = Endpoint_Run(input)

STEP_DIST = input.parameter;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Objective Function                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
switch input.auxdata.cost.method
    case 'Work'
        output.objective = input.phase(1).integral + input.phase(2).integral;
    case 'CoT'
        Dyn = input.auxdata.dynamics;
        weight = (Dyn.M + 2*Dyn.m)*Dyn.g;
        output.objective = ...
            (input.phase(1).integral + input.phase(2).integral)/weight;
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Discrete Constraints                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

START = 1; END = 2;
FLIGHT = 1; SINGLE = 2;

States_Flight = [input.phase(FLIGHT).initialstate(:,1:12);...
    input.phase(FLIGHT).finalstate(:,1:12)];
[Pos_F, Vel_F] = getPosVel_flight(States_Flight);

Control_Single = input.phase(SINGLE).finalstate(:,9:12);
States_Single = [input.phase(SINGLE).initialstate(:,1:8);...
    input.phase(SINGLE).finalstate(:,1:8)];
[Pos_S, Vel_S] = getPosVel_single(States_Single);

%Get contact forces at toe-off
P_Dyn = input.auxdata.dynamics;
[~,contactForces] = dynamics_single(States_Single(END,:),Control_Single,P_Dyn);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
n1 = 0;   %angle between normal vector and vertical

Step_Vec = zeros(1,2);
Step_Vec(1) = STEP_DIST;   %Horizontal Component
Step_Vec(2) = 0; %Foot Two @ Start of Flight

Duration = input.phase(1).finaltime + input.phase(2).finaltime;

%%%% Heel-Strike Defect in Hip
output.eventgroup(1).event = [...
    Pos_S.hip.x(START) - Pos_F.hip.x(END),...
    Pos_S.hip.y(START) - Pos_F.hip.y(END),...
    Vel_S.hip.x(START) - Vel_F.hip.x(END),...
    Vel_S.hip.y(START) - Vel_F.hip.y(END)];

%%%% Heel-Strike Defect in Foot Two
output.eventgroup(2).event = [...
    Pos_S.footTwo.x(START) - Pos_F.footTwo.x(END),...
    Pos_S.footTwo.y(START) - Pos_F.footTwo.y(END),...
    Vel_S.footTwo.x(START) - Vel_F.footTwo.x(END),...
    Vel_S.footTwo.y(START) - Vel_F.footTwo.y(END)];

%%%% Heel-Strike Defect in Foot One  (Impact Location at origin)
output.eventgroup(3).event = [...
    Pos_F.footOne.x(END),...
    Pos_F.footOne.y(END)];

%%%% Hip and foot two velocity constraints  (periodic)
output.eventgroup(4).event = [...
    Vel_S.footTwo.x(END) - Vel_F.footOne.x(START),...
    Vel_S.footTwo.y(END) - Vel_F.footOne.y(START)
    Vel_S.hip.x(END) - Vel_F.hip.x(START),...
    Vel_S.hip.y(END) - Vel_F.hip.y(START)];


%%%% Foot Two Initial State (Step vector and periodic constraint)
output.eventgroup(5).event = [...
    Pos_F.footTwo.x(START) + Step_Vec(1),...
    Pos_F.footTwo.y(START) + Step_Vec(2),...
    Vel_F.footTwo.x(START),...
    Vel_F.footTwo.y(START)];

%%%% Relative position of front and rear feet must be periodic
output.eventgroup(6).event = [...
    (Pos_F.footOne.x(START) - Pos_F.footTwo.x(START)) - (Pos_S.footTwo.x(END) - Pos_F.footOne.x(END)),...
    (Pos_F.footOne.y(START) - Pos_F.footTwo.y(START)) - (Pos_S.footTwo.y(END) - Pos_F.footOne.y(END))];

%%%% Relative position of hip must be periodic
output.eventgroup(7).event = [...
    (Pos_F.hip.x(START) - Pos_F.footTwo.x(START)) - (Pos_S.hip.x(END) - Pos_F.footOne.x(END)),...
    (Pos_F.hip.y(START) - Pos_F.footTwo.y(START)) - (Pos_S.hip.y(END) - Pos_F.footOne.y(END))];

%%%% Speed Constraint:
output.eventgroup(8).event = STEP_DIST/Duration;    %%%% HACK %%%% should take into account slope!

%%%% Contact force goes to zero at toe-off:
output.eventgroup(9).event = V1*cos(n1) - H1*sin(n1); 

end

