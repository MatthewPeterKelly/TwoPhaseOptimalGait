function plotSolution(plotInfo)

figNum = 1000;  %Start counting here

%stitch together the data for faster plotting
[D, Jumps, JumpIdx] = stitchData(plotInfo.data);

Color_One = 'r';
Color_Two = 'b';
Color_Hip = 'm';

Color_A = [0.5,0.8,0.1];
Color_B = [0.1,0.6,0.6];
Color_Ground = [0.3,0.22,0.1];

FontSize.title = 16;
FontSize.xlabel = 14;
FontSize.ylabel = 14;

LineWidth = 2;
DotSize = 30;
nGround = 250;

energyDatum = mean(D.energy.Total);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Leg Angles                                    %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
createFigure(figNum,'Leg Angles'); figNum=figNum+1;

%Leg One Angle
subplot(3,1,1); hold on;
plot(D.time, D.state.th1,Color_One,'LineWidth',LineWidth);
title('Leg One Angle','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Angle (rad)','FontSize',FontSize.ylabel)
dottedLine(Jumps,axis);
subplot(3,1,2); hold on;
plot(D.time, D.state.th2,Color_Two,'LineWidth',LineWidth);
title('Leg Two Angle','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Angle (rad)','FontSize',FontSize.ylabel)
dottedLine(Jumps,axis);
subplot(3,1,3); hold on;
plot(D.time, D.state.dth1,Color_One,'LineWidth',LineWidth);
plot(D.time, D.state.dth2,Color_Two,'LineWidth',LineWidth);
title('Leg Angular Rates','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Angular Rate (rad/s)','FontSize',FontSize.ylabel)
legend('Leg One','Leg Two');
dottedLine(Jumps,axis);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Leg Lengths                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
createFigure(figNum,'Leg Lengths'); figNum=figNum+1;

subplot(3,1,1); hold on;
plot(D.time, D.state.L1,Color_One,'LineWidth',LineWidth);
title('Leg One Length','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Length (m)','FontSize',FontSize.ylabel)
dottedLine(Jumps,axis);
subplot(3,1,2); hold on;
plot(D.time, D.state.L2,Color_Two,'LineWidth',LineWidth);
title('Leg Two Length','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Length (m)','FontSize',FontSize.ylabel)
dottedLine(Jumps,axis);
subplot(3,1,3); hold on;
plot(D.time, D.state.dL1,Color_One,'LineWidth',LineWidth);
plot(D.time, D.state.dL2,Color_Two,'LineWidth',LineWidth);
title('Leg Length Rates','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Length Rate of Change (m/s)','FontSize',FontSize.ylabel)
legend('Leg One','Leg Two');
dottedLine(Jumps,axis);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Mass Traces                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
createFigure(figNum,'Point Traces'); figNum=figNum+1;

hold on;

plot(D.state.x0(JumpIdx),D.state.y0(JumpIdx),[Color_Hip,'.'],'MarkerSize',DotSize);
plot(D.state.x1(JumpIdx),D.state.y1(JumpIdx),[Color_One,'.'],'MarkerSize',DotSize);
plot(D.state.x2(JumpIdx),D.state.y2(JumpIdx),[Color_Two,'.'],'MarkerSize',DotSize);
bounds = axis;
xGnd = linspace(bounds(1),bounds(2),nGround);
yGnd = zeros(size(xGnd));

plot(D.state.x0,D.state.y0,Color_Hip,'LineWidth',LineWidth);
plot(D.state.x1,D.state.y1,Color_One,'LineWidth',LineWidth);
plot(D.state.x2,D.state.y2,Color_Two,'LineWidth',LineWidth);
xlabel('Horizontal Position (m)','FontSize',FontSize.xlabel)
ylabel('Vertical Position (m)','FontSize',FontSize.ylabel)
title('Point Mass Traces','FontSize',FontSize.title)
legend('Hip','Foot One','Foot Two')

axis equal


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Position and Velocity                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
createFigure(figNum,'Kinematics'); figNum=figNum+1;

subplot(3,1,1); hold on;
plot(D.time,D.state.x0,Color_Hip,'LineWidth',LineWidth);
plot(D.time,D.state.x1,Color_One,'LineWidth',LineWidth);
plot(D.time,D.state.x2,Color_Two,'LineWidth',LineWidth);
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Horizontal Position (m)','FontSize',FontSize.ylabel)
title('Point Mass Horizontal Positions','FontSize',FontSize.title)
legend('Hip','Foot One','Foot Two')
dottedLine(Jumps,axis);

subplot(3,1,2); hold on;
plot(D.time,D.state.y0,Color_Hip,'LineWidth',LineWidth);
plot(D.time,D.state.y1,Color_One,'LineWidth',LineWidth);
plot(D.time,D.state.y2,Color_Two,'LineWidth',LineWidth);
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Vertical Position (m)','FontSize',FontSize.ylabel)
title('Point Mass Vertical Positions','FontSize',FontSize.title)
legend('Hip','Foot One','Foot Two')
dottedLine(Jumps,axis);

subplot(3,1,3); hold on;
Speed.hip = sqrt(D.state.dx0.^2 + D.state.dy0.^2);
Speed.one = sqrt(D.state.dx1.^2 + D.state.dy1.^2);
Speed.two = sqrt(D.state.dx2.^2 + D.state.dy2.^2);
plot(D.time,Speed.hip,Color_Hip,'LineWidth',LineWidth);
plot(D.time,Speed.one,Color_One,'LineWidth',LineWidth);
plot(D.time,Speed.two,Color_Two,'LineWidth',LineWidth);
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Speed (m/s)','FontSize',FontSize.ylabel)
title('Point Mass Speed','FontSize',FontSize.title)
legend('Hip','Foot One','Foot Two')
dottedLine(Jumps,axis);

% % 
% % %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% % %                           Phase-Space                                    %
% % %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% % createFigure(figNum,'Stance Phase-Space'); figNum=figNum+1;
% % 
% % clf; hold on;
% % plot(D.state.th1,D.state.dth1,Color_One,'LineWidth',LineWidth+2);
% % plot(D.state.th2,D.state.dth2,Color_Two,'LineWidth',LineWidth+2);
% % xlabel('Leg Angle (rad)','FontSize',FontSize.xlabel)
% % ylabel('Leg Rate (rad/s)','FontSize',FontSize.ylabel)
% % title('Leg Angle Phase-Space','FontSize',FontSize.title)
% % legend('Stance Leg','Swing Leg')
% % axis(rescaleExtents(axis,1.15)); %Zoom out a bit

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Actuators                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
createFigure(figNum,'Actuators'); figNum=figNum+1;

subplot(3,1,1); hold on;
plot(D.time, D.control.F1, Color_One,'LineWidth',LineWidth);
plot(D.time, D.control.F2, Color_Two,'LineWidth',LineWidth);
title('Leg Axial Force','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Force (N)','FontSize',FontSize.ylabel)
legend('Leg One','Leg Two');
dottedLine(Jumps,axis);
subplot(3,1,2); hold on;
plot(D.time, D.control.T1, Color_One,'LineWidth',LineWidth);
plot(D.time, D.control.T2, Color_Two,'LineWidth',LineWidth);
title('Angle Torque','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Torque (Nm)','FontSize',FontSize.ylabel)
legend('Leg One','Leg Two');
dottedLine(Jumps,axis);
subplot(3,1,3); hold on;
plot(D.time, D.control.Thip,Color_Hip,'LineWidth',LineWidth);
title('Hip Torque','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Torque (Nm)','FontSize',FontSize.ylabel)
dottedLine(Jumps,axis);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Power                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
createFigure(figNum,'Power'); figNum=figNum+1;

subplot(3,1,1); hold on;
plot(D.time, D.power.legOne, Color_One,'LineWidth',LineWidth);
plot(D.time, D.power.legTwo, Color_Two,'LineWidth',LineWidth);
title('Leg Axial Power','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Power (W)','FontSize',FontSize.ylabel)
legend('Leg One','Leg Two');
dottedLine(Jumps,axis);
subplot(3,1,2); hold on;
plot(D.time, D.power.ankleOne, Color_One,'LineWidth',LineWidth);
plot(D.time, D.power.ankleTwo, Color_Two,'LineWidth',LineWidth);
title('Ankle Power','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Power (W)','FontSize',FontSize.ylabel)
legend('Leg One','Leg Two');
dottedLine(Jumps,axis);
subplot(3,1,3); hold on;
plot(D.time, D.power.hip,Color_Hip,'LineWidth',LineWidth);
title('Hip Power','FontSize',FontSize.title)
xlabel('Time (s)','FontSize',FontSize.xlabel)
ylabel('Power (W)','FontSize',FontSize.ylabel)
dottedLine(Jumps,axis);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Contact Forces                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
createFigure(figNum,'Contact Forces'); figNum=figNum+1;

subplot(2,1,1); hold on;
plot(D.time,D.contact.Ang1,Color_One,'LineWidth',LineWidth+1);
plot(D.time,D.contact.Ang2,Color_Two,'LineWidth',LineWidth+1);
title('Contact Force Angle','FontSize',FontSize.title)
ylabel('Angle from vertical (rad)','FontSize',FontSize.ylabel)
xlabel('Time (s)','FontSize',FontSize.xlabel)
legend('Foot One', 'Foot Two')
dottedLine(Jumps,axis);
subplot(2,1,2); hold on;
plot(D.time,D.contact.Mag1,Color_One,'LineWidth',LineWidth+1);
plot(D.time,D.contact.Mag2,Color_Two,'LineWidth',LineWidth+1);
title('Contact Force Magnitude','FontSize',FontSize.title)
ylabel('Force (N)','FontSize',FontSize.ylabel)
xlabel('Time (s)','FontSize',FontSize.xlabel)
legend('Foot One', 'Foot Two')
dottedLine(Jumps,axis);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         System Energy                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
createFigure(figNum,'System Energy'); figNum=figNum+1;
hold on;
if isempty(energyDatum)
    energyDatum = mean(D.energy.Potential);
end
plot(D.time,D.energy.Kinetic,'k:','LineWidth',LineWidth+1)
plot(D.time,D.energy.Potential - energyDatum,'k--','LineWidth',LineWidth+1)
plot(D.time,D.energy.Total - energyDatum,'k-','LineWidth',LineWidth+2)
title('System Energy','FontSize',FontSize.title)
ylabel('Energy (J)','FontSize',FontSize.ylabel)
xlabel('Time (s)','FontSize',FontSize.xlabel)
legend('Kinetic','Potential','Total')
dottedLine(Jumps,axis);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Integrand                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
if isfield(D,'integrand')
    
    createFigure(figNum,'Integrand'); figNum=figNum+1;
    
    subplot(2,1,1); hold on;
    plot(D.time,D.integrand.value,'k-','LineWidth',LineWidth)
    title(['Integrand Value  --  Objective = ' ...
        num2str(plotInfo.stats.objective) ...
        '  --  Method: ' plotInfo.parameters.cost.method])
    xlabel('Time (s)')
    dottedLine(Jumps,axis);
    
    subplot(2,1,2); hold on;
    plot(D.time,D.integrand.absX,'k-','LineWidth',LineWidth)
    plot(D.time,D.integrand.actSquared,'Color',Color_A,'LineWidth',LineWidth)
    plot(D.time,D.integrand.rateSquared,'Color',Color_B,'LineWidth',LineWidth)
    legend('absX','u^2','du^2')
    title('Integrand components')
    xlabel('Time (s)')
    dottedLine(Jumps,axis);
    set(gca,'YScale','log');
    
end


end

%%%% SUB FUNCTIONS %%%%

function dottedLine(time,AXIS)

for i=1:length(time)
    %Plots a dotted line between phases
    plot(time(i)*[1;1],[AXIS(3);AXIS(4)],'k:','LineWidth',1);
end

end

function E = rescaleExtents(extents,scale)

%This function scales the extents of a plot about its center

E = zeros(1,4);

xBnd = extents(1:2);
xCenter = mean(xBnd);
xRange = diff(xBnd);
xScaledRange = scale*xRange;
E(1) = xCenter - 0.5*xScaledRange;
E(2) = xCenter + 0.5*xScaledRange;

yBnd = extents(3:4);
yCenter = mean(yBnd);
yRange = diff(yBnd);
yScaledRange = scale*yRange;
E(3) = yCenter - 0.5*yScaledRange;
E(4) = yCenter + 0.5*yScaledRange;

end
