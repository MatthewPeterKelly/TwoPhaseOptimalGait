function h = createFigure(figNum,figName)

h  = figure(figNum); clf;
set(h,'Name',figName,'NumberTitle','off');

mp = get(0,'MonitorPositions');



%Shift position to avoid overlapping with windows bar
pos = mp(1,:);
pos(2) = pos(2) + 40;
pos(4) = pos(4) - 40;

set(h,'OuterPosition',pos);

end