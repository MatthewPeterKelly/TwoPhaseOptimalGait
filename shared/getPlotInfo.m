function plotInfo = getPlotInfo(output)

%This extracts information for plotting from the output of a GPOPS
%optimization run.

P = output.result.setup.auxdata;
P.ground.step_dist = output.result.solution.parameter;
nPhase = length(P.phase);
D = zeros(nPhase,0);

plotInfo.stats.objective = output.result.objective;

for iphase = 1:length(P.phase)
    
    time = output.result.interpsolution.phase(iphase).time;
    phase = P.phase{iphase};
    
    switch phase
        case 'S' %Single stance, with Stance_Foot == Foot_One
            
            IdxState = 1:8;
            IdxAct = 9:12;
            IdxActRate = 1:4;
            IdxAbsPos = 5:8;
            IdxAbsNeg = 9:12;
            
            States = output.result.interpsolution.phase(iphase).state(:,IdxState);
            Actuators = output.result.interpsolution.phase(iphase).state(:,IdxAct);
            Actuator_Rate = output.result.interpsolution.phase(iphase).control(:,IdxActRate);
            AbsPos = output.result.interpsolution.phase(iphase).control(:,IdxAbsPos);
            AbsNeg = output.result.interpsolution.phase(iphase).control(:,IdxAbsNeg);
            Parameters = P.dynamics;
            
            Data = formatData(time, phase, States, Actuators, Parameters);
            names = fieldnames(Data);
            for i=1:length(names)
                D(iphase).(names{i}) = Data.(names{i});
            end
            
            Step_Dist = output.result.solution.parameter;
                        
            [cost, ~, info] = costFunc(States, Actuators, Actuator_Rate,...
                P.dynamics,P.cost , AbsPos, AbsNeg, D(iphase).phase, Step_Dist);
            D(iphase).integrand.value = cost;
            D(iphase).integrand.absX = info.absX;
            D(iphase).integrand.actSquared = info.actSquared;
            D(iphase).integrand.rateSquared = info.rateSquared;
            
            D(iphase).objective.value = output.result.objective;
            D(iphase).objective.method = P.cost.method;
            
            D(iphase).step.distance = Step_Dist;
            
        case 'D' %Double stance, with Stance_Foot at origin
            
            IdxState = 1:4;
            IdxAct = 5:6;
            IdxActRate = 1:2;
            IdxAbsPos = 3:4;
            IdxAbsNeg = 5:6;
            
            States = output.result.interpsolution.phase(iphase).state(:,IdxState);
            Actuators = output.result.interpsolution.phase(iphase).state(:,IdxAct);
            Actuator_Rate = output.result.interpsolution.phase(iphase).control(:,IdxActRate);
            AbsPos = output.result.interpsolution.phase(iphase).control(:,IdxAbsPos);
            AbsNeg = output.result.interpsolution.phase(iphase).control(:,IdxAbsNeg);
            Parameters = P.dynamics;
            
            Step_Dist = output.result.solution.parameter;
            Parameters.x2 = -Step_Dist;
            Parameters.y2 = 0;
            
            Data = formatData(time, phase, States, Actuators, Parameters);
            names = fieldnames(Data);
            for i=1:length(names)
                D(iphase).(names{i}) = Data.(names{i});
            end
          
            [cost, ~, info] = costFunc(States, Actuators, Actuator_Rate,...
                Parameters,P.cost , AbsPos, AbsNeg, phase, Step_Dist);
            D(iphase).integrand.value = cost;
            D(iphase).integrand.absX = info.absX;
            D(iphase).integrand.actSquared = info.actSquared;
            D(iphase).integrand.rateSquared = info.rateSquared;
            
            D(iphase).objective.value = output.result.objective;
            D(iphase).objective.method = P.cost.method;
            
            D(iphase).step.distance = Step_Dist;
            
        case 'F' %Single stance, with Stance_Foot == Foot_One
            
            IdxState = 1:12;
            IdxAct = 13:15;
            IdxActRate = 1:3;
            IdxAbsPos = 4:6;
            IdxAbsNeg = 7:9;
            
            States = output.result.interpsolution.phase(iphase).state(:,IdxState);
            Actuators = output.result.interpsolution.phase(iphase).state(:,IdxAct);
            Actuator_Rate = output.result.interpsolution.phase(iphase).control(:,IdxActRate);
            AbsPos = output.result.interpsolution.phase(iphase).control(:,IdxAbsPos);
            AbsNeg = output.result.interpsolution.phase(iphase).control(:,IdxAbsNeg);
            Parameters = P.dynamics;
            
             Data = formatData(time, phase, States, Actuators, Parameters);
            names = fieldnames(Data);
            for i=1:length(names)
                D(iphase).(names{i}) = Data.(names{i});
            end
            
            Step_Dist = output.result.solution.parameter;
            
            [cost, ~, info] = costFunc(States, Actuators, Actuator_Rate,...
                P.dynamics,P.cost , AbsPos, AbsNeg, D(iphase).phase, Step_Dist);
            D(iphase).integrand.value = cost;
            D(iphase).integrand.absX = info.absX;
            D(iphase).integrand.actSquared = info.actSquared;
            D(iphase).integrand.rateSquared = info.rateSquared;
            
            D(iphase).objective.value = output.result.objective;
            D(iphase).objective.method = P.cost.method;
            
            D(iphase).step.distance = Step_Dist;
            
        otherwise
            error('Unsupported Phase')
    end
end

%Go through and apply shifts necessary to line things up
for iphase=2:length(P.phase)
    D(iphase).time = D(iphase).time + D(iphase-1).time(end);
end

plotInfo.data = D;
plotInfo.parameters = P;
plotInfo.parameters.ground.type = 'flat';

end