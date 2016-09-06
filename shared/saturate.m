function uSat = saturate(u,Bnd)

%This function saturates the actuators for the robot.
%
% u = [N x M] matrix of actuator values
% Bnd = [1 x M] vector of actuator saturation values

uSat = u;
for i=1:length(Bnd)
    upp = uSat(:,i)>Bnd(i);
    low = uSat(:,i)<(-Bnd(i));
    uSat(upp,i) = Bnd(i);
    uSat(low,i) = -Bnd(i);
end

end