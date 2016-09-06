function y = footClearance(x,xBnd,h)

%This function returns a cosine curve with the following properties:
%
%   y(xBnd(1)) = 0;
%   y(xBnd(2)) = 0;
%   y(mean(xBnd)) = h;
%

T = xBnd(:,2) - xBnd(:,1);
xMid = 0.5*(xBnd(:,2) + xBnd(:,1));

y = 0.5*h*(1 + cos(2*pi*(x - xMid)./T));

end