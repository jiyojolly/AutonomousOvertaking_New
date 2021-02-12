function Tbvertex = getVelocityVertex(vLV,vEV)
%GETVELOCITYVERTEX Summary of this function goes here
%   Detailed explanation goes here
Dmax = 5.0;
gamma = 0.5;
inflexPoint = 0;
Tbvertex = 1+ (Dmax/(1+exp(-gamma*(abs(vLV) - inflexPoint))));
end

