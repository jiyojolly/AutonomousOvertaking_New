function Tbvertex = getVelocityVertex(vLV, vEV, Dmax, gamma, inflexPoint)
%GETVELOCITYVERTEX Summary of this function goes here
%   Detailed explanation goes here
Tbvertex = 1+ (Dmax/(1+exp(-gamma*(abs(vLV) - inflexPoint))));
end

