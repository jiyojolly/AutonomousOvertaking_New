function Tbvertex = getVelocityVertex(vLV, vEV, Dmax, gamma, inflexPoint)
%GETVELOCITYVERTEX Summary of this function goes here
%   Detailed explanation goes here
d0 = 20; Tf = 3;
if vEV > d0/Tf
    Tbvertex =  min((((Tf*vEV)/d0)*exp(gamma*abs(vLV))),50);
else 
    Tbvertex =  min(((1)*exp(gamma*abs(vLV))),50);
end

