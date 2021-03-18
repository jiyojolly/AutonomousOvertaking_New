function Tbvertex = getVelocityVertex(vLV, vEV, d0VelVertex, gamma, TfVelVertex)
%GETVELOCITYVERTEX Summary of this function goes here
%   Detailed explanation goes here
if vEV > d0VelVertex/TfVelVertex
    Tbvertex =  min((((TfVelVertex*vEV)/d0VelVertex)*exp(gamma*abs(vLV))),50);
else 
    Tbvertex =  min(((1)*exp(gamma*abs(vLV))),50);
end

