function dx = vkinematicmodel_bicycleWrapper(x,u,ellipCoeff,L,l_F)
%VKINEMATICMODEL_BICYCLEWRAPPER Summary of this function goes here
%   Detailed explanation goes here
t = 0;
dx = vkinematicmodel_bicycle(t,x,u,L,l_F);
end

