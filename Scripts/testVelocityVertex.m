clc;close all;clear;
x = 0:0.1:50;
vLV = 8
Dmax = 2; gamma = 0.05;inflexPoint = 50;
for i = 1:length(x)
    TbVertex(i) = getVelocityVertex(x(i)-vLV,x(i),Dmax,gamma,inflexPoint);
end

plot(x,TbVertex,'x');