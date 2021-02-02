function dx = vkinematicmodel_bicycle(t, x, u, L, l_F)
% vkinematicmodel_bicycle - bicycle model (see [1])
%
%
% Syntax:
%    dx = vkinematicmodel_bicycle(x,u)
%
% Inputs:
%    x - state vector (x,y,gamma,v)
%    u - input vector (af, psi)
%
% Outputs:
%    f - time-derivative of the stante vector
%

%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%

% Author:       Jiyo Palatti
% Written:      01-Sept-2020
% Last update:  01-Sept-2020
% Last revision:---

%------------- BEGIN CODE --------------

%Bicycle Model with
% - normal force equilibrium for pitching-moments
% - linear tyre model
% state x=[X,Y,psi,v]
% input u=[a,delta]

%parameters: get parameters from p vector
%body
%     m = 1750;
%     J = 2500;
l_R = L-l_F;

%street
%     g = 9.81;
% Initialize dx with zero
dx = zeros(4,1);

%Calculate slip angle (beta)
beta = atan2(l_R*tan(u(2)),l_F+l_R);

% Calculate all angles
cpsibeta = cos(x(3)+beta);
spsibeta = sin(x(3)+beta);
cbeta = cos(beta);
tdelta = tan(u(2));
%position
dx(1,1) = x(4)*cpsibeta;
dx(2,1) = x(4)*spsibeta;
%heading
dx(3,1) = ((x(4)*cbeta*tdelta)/(l_F+l_R));
%velocity
dx(4,1) = u(1);
%------------- END OF CODE --------------