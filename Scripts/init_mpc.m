clear;
nx = 4; ny = 4; nu = 2;
T_horizon = 1;
Ts = 0.1;
PredHor = T_horizon/Ts;
CntrlHor = 2;
enable_MPC = 0;

acc.min = -10;
acc.max = 5;
steerang.min = deg2rad(-70);
steerang.max = deg2rad(70);
v_des = 8.333; %30km\h

obstcl_ellip_order = 6;
ellip_coeff = [2 2 2 2 2 obstcl_ellip_order];
inflation_factor = 1.2;

x = [0 1 2 3];
mv = [1 1];

mpc_planner = nlmpc(nx,ny,nu);
mpc_planner.Ts = Ts; 
mpc_planner.PredictionHorizon = PredHor;
mpc_planner.ControlHorizon = CntrlHor;
mpc_planner.Model.StateFcn = "vkinematicmodel_bicycle";
mpc_planner.Model.IsContinuousTime = true;
mpc_planner.Model.NumberOfParameters = 1;
mpc_planner.Optimization.CustomIneqConFcn = "constraint_obstcl_avoid";
% mpc_planner.Jacobian.CustomIneqConFcn = "constraint_obstcl_avoid_jacobian";

%Apply limits on input
mpc_planner.ManipulatedVariables(1).Min = acc.min;
mpc_planner.ManipulatedVariables(2).Min = steerang.min;
mpc_planner.ManipulatedVariables(1).Max = acc.max;
mpc_planner.ManipulatedVariables(2).Max = steerang.max;
mpc_planner.ManipulatedVariables(1).RateMin = -0.9;
mpc_planner.ManipulatedVariables(1).RateMax = 0.9;
mpc_planner.ManipulatedVariables(1).RateMin = -0.5;
mpc_planner.ManipulatedVariables(1).RateMax = 0.5;
%Weights
mpc_planner.Weights.OutputVariables = [10 10 2 5];

createParameterBus(mpc_planner,['Controller/MPC/Nonlinear MPC Controller'],'params',{ellip_coeff});
x0 = [2 0 -pi/2 0.3];
u0 = [0.4 0.0];
validateFcns(mpc_planner, x0, u0, [], {ellip_coeff});
% ellip = @(x1,x2) ((((x1-ellip_coeff(3)).*cos(ellip_coeff(5)) - (x2-ellip_coeff(4)).*sin(ellip_coeff(5))))./ellip_coeff(1)).^ellip_coeff(6) + ((((x1-ellip_coeff(3)).*sin(ellip_coeff(5)) + (x2-ellip_coeff(5)).*cos(ellip_coeff(6))))./ellip_coeff(2)).^ellip_coeff(6) - 1; 
% fimplicit(ellip)
