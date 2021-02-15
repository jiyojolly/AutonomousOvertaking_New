%%%%%%% Ego Car parameters %%%%%%%%
clear;
clc;
egoVehcenterToFront = 1.491;
egoVehcenterToRear  = 1.529;
egoVehfrontOverhang = 0.983;
egoVehrearOverhang  = 0.945;
egoVehWidth  = 2.009;
egoVehHeight = 1.370;
egoVehLength = egoVehcenterToFront + egoVehcenterToRear + egoVehfrontOverhang + egoVehrearOverhang;
egoVehWheelbase = egoVehcenterToFront + egoVehcenterToRear;

egoVehicleDims = vehicleDimensions(egoVehLength,egoVehWidth,egoVehHeight, ...
    'FrontOverhang',egoVehfrontOverhang,'RearOverhang',egoVehrearOverhang);
% Wheelbase
L = 3.705;
l_F = 1.2525;
%Steering Parameters
egoSteeringRatio = 18;
egoMaxSteeringWheelAng = 3*pi;
% Engine paramters
egoAccMin = -10.0;  % m/s^2
egoAccMax = 5.0;    % m/s^2
egoSteerAngMin = -egoMaxSteeringWheelAng/egoSteeringRatio; %rad
egoSteerAngMax = egoMaxSteeringWheelAng/egoSteeringRatio;  %rad
%% NPCs Vehicle parameters %%
npcVehcenterToFront = 1.513;
npcVehcenterToRear  = 1.305;
npcVehfrontOverhang = 0.911;
npcVehrearOverhang  = 1.119;
npcVehWidth  = 1.842;
npcVehHeight = 1.517;
npcVehLength = npcVehcenterToFront + npcVehcenterToRear + npcVehfrontOverhang + npcVehrearOverhang;
npcVehWheelbase = npcVehcenterToFront + npcVehcenterToRear;
npcVehDims = vehicleDimensions(npcVehLength,npcVehWidth,npcVehHeight, ...
    'FrontOverhang',npcVehfrontOverhang,'RearOverhang',npcVehrearOverhang);

%% Sensing Range & grid resolutions
XSenseRange = 20; YSenseRange = 20;
SenseResolution = 0.1;
XSenseRangeArrSize = (2*XSenseRange/SenseResolution)+1;YSenseRangeArrSize = (2*YSenseRange/SenseResolution)+1;
ReachableSet_MaxVertices = 200;
%% Safe Reachable Set Generation
RiskMaxValue = 100; RiskValueThreshold = 15;
eetaRoad = 5;
ReachableSetCurveResolution = 0.1;
%% Controller parameters
dt = .05;
tf = 2.0;
v_des = 8; %m/s %30km\h
controllerSampleTime = 0.1;
t=0;
%MPC Parameters
T_horizon = tf;
PredHor = 10;
CntrlHor = 10;
nx = 4; ny = 4; nu = 2;
obstcl_ellip_order = 6;
inflation_factor = 1.0;

mpc_planner = nlmpc(nx,ny,nu);
mpc_planner.Ts = controllerSampleTime;
mpc_planner.PredictionHorizon = PredHor;
mpc_planner.ControlHorizon = CntrlHor;
mpc_planner.Model.StateFcn = @(x,u,params) vkinematicmodel_bicycle(t,x,u,L,l_F);
mpc_planner.Model.IsContinuousTime = true;
mpc_planner.Model.NumberOfParameters = 1;
mpc_planner.Optimization.CustomIneqConFcn = "constraint_obstcl_avoid";
% mpc_planner.Jacobian.CustomIneqConFcn = "constraint_obstcl_avoid_jacobian";

%Apply limits on input
mpc_planner.ManipulatedVariables(1).Min = egoAccMin;
mpc_planner.ManipulatedVariables(2).Min = egoSteerAngMin;
mpc_planner.ManipulatedVariables(1).Max = egoAccMax;
mpc_planner.ManipulatedVariables(2).Max = egoSteerAngMax;
%Rate Limits
% mpc_planner.ManipulatedVariables(1).RateMin = -5.0*(controllerSampleTime^1);
% mpc_planner.ManipulatedVariables(1).RateMax = 5.0*(controllerSampleTime^1);
% mpc_planner.ManipulatedVariables(2).RateMin = -1.5*(controllerSampleTime^1);
% mpc_planner.ManipulatedVariables(2).RateMax = 1.5*(controllerSampleTime^1);
%Weights
% mpc_planner.Weights.OutputVariables = [repmat([0 0 10 10],PredHor/2, 1);
%                                        repmat([10 10 0 0],round(PredHor/4)-1, 1);...
%                                        repmat([20 20 0 0],round(PredHor/4)-1, 1);...
%                                        50 50 5 10];
mpc_planner.Weights.OutputVariables = [repmat([5 5 0 5],PredHor/2, 1);
                                       repmat([5 10 0 5],round(PredHor/4)-1, 1);...
                                       repmat([10 10 10 10],round(PredHor/4)-1, 1);...
                                       10 50 50 30];
% mpc_planner.Weights.OutputVariables = [repmat([5 5 0 5],2, 1);
%                                        repmat([5 10 0 10],2, 1);
%                                        [50 50 50 10]];

mpc_planner.Weights.ManipulatedVariables = [5 50];
  

ellipCoeffValidateFcns = [2 2 2 2 2 obstcl_ellip_order];
createParameterBus(mpc_planner,['Controller/Control/MPC/Nonlinear MPC Controller'],'MPCparams',{ellipCoeffValidateFcns});
xValidateFcns = [2 0 -pi/2 0.3];
uValidateFcns = [0.4 0.0];
validateFcns(mpc_planner, xValidateFcns, uValidateFcns, [], {ellipCoeffValidateFcns});


%% For Plotting
plotSampleTime = 0.1;

%%
myDictionaryObj = Simulink.data.dictionary.open('AOTDataDictionary.sldd');
[~,~] = importFromBaseWorkspace(myDictionaryObj,'existingVarsAction','overwrite', 'clearWorkspaceVars', true);
if myDictionaryObj.HasUnsavedChanges
    saveChanges(myDictionaryObj);
end
clear myDictionaryObj;
