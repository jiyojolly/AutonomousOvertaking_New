%%%%%%% Ego Car parameters %%%%%%%%
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
amin = -10.0;
amax = 5.0;
delta_min = deg2rad(-40);
delta_max = deg2rad(40);
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


%%
dt = .05;
tf = 1;
v_des = 15.0;
