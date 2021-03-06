clf;
hold on;
SimulationParameters;
StructureDefs;
load("SimDataDump.mat");
%%
% Generate mesh grid
x = -20:0.1:20;
y = -20:0.1:20;
[X,Y] = meshgrid(x,y);

% Create Vehicle polygons
% vehPgons = vehPgonsStruct();
% vehPgons.NumVeh = 2;
% vehPgons.VehPgon(1).X = [1 1 2 2];vehPgons.VehPgon(1).Y = [1 2 2 1];
% vehPgons.VehPgon(2).X = [-1 -1 -2 -2];vehPgons.VehPgon(2).Y = [-1 -2 -2 -1];
vehPgons.NumVeh = logsout{11}.Values.NumVeh.Data(1,1,1);
vehPgons.VehPgon(1).X = logsout{11}.Values.VehPgon(1).X.Data(:,:,1); vehPgons.VehPgon(1).Y = logsout{11}.Values.VehPgon(1).Y.Data(:,:,1);
vehPgons.VehPgon(2).X = logsout{11}.Values.VehPgon(2).X.Data(:,:,1); vehPgons.VehPgon(2).Y = logsout{11}.Values.VehPgon(2).Y.Data(:,:,1);
% Create/Load lane information
% lanes = logsout_straight{2}.Values.LaneBoundaries;
lanes = laneStruct();
laneDump = logsout{2}.Values.LaneBoundaries;
for i = 1:length(laneDump)
    lanes(i).Coordinates = laneDump(i).Coordinates.Data(:,:,1);
    plot(lanes(i).Coordinates(:,1), lanes(i).Coordinates(:,2))
end
%Generate Safe set
[safeSet, safeRiskMap] = generateSafeSet(vehPgons,lanes, XSenseRange, YSenseRange, SenseResolution, XSenseRangeArrSize, YSenseRangeArrSize, RiskMaxValue, RiskValueThreshold, eetaRoad);
X(lanes(i).Coordinates(:,1),lanes(i).Coordinates(:,2)) = 0;
%Generate Reachable Set
x0 = [0 0 0 5];
reachableSet = generateReachableSet(tf, egoAccMin, egoAccMax, egoSteerAngMin, egoSteerAngMax, v_des, L, l_F,ReachableSetCurveResolution,...
                                                x0);

%Generate SR Set
SRSet = generateSRSet(safeSet,reachableSet);
%% Plot Sets 
plot(safeSet(:,1), safeSet(:,2), 'x')
plot(SRSet(:,1), SRSet(:,2), 'x')
% h1 = surf(X,Y,safeRiskMap);
% set(h1,'LineStyle','none')

%% Plot ego car
plot(0,0,'o');
plot(polyshape([-2 -2 2 2],[-1 1 1 -1]));
% plot npc vehicles
plot(polyshape(vehPgons.VehPgon(1).X,vehPgons.VehPgon(1).Y), 'FaceColor', 'none');plot(polyshape(vehPgons.VehPgon(2).X,vehPgons.VehPgon(2).Y), 'FaceColor', 'none');
plot(polyshape(reachableSet));

%% Get intermediate ref point
npcVeh.X = 10; npcVeh.Y = 15;
%plot final ref point 
plot(npcVeh.X, npcVeh.Y, 'x');

[maxDistLocationValue, maxDistLocationIndex]  = min(vecnorm(SRSet-[npcVeh.X, npcVeh.Y],2,2));
plot(SRSet(maxDistLocationIndex,1), SRSet(maxDistLocationIndex,2),'x')

