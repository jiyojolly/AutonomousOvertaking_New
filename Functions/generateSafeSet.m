function [safeSet,safeRiskMap] = generateSafeSet(vehPgons,lanes)

x = -20:0.1:20;
y = -20:0.1:20;

[X,Y] = meshgrid(x,y);
vehSafeSet = getVehPotential(X, Y, vehPgons);
roadSafeSet = getRoadPotential(X, Y, lanes, 5);
safeRiskMap = vehSafeSet + roadSafeSet;
safeRiskMap(safeRiskMap>100) = 100;

%Mask Values below threshold
safeMask = safeRiskMap < 10;
safeSet = [X(safeMask), Y(safeMask)];
end