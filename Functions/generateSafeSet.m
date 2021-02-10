function [safeSet,safeRiskMap] = generateSafeSet(vehPgons,lanes, XSenseRange, YSenseRange, SenseResolution, XSenseRangeArrSize, YSenseRangeArrSize, RiskMaxValue, RiskValueThreshold, eetaRoad)

x = -XSenseRange:SenseResolution:XSenseRange;
y = -YSenseRange:SenseResolution:YSenseRange;
[X,Y] = meshgrid(x,y);
vehRiskMap = zeros(XSenseRangeArrSize,YSenseRangeArrSize);
roadRiskMap = zeros(XSenseRangeArrSize,YSenseRangeArrSize);

vehRiskMap = getVehPotential(X, Y, vehPgons);
roadRiskMap = getRoadPotential(X, Y, lanes, eetaRoad);
safeRiskMap = vehRiskMap + roadRiskMap;
safeRiskMap(safeRiskMap>RiskMaxValue) = RiskMaxValue;

%Mask Values below threshold
safeMask = safeRiskMap < RiskValueThreshold;
safeSet = [X(safeMask), Y(safeMask)];
end