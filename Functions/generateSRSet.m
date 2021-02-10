function SRSet = generateSRSet(safeSet,reachableSet)
%GENERATESRSET Summary of this function goes here
%   Detailed explanation goes here
SRMask = inpolygon(safeSet(:,1), safeSet(:,2), reachableSet(:,1), reachableSet(:,2));
safeSetX = safeSet(:,1);
safeSetY = safeSet(:,2);
SRSet = [safeSetX(SRMask), safeSetY(SRMask)]; 
end

