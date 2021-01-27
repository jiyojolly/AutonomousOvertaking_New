function U_road = getRoadPotential(X, Y, lanes, eetaRoad)
%GETROADPOTENTIAL Summary of this function goes here
%   Detailed explanation goes here

U_road = zeros(size(X));
for i = [1 7]
    lane1X = lanes(i).Coordinates(:,1);
    %     lane1X_scaled = lane1X - (max(lane1X) - 20);
    lane1X_scaled = (((lane1X-min(lane1X))/(max(lane1X)-min(lane1X)))*40)-20;
    lane1Y = lanes(i).Coordinates(:,2);
    lane1Y_scaled = lane1Y;
    %     lane1Y_scaled = (((lane1Y-min(lane1Y))/(max(lane1Y)-min(lane1Y)))*40)-20;
    U_road = U_road + 0.5*eetaRoad*(1./((lane1X_scaled'-X)+(lane1Y_scaled'-Y))).^2;
    
    if i == 1
        mask1 = (X + Y) > (lane1X' + lane1Y');
    elseif i == 7
        mask1 = (X + Y) < (lane1X' + lane1Y');
    end
    U_road(mask1) = 100;
    
end

