function URoad = getRoadPotential(X, Y, lanes, eetaRoad)
%GETROADPOTENTIAL Summary of this function goes here
%   Detailed explanation goes here

URoad = zeros(size(X));
for i = [1 7]
    laneHeadingAngle = -deg2rad(lanes(i).HeadingAngle);
%     laneHeadingAngle = deg2rad(45);
%     lane1X = lanes(i).Coordinates(:,1);
%     %     lane1X_scaled = lane1X - (max(lane1X) - 20);
%     lane1X_scaled = lane1X;
% %     lane1X_scaled = (((lane1X-min(lane1X))/(max(lane1X)-min(lane1X)))*40)-20;
%     lane1Y = lanes(i).Coordinates(:,2);
%     lane1Y_scaled = lane1Y;
%     %     lane1Y_scaled = (((lane1Y-min(lane1Y))/(max(lane1Y)-min(lane1Y)))*40)-20;
%     U_road = U_road + 0.5*eetaRoad*(1./(((X - lane1X_scaled')/2)+((Y - lane1Y_scaled')/2))).^2;
    URoad = URoad + 0.5*eetaRoad*(1./(sin(laneHeadingAngle)*(X - lanes(i).Coordinates(200,1)) +...
                                      cos(laneHeadingAngle)*(Y - lanes(i).Coordinates(200,2))));
    
    if i == 1
        mask1 = (sin(laneHeadingAngle).*X + cos(laneHeadingAngle).*Y) >...
                    (sin(laneHeadingAngle).*lanes(i).Coordinates(200,1) + cos(laneHeadingAngle).*lanes(i).Coordinates(200,2));
    elseif i == 7
        mask1 = (sin(laneHeadingAngle).*X + cos(laneHeadingAngle).*Y) <...
                    (sin(laneHeadingAngle).*lanes(i).Coordinates(200,1) + cos(laneHeadingAngle).*lanes(i).Coordinates(200,2));
    end
    URoad(mask1) = 100;
    
end
end

