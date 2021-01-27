clf;
hold on;
% vehPgon1 = polyshape([1 1 2 2], [1 2 2 1]);
% vehPgon2 = polyshape([-1 -1 -2 -2], [-1 -2 -2 -1]);
%
% point = [1.5 1.75]
% xp = [-1];
% yp = [-3];
%
% xv = [1 1 2 2];
% yv = [1 2 2 1];
%
% % [d_min, x_d_min, y_d_min] = p_poly_dist(point(1), point(2), vehPgon.Vertices(:,1)', vehPgon.Vertices(:,2)');
%
% x = -20:0.1:20;
% y = -20:0.1:20;
% [X,Y] = meshgrid(x,y);
%
% [d_min, x_d_min, y_d_min] = p_poly_dist(X(:), Y(:), vehPgon1.Vertices(:,1)', vehPgon1.Vertices(:,2)', true);
% [d_min2, x_d_min2, y_d_min2] = p_poly_dist(X(:), Y(:), vehPgon2.Vertices(:,1)', vehPgon2.Vertices(:,2)', true);
%
% D_min_mat = reshape(d_min, size(X));
% D_min_mat2 = reshape(d_min2, size(X));
% U_SafeVeh = getYukawaPotential(D_min_mat,10, 0.5);
% U_SafeVeh2 = getYukawaPotential(D_min_mat2,10, 0.5);
% U_SafeVeh(U_SafeVeh <= 0) = 100;
% U_SafeVeh2(U_SafeVeh2 <= 0) = 100;
% U_SafeVeh = U_SafeVeh + U_SafeVeh2;
%
% lanes = logsout{2}.Values.LaneBoundaries;
% lane1 = logsout{2}.Values.LaneBoundaries(1).Coordinates;
% % [d_minLane, x_d_minLane, y_d_minLane] = p_poly_dist(X(:), Y(:), lane1.Data(:,1,1), lane1.Data(:,2,1));
% % D_min_matLane = reshape(d_minLane, size(X));
% % U_SafeLanes = getYukawaPotential(D_min_matLane,10, 0.5);
% % U_SafeLanes(U_SafeLanes > 100) = 100;
% U_SafeLanes = getRoadPotential(X, Y, lanes, 5);
% U_SafeLanes(U_SafeLanes > 100) = 100;
% U_SafeLanes(U_SafeLanes < -100) = -100;
%
% % plot(x_d_minLane, y_d_minLane,'x')
% for i = 1:length(logsout{2}.Values.LaneBoundaries)
%     plot(lanes(i).Coordinates.Data(:,1,1), lanes(i).Coordinates.Data(:,2,1), 'x')
% end
% plot(vehPgon1);plot(vehPgon2);
% h1 = surf(X,Y,U_SafeVeh);
% set(h1,'LineStyle','none')
% h2 = surf(X,Y,U_SafeLanes);
% set(h2,'LineStyle','none')


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
end
%Generate Safe set
[safeSet, safeRiskMap] = generateSafeSet(vehPgons,lanes);

%plot stuff
plot(polyshape(vehPgons.VehPgon(1).X,vehPgons.VehPgon(1).Y));plot(polyshape(vehPgons.VehPgon(2).X,vehPgons.VehPgon(2).Y));
plot(safeSet(:,1), safeSet(:,2))
h1 = surf(X,Y,safeRiskMap);
set(h1,'LineStyle','none')
