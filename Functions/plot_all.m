function plot_all(VehFdbk_ISO, ActorsEgo, Lane, egoVehLength, egoVehWidth, npcVehLength, npcVehWidth, npcVehPgons, safeSet, safeRiskMap)
%PLOT_ALL Summary of this function goes here
%   Detailed explanation goes here
figure(1);
clf;
hold on;
%% Plot Ego car %%
% plot(VehFdbk_ISO.Position_ISO.X,VehFdbk_ISO.Position_ISO.Y, 'o')
egoVehPgon = polyshape([egoVehLength/2 egoVehLength/2 -egoVehLength/2 -egoVehLength/2],...
    [egoVehWidth/2 -egoVehWidth/2 -egoVehWidth/2 egoVehWidth/2]);
% egoVehPgon = rotate(egoVehPgon, rad2deg(VehFdbk_ISO.Orientation_ISO.psi));
plot(egoVehPgon);

%% Plot Lane Boundaries %%
for i = 1:Lane.NumLaneBoundaries
    plot(Lane.LaneBoundaries(i,1).Coordinates(:,1), Lane.LaneBoundaries(i,1).Coordinates(:,2), '-')
end

%% Plot NPCs %%
% 
% for i = 1:ActorsEgo.NumActors
%     plot(ActorsEgo.Actors(i).Position(1,1),ActorsEgo.Actors(i).Position(1,2), 'o')
%     npcLoc = [ActorsEgo.Actors(i).Position(1,1),ActorsEgo.Actors(i).Position(1,2)];
%     npcVehPgonV1 = [npcVehLength/2 npcVehLength/2 -npcVehLength/2 -npcVehLength/2;...
%         npcVehWidth/2 -npcVehWidth/2 -npcVehWidth/2 npcVehWidth/2]';
%     npcVehPgonVerEgo = npcVehPgonV1 + repmat(npcLoc, 4,1);
%     npcVehPgon = rotate(polyshape(npcVehPgonVerEgo(:,1), npcVehPgonVerEgo(:,2)), ActorsEgo.Actors(i).Yaw, npcLoc);
%     plot(npcVehPgon);
% end
for i = 1:npcVehPgons.NumVeh
    npcVehPgon = polyshape(npcVehPgons.VehPgon(i,1).X(:,1),npcVehPgons.VehPgon(i,1).Y(:,1));
    plot(npcVehPgon);
end

plot(safeSet(:,1), safeSet(:,2), 'x');

axis auto;
a = axis;
amax = max(abs(a));
axis([-amax,amax,-amax/4,amax/4]);

hold off;
figure(2);
clf;
x = -20:0.1:20;
y = -20:0.1:20;
[X,Y] = meshgrid(x,y);
h1 = surf(X,Y,safeRiskMap);
set(h1,'LineStyle','none')
end

