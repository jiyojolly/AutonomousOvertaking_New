function plot_all(VehFdbk_ISO, Actors, Objects_Ego, Lane, egoVehLength, egoVehWidth, npcVehLength, npcVehWidth)
%PLOT_ALL Summary of this function goes here
%   Detailed explanation goes here
clf;
hold on;

%% Plot Ego car %%
% plot(VehFdbk_ISO.Position_ISO.X,VehFdbk_ISO.Position_ISO.Y, 'o')
egoVehPgon = polyshape([egoVehLength/2 egoVehLength/2 -egoVehLength/2 -egoVehLength/2],...
    [egoVehWidth/2 -egoVehWidth/2 -egoVehWidth/2 egoVehWidth/2]);
% egoVehPgon = rotate(egoVehPgon, rad2deg(VehFdbk_ISO.Orientation_ISO.psi));
plot(egoVehPgon);
% disp(L)

%% Plot Lane Boundaries %%
for i = 1:Lane.NumLaneBoundaries
    plot(Lane.LaneBoundaries(i,1).Coordinates(:,1), Lane.LaneBoundaries(i,1).Coordinates(:,2), '-')
end

%% Plot NPCs %%
plot(Actors.Actors.Position(1,1),Actors.Actors.Position(1,2), 'o')
npcVehPgon = polyshape([egoVehLength/2 egoVehLength/2 -egoVehLength/2 -egoVehLength/2],...
    [egoVehWidth/2 -egoVehWidth/2 -egoVehWidth/2 egoVehWidth/2]);
end

