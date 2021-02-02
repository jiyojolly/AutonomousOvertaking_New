function plot_all(XSenseRange, YSenseRange, SenseResolution,egoVehLength, egoVehWidth,...
                  VehFdbk_ISO, ActorsEgo, Lanes, npcVehPgons, safeSetEgo, safeRiskMapEgo, reachableSetEgo, SRSetEgo, XPredictedWorld, XmpcRefWorld)
%PLOT_ALL Summary of this function goes here
%   Detailed explanation goes here
figure(1);
clf;
hold on;
%% Plot Sets

% Plot Safe Set
plot(safeSetEgo(:,1), safeSetEgo(:,2), '.', 'Color', '#ffffed');

% Plot Reachable Set
pg = plot(polyshape(reachableSetEgo));
pg.FaceColor = 'none';
pg.EdgeColor = 'green';

% Plot Safe, Reachable Set
plot(SRSetEgo(:,1), SRSetEgo(:,2), 'x', 'Color', 'g');


%% Plot Ego car %%
% plot(VehFdbk_ISO.Position_ISO.X,VehFdbk_ISO.Position_ISO.Y, 'o')
egoVehPgon = polyshape([egoVehLength/2 egoVehLength/2 -egoVehLength/2 -egoVehLength/2],...
    [egoVehWidth/2 -egoVehWidth/2 -egoVehWidth/2 egoVehWidth/2]);
% egoVehPgon = rotate(egoVehPgon, rad2deg(VehFdbk_ISO.Orientation_ISO.psi));
pg1 = plot(egoVehPgon);
pg1.EdgeColor = 'blue';

%% Plot Lane Boundaries %%
for i = 1:Lanes.NumLaneBoundaries
    plot(Lanes.LaneBoundaries(i,1).Coordinates(:,1), Lanes.LaneBoundaries(i,1).Coordinates(:,2), '-')
end

%% Plot NPCs %%
for i = 1:npcVehPgons.NumVeh
    npcVehPgon = polyshape(npcVehPgons.VehPgon(i,1).X(:,1),npcVehPgons.VehPgon(i,1).Y(:,1));
    pg2 = plot(npcVehPgon);
    pg2.EdgeColor = 'red';
    pg2.FaceColor = 'none';
end

%% Plot Planning related stuff
    inputPose.X = XmpcRefWorld(1);inputPose.Y = XmpcRefWorld(2);inputPose.Z = 0.0;
    outputPose = transformEgo(inputPose, VehFdbk_ISO.Position_ISO, VehFdbk_ISO.Orientation_ISO, 0);
    plot(outputPose.X,outputPose.Y,'x')
%Plot MPC predicted path      
%     if ~isscalar(obstcl) && nnz(x_pred) ~= 0
for i = 1:size(XPredictedWorld, 1)
    inputPose.X = XPredictedWorld(i,1);inputPose.Y = XPredictedWorld(i,2);inputPose.Z = XPredictedWorld(i,3);
    outputPose = transformEgo(inputPose, VehFdbk_ISO.Position_ISO, VehFdbk_ISO.Orientation_ISO, 0);
    scatter(outputPose.X,outputPose.Y,'filled')
end
%% Set Figure
axis auto;
axis([-XSenseRange*2,XSenseRange*2,-YSenseRange,YSenseRange]);

% %% Plot 3d Map
% hold off;
% figure(2);
% clf;
% x = -XSenseRange:SenseResolution:XSenseRange;
% y = YSenseRange:-SenseResolution:-YSenseRange;
% [X,Y] = meshgrid(x,y);
% h1 = surf(X,Y,safeRiskMapEgo);
% set(h1,'LineStyle','none')

end

