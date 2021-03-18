clear;
clc;
clf;
close all;
% SimulationParameters;
%% NPCs Vehicle parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
npcVehcenterToFront = 1.513;
npcVehcenterToRear  = 1.305;
npcVehfrontOverhang = 0.911;
npcVehrearOverhang  = 1.119;
npcVehWidth  = 1.842;
npcVehHeight = 1.517;
npcVehLength = npcVehcenterToFront + npcVehcenterToRear + npcVehfrontOverhang + npcVehrearOverhang;
npcVehWheelbase = npcVehcenterToFront + npcVehcenterToRear;
%Ego Vehicle parameters %%
egoVehcenterToFront = 1.491;
egoVehcenterToRear  = 1.529;
egoVehfrontOverhang = 0.983;
egoVehrearOverhang  = 0.945;
egoVehWidth  = 2.009;
egoVehHeight = 1.370;
egoVehLength = egoVehcenterToFront + egoVehcenterToRear + egoVehfrontOverhang + egoVehrearOverhang;
egoVehWheelbase = egoVehcenterToFront + egoVehcenterToRear;

%Steering Parameters
egoSteeringRatio = 18;
egoMaxSteeringWheelAng = 3*pi;
% Engine paramters
egoAccMin = -10.0;  % m/s^2
egoAccMax = 5.0;    % m/s^2
egoSteerAngMin = -egoMaxSteeringWheelAng/egoSteeringRatio; %rad
egoSteerAngMax = egoMaxSteeringWheelAng/egoSteeringRatio;  %rad

inflationFactor = 1.2;
npcVehLengthInflated = (npcVehLength*inflationFactor) + egoVehLength;
npcVehWidthInflated = (npcVehWidth*inflationFactor) + egoVehWidth;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load Data and create time series
load('SimOutputs/2021_3_11_20_05_51_sim_out_Abort');
start_time_idx = 1;

%Selected Maneuver TS
SelectedManeuver_TS = timeseries(squeeze(logsout{15}.Values.Data(start_time_idx:end,1)),...
    logsout{15}.Values.Time(start_time_idx:end));
%Reference Goal
RefGoalWorld_TS = timeseries([logsout{5}.Values.X.Data(start_time_idx:end)...
    logsout{5}.Values.Y.Data(start_time_idx:end)],...
    logsout{5}.Values.X.Time(start_time_idx:end));
%Reference Intd. MPC Ref
XmpcRefWorld_TS = timeseries(logsout{18}.Values.Data(1,:,start_time_idx:end),...
    logsout{18}.Values.Time(start_time_idx:end));
% Obstacle data
npcVehPgonsWorld_TS = timeseries([logsout{7}.Values.VehPgon(1).X.Data(:,1,start_time_idx:end)...
    logsout{7}.Values.VehPgon(1).Y.Data(:,1,start_time_idx:end)],...
    logsout{7}.Values.VehPgon(1).X.Time(start_time_idx:end));
npcVehPgonsOrgWorld_TS = timeseries([logsout{6}.Values.VehPgon(1).X.Data(:,1,start_time_idx:end)...
    logsout{6}.Values.VehPgon(1).Y.Data(:,1,start_time_idx:end)],...
    logsout{6}.Values.VehPgon(1).X.Time(start_time_idx:end));
MpcOAConstraintsWorld_TS = timeseries(logsout{13}.Values.Data(1,:,start_time_idx:end),...
    logsout{13}.Values.Time(start_time_idx:end));
%Predicted states data
XPredWorld_TS = timeseries(logsout{17}.Values.Data(:,:,start_time_idx:end),...
    logsout{17}.Values.Time(start_time_idx:end));

%Ego car data
VehFdbkISO_TS = timeseries([logsout{16}.Values.Position.X.Data(start_time_idx:end),...
    logsout{16}.Values.Position.Y.Data(start_time_idx:end),...
    logsout{16}.Values.Orientation.psi.Data(start_time_idx:end),...
    logsout{16}.Values.Velocity.Xdot.Data(start_time_idx:end)],...
    logsout{16}.Values.Position.X.Time(start_time_idx:end));

%Ego car states actual
X1Actual_TS = timeseries(logsout{16}.Values.Position.X.Data(start_time_idx:end),...
    logsout{16}.Values.Position.X.Time(start_time_idx:end));
X1Actual_TS.Name = 'Longitudinal Position x1 (m)';
X2Actual_TS = timeseries(logsout{16}.Values.Position.Y.Data(start_time_idx:end),...
    logsout{16}.Values.Position.Y.Time(start_time_idx:end));
X2Actual_TS.Name = 'Lateral Position x2 (m)';
X3Actual_TS = timeseries(logsout{16}.Values.Position.X.Data(start_time_idx:end),...
    logsout{16}.Values.Position.X.Time(start_time_idx:end));
X3Actual_TS.Name = 'Heading Ψ (rad)';
X4Actual_TS = timeseries(logsout{16}.Values.Velocity.Xdot.Data(start_time_idx:end),...
    logsout{16}.Values.Velocity.Xdot.Time(start_time_idx:end));
X4Actual_TS.Name = 'Velocity v (m/s)';

% %Ego car control inputs actual
MV1Actual_TS = timeseries(logsout{16}.Values.Acceleration.xddot.Data(start_time_idx:end),...
    logsout{16}.Values.Acceleration.xddot.Time(start_time_idx:end));
MV1Actual_TS.Name = 'Acc. Cmd a_x (m/s^2)';
MV2Actual_TS = timeseries(logsout{16}.Values.Orientation.psi.Data(start_time_idx:end),...
    logsout{16}.Values.Orientation.psi.Time(start_time_idx:end));
MV2Actual_TS.Name = 'Steer Angle Cmd 𝛿 (rad)';

MV1Pred_TS = timeseries(logsout{11}.Values.Data(1,1,start_time_idx:end),...
    logsout{11}.Values.Time(start_time_idx:end));
MV1Pred_TS.Name = 'Predicted Acc. Cmd a_x (m/s^2)';
MV2Pred_TS = timeseries(logsout{11}.Values.Data(1,2,start_time_idx:end),...
    logsout{11}.Values.Time(start_time_idx:end));
MV2Pred_TS.Name = 'Predicted Steer Ang Cmd a_x (m/s^2)';

%Reference Goal in LV frame
RefGoalLV_TS = timeseries(logsout{8}.Values.Data(1,:,start_time_idx:end),...
    logsout{8}.Values.Time(start_time_idx:end));
%Reference Intd. MPC Ref in LV frame
XmpcRefLV_TS = timeseries(logsout{10}.Values.Data(1,:,start_time_idx:end),...
    logsout{10}.Values.Time(start_time_idx:end));

%Ego vehicle state (VehFdbk) in LV frame
VehFdbkISOLV_TS = timeseries([squeeze(logsout{9}.Values.Data(1,1,start_time_idx:end)),...
    squeeze(logsout{9}.Values.Data(1,2,start_time_idx:end))],...
    logsout{9}.Values.Time(start_time_idx:end));
%%% Create NPC polygons %%%%
Tbvertex = 5;
npcVehPgonV1Org = [npcVehLength/2 npcVehLength/2 -npcVehLength/2 -npcVehLength/2;...
    npcVehWidth/2  -npcVehWidth/2 -npcVehWidth/2  npcVehWidth/2]';
npcVehPgonV1 = [npcVehLengthInflated/2 npcVehLengthInflated/2+Tbvertex npcVehLengthInflated/2 -npcVehLengthInflated/2 -npcVehLengthInflated/2-Tbvertex -npcVehLengthInflated/2;...
    npcVehWidthInflated/2 0                               -npcVehWidthInflated/2 -npcVehWidthInflated/2  0                                npcVehWidthInflated/2]';
npcPgonOrg = polyshape(npcVehPgonV1Org(:,1), npcVehPgonV1Org(:,2));
npcPgon = polyshape(npcVehPgonV1(:,1), npcVehPgonV1(:,2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plotting everything%%%%%%%
% c = rescale(1:size(XPredWorld_TS.Time));
mnvrColor.L = '#006400';
mnvrColor.F = 'blue';
mnvrColor.O = '#CCCC00';
mnvrColor.A = 'red';

fontsize = 12;
linewidth = 1;
%% Plot main trajectory in W-frame and L-Frame
h1 = figure;
set(h1,'DefaultAxesFontSize',fontsize)
set(h1,'defaultAxesFontName','Arial')
set(h1,'defaultTextFontName', 'Arial')
set(h1,'defaultfigurecolor',[1 1 1])

t = tiledlayout(2,1);
% Plot once for Label
nexttile
hold on;
%%% Plot desired %%%
% scatter(RefGoalWorld_TS.Data(i,1), RefGoalWorld_TS.Data(i,2), 50, 'p',...
%     'MarkerEdgeColor','#008000', 'MarkerFaceColor', '#008000','DisplayName','P_d_e_s');
% %% Plot reference
% scatter(XmpcRefWorld_TS.Data(1,1,i), XmpcRefWorld_TS.Data(1,2,i), 25, 'd',...
%     'MarkerEdgeColor','#CCCC00','MarkerFaceColor', '#CCCC00','DisplayName','X_r_e_f');
%%%%%%%%%%%%%%%%%%%%%%
%%% Plot Ego car trajectory(X) %%%
i = 1;
plot(VehFdbkISO_TS.Data(i,1),VehFdbkISO_TS.Data(i,2),...
    'Color', mnvrColor.L,'Marker','None','LineWidth', linewidth);
plot(VehFdbkISO_TS.Data(i,1),VehFdbkISO_TS.Data(i,2),...
    'Color', mnvrColor.F,'Marker','None','LineWidth', linewidth);
plot(VehFdbkISO_TS.Data(i,1),VehFdbkISO_TS.Data(i,2),...
    'Color', mnvrColor.O,'Marker','None','LineWidth', linewidth);
plot(VehFdbkISO_TS.Data(i,1),VehFdbkISO_TS.Data(i,2),...
    'Color', mnvrColor.A,'Marker','None','LineWidth', linewidth);

for i = 1:1:size(VehFdbkISO_TS.Time)-1
    if SelectedManeuver_TS.Data(i) == 0
        plot([VehFdbkISO_TS.Data(i,1) VehFdbkISO_TS.Data(i+1,1)],...
            [VehFdbkISO_TS.Data(i,2) VehFdbkISO_TS.Data(i+1,2)],...
            'Color', mnvrColor.L, 'LineWidth' , linewidth, 'HandleVisibility','off');
    end
    if SelectedManeuver_TS.Data(i) == 1
        plot([VehFdbkISO_TS.Data(i,1) VehFdbkISO_TS.Data(i+1,1)],...
            [VehFdbkISO_TS.Data(i,2) VehFdbkISO_TS.Data(i+1,2)],...
            'Color', mnvrColor.F, 'LineWidth' , linewidth, 'HandleVisibility','off');
    end
    if SelectedManeuver_TS.Data(i) == 2
        plot([VehFdbkISO_TS.Data(i,1) VehFdbkISO_TS.Data(i+1,1)],...
            [VehFdbkISO_TS.Data(i,2) VehFdbkISO_TS.Data(i+1,2)],...
            'Color', mnvrColor.O, 'LineWidth' , linewidth, 'HandleVisibility','off');
    end
    if SelectedManeuver_TS.Data(i) == 3
        plot([VehFdbkISO_TS.Data(i,1) VehFdbkISO_TS.Data(i+1,1)],...
            [VehFdbkISO_TS.Data(i,2) VehFdbkISO_TS.Data(i+1,2)],...
            'Color', mnvrColor.A, 'LineWidth' , linewidth, 'HandleVisibility','off');
    end
end
i=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot Ego Car Polygon %%%
% plgn = polyshape(ego_car_timeseries.Data(:,1,i), ego_car_timeseries.Data(:,2,i));
% plot(plgn,'EdgeColor', '#0000ff', 'FaceColor', 'None', 'LineStyle', ':','DisplayName','Ego Car');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot obstacle polygon %%%
npcPgonOrgWorld = polyshape(npcVehPgonsOrgWorld_TS.Data(:,1,i), npcVehPgonsOrgWorld_TS.Data(:,2,i));
plot(npcPgonOrgWorld,'EdgeColor', 'None','FaceColor', '#FFCCCB');
plot(npcPgonOrgWorld,'EdgeColor', 'None','FaceColor', mnvrColor.L,'HandleVisibility','off');
% npcPgonWorld = polyshape(npcVehPgonsWorld_TS.Data(:,1,i), npcVehPgonsWorld_TS.Data(:,2,i));
% plot(npcPgonWorld,'LineStyle',':','EdgeColor', 'red', 'FaceColor', 'None','DisplayName','Obstacle Inflation')

%%Plot inflated obstacle ellipse %%
% n = MpcOAConstraintsWorld_TS.Data(1,6,i);
% a = MpcOAConstraintsWorld_TS.Data(1,1,i); b = MpcOAConstraintsWorld_TS.Data(1,2,i);
% xe = MpcOAConstraintsWorld_TS.Data(1,3,i); ye = MpcOAConstraintsWorld_TS.Data(1,4,i);
% phi = MpcOAConstraintsWorld_TS.Data(1,5,i);
% ellip = @(x,y) ((((x-xe).*cos(phi) - (y-ye).*sin(phi)))./a).^n + ((((x-xe).*sin(phi) + (y-ye).*cos(phi)))./b).^n - 1;
% fimplicit(ellip, 'Color', '#FF0000', 'LineStyle', '--','DisplayName','Obstacle Inflation');
%Plot Predicted trajectory
% scatter(XPredWorld_TS.Data(:,1,i),XPredWorld_TS.Data(:,2,i), 5, 'o',...
%     'DisplayName','Predicted Trajectory','MarkerEdgeColor','#d3d3d3','MarkerFaceColor','None');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lgnd = legend('Lane Keep','Follow LV', 'Overtake', 'Abort','Lead Vehicle (5 m/s)','NumColumns',5);
% lgnd.Box = 'off';

text(VehFdbkISO_TS.Data(1,1), VehFdbkISO_TS.Data(1,2)+1.0,...
    ['t = ' num2str(VehFdbkISO_TS.Time(1))],'Color','black',...
    'FontSize',fontsize-2,'HorizontalAlignment', 'center',...
    'Position',[0 0],'Units','data');

plot(VehFdbkISO_TS.Data(1,1), VehFdbkISO_TS.Data(1,2),...
                'Color', mnvrColor.L, 'Marker','d', 'LineWidth' , linewidth, 'HandleVisibility','off');
%%% Plot Rest of time %%%
for i = 2:1:size(XPredWorld_TS.Time)
    
    %%% Plot desired %%%
    %     scatter(RefGoalWorld_TS.Data(i,1), RefGoalWorld_TS.Data(i,2), 10, 'p',...
    %         'MarkerEdgeColor','#008000', 'MarkerFaceColor', '#008000','HandleVisibility','off');
    %     %Plot reference
    %     scatter(XmpcRefWorld_TS.Data(1,1,i), XmpcRefWorld_TS.Data(1,2,i), 10, 'd',...
    %         'MarkerEdgeColor','#CCCC00','MarkerFaceColor', '#CCCC00','HandleVisibility','off');
    
    %%% Plot obstacle polygon %%%
    npcPgonOrgWorld = polyshape(npcVehPgonsOrgWorld_TS.Data(:,1,i), npcVehPgonsOrgWorld_TS.Data(:,2,i));
    %npcPgonWorld = polyshape(npcVehPgonsWorld_TS.Data(:,1,i), npcVehPgonsWorld_TS.Data(:,2,i));
    % Single color obstacle plot
    %     plot(npcPgonOrgWorld,'EdgeColor', 'None','FaceColor', '#FFCCCB','HandleVisibility','off');
    %     plot(npcPgonWorld,'LineStyle',':','EdgeColor', 'red', 'FaceColor', 'None','HandleVisibility','off')
    % Maneuver depended plot
    if SelectedManeuver_TS.Data(i-1) == 0
        if SelectedManeuver_TS.Data(i) == 1
            plot(npcPgonOrgWorld,'EdgeColor', 'None','FaceColor', mnvrColor.F,'HandleVisibility','off');
            %         plot(npcPgonWorld,'LineStyle',':','EdgeColor', mnvrColor.L, 'FaceColor', 'None','HandleVisibility','off');
            plot(VehFdbkISO_TS.Data(i,1), VehFdbkISO_TS.Data(i,2),...
                'Color', mnvrColor.F, 'Marker','d', 'LineWidth' , linewidth, 'HandleVisibility','off');
            text(VehFdbkISO_TS.Data(i,1), VehFdbkISO_TS.Data(i,2)+1.0,...
                ['t = ' num2str(VehFdbkISO_TS.Time(i))],'Color','black',...
                'FontSize',fontsize-2,'HorizontalAlignment', 'center',...
                'Position',[0 0],'Units','data');
        end
    end
    if SelectedManeuver_TS.Data(i-1) == 1
        if SelectedManeuver_TS.Data(i) == 2
            plot(npcPgonOrgWorld,'EdgeColor', 'None','FaceColor', mnvrColor.O,'HandleVisibility','off');
            %plot(npcPgonWorld,'LineStyle',':','EdgeColor', mnvrColor.L, 'FaceColor', 'None','HandleVisibility','off');
            plot(VehFdbkISO_TS.Data(i,1), VehFdbkISO_TS.Data(i,2),...
                'Color', mnvrColor.O, 'Marker','d', 'LineWidth' , linewidth, 'HandleVisibility','off');
            text(VehFdbkISO_TS.Data(i,1), VehFdbkISO_TS.Data(i,2)+1.0,...
                ['t = ' num2str(VehFdbkISO_TS.Time(i))],'Color','black',...
                'FontSize',fontsize-2,'HorizontalAlignment', 'center',...
                'Position',[0 0],'Units','data');
        end
    end
    if SelectedManeuver_TS.Data(i-1) == 2
        if SelectedManeuver_TS.Data(i) == 3
            plot(npcPgonOrgWorld,'EdgeColor', 'None','FaceColor', mnvrColor.A,'HandleVisibility','off');
            %         plot(npcPgonWorld,'LineStyle',':','EdgeColor', mnvrColor.L, 'FaceColor', 'None','HandleVisibility','off');
            plot(VehFdbkISO_TS.Data(i,1), VehFdbkISO_TS.Data(i,2),...
                'Color', mnvrColor.A, 'Marker','d', 'LineWidth' , linewidth, 'HandleVisibility','off');
            text(VehFdbkISO_TS.Data(i,1), VehFdbkISO_TS.Data(i,2)+1.0,...
                ['t = ' num2str(VehFdbkISO_TS.Time(i))],'Color','black',...
                'FontSize',fontsize-2,'HorizontalAlignment', 'center',...
                'Position',[0 0],'Units','data');
        end
    end
    if SelectedManeuver_TS.Data(i-1) == 3
        if SelectedManeuver_TS.Data(i) == 1
            plot(npcPgonOrgWorld,'EdgeColor', 'None','FaceColor', mnvrColor.F,'HandleVisibility','off');
            %         plot(npcPgonWorld,'LineStyle',':','EdgeColor', mnvrColor.L, 'FaceColor', 'None','HandleVisibility','off');
            plot(VehFdbkISO_TS.Data(i,1), VehFdbkISO_TS.Data(i,2),...
                'Color', mnvrColor.F, 'Marker','d', 'LineWidth' , linewidth, 'HandleVisibility','off');
            text(VehFdbkISO_TS.Data(i,1), VehFdbkISO_TS.Data(i,2)+1.0,...
                ['t = ' num2str(VehFdbkISO_TS.Time(i))],'Color','black',...
                'FontSize',fontsize-2,'HorizontalAlignment', 'center',...
                'Position',[0 0],'Units','data');
        end
    end
    if SelectedManeuver_TS.Data(i-1) == 2
        if SelectedManeuver_TS.Data(i) == 0
            plot(npcPgonOrgWorld,'EdgeColor', 'None','FaceColor', mnvrColor.L,'HandleVisibility','off');
            %         plot(npcPgonWorld,'LineStyle',':','EdgeColor', mnvrColor.L, 'FaceColor', 'None','HandleVisibility','off');
            plot(VehFdbkISO_TS.Data(i,1), VehFdbkISO_TS.Data(i,2),...
                'Color', mnvrColor.L, 'Marker','d', 'LineWidth' , linewidth, 'HandleVisibility','off');
            text(VehFdbkISO_TS.Data(i,1), VehFdbkISO_TS.Data(i,2)+1.0,...
                ['t = ' num2str(VehFdbkISO_TS.Time(i))],'Color','black',...
                'FontSize',fontsize-2,'HorizontalAlignment', 'center',...
                'Position',[0 0],'Units','data');
        end
    end
    
    %%% Plot inflated obstacle ellipse %%%
    %     n = MpcOAConstraintsWorld_TS.Data(1,6,i);
    %     a = MpcOAConstraintsWorld_TS.Data(1,1,i); b = MpcOAConstraintsWorld_TS.Data(1,2,i);
    %     xe = MpcOAConstraintsWorld_TS.Data(1,3,i); ye = MpcOAConstraintsWorld_TS.Data(1,4,i);
    %     phi = MpcOAConstraintsWorld_TS.Data(1,5,i);
    %     ellip = @(x,y) ((((x-xe).*cos(phi) - (y-ye).*sin(phi)))./a).^n + ((((x-xe).*sin(phi) + (y-ye).*cos(phi)))./b).^n - 1;
    %     fimplicit(ellip, 'Color', '#FF0000', 'LineStyle', '--','HandleVisibility','off')
    
    %%% Plot Ego Car Polygon %%%
    %plgn = polyshape(ego_car_timeseries.Data(:,1,i), ego_car_timeseries.Data(:,2,i));
    %plot(plgn,'EdgeColor', '#0000ff', 'FaceColor', 'None', 'LineStyle', ':','HandleVisibility','off');
    
    
    %%% Plot Predicted trajectory %%%
    %     scatter(XPredWorld_TS.Data(:,1,i),XPredWorld_TS.Data(:,2,i), 5, 'o',...
    %         'MarkerEdgeColor','#d3d3d3','MarkerFaceColor','None','HandleVisibility','off');
end
%Plot lanes
yline(-3.9,'--','Color','black','HandleVisibility','off');
yline(-7.7,'--','Color','black','HandleVisibility','off');
ylim([-(3.8+3.9)-(3.8/2) 0-(2*3.8/4)])
title('Top View (W-Frame)')

% xlim([-210 -190])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot LV POV
nexttile
hold on;

%%% Plot LV Polygon %%%
plot(npcPgonOrg,'EdgeColor', 'None','FaceColor', '#FFCCCB','DisplayName','Lead Vehicle (8 m/s)');
plot(npcPgon,'LineStyle',':','EdgeColor', 'red', 'FaceColor', 'None','DisplayName','Obstacle Inflation')

%%% Plot desired LV %%%
% scatter(RefGoalWorld_TS.Data(i,1), RefGoalWorld_TS.Data(i,2), 50, 'p',...
%         'MarkerEdgeColor','#008000', 'MarkerFaceColor', '#008000','DisplayName','P_d_e_s');

%%% Plot Ego car trajectory(X) in LV frame %%%
i = 1;
plot(VehFdbkISOLV_TS.Data(i,1),VehFdbkISOLV_TS.Data(i,2),...
    'Color', mnvrColor.L,'Marker','None','LineWidth', linewidth);
plot(VehFdbkISOLV_TS.Data(i,1),VehFdbkISOLV_TS.Data(i,2),...
    'Color', mnvrColor.F,'Marker','None','LineWidth', linewidth);
plot(VehFdbkISOLV_TS.Data(i,1),VehFdbkISOLV_TS.Data(i,2),...
    'Color', mnvrColor.O,'Marker','None','LineWidth', linewidth);
plot(VehFdbkISOLV_TS.Data(i,1),VehFdbkISOLV_TS.Data(i,2),...
    'Color', mnvrColor.A,'Marker','None','LineWidth', linewidth);

for i = 1:1:size(VehFdbkISOLV_TS.Time)-1
    if SelectedManeuver_TS.Data(i) == 0
        plot([VehFdbkISOLV_TS.Data(i,1) VehFdbkISOLV_TS.Data(i+1,1)],...
            [VehFdbkISOLV_TS.Data(i,2) VehFdbkISOLV_TS.Data(i+1,2)],...
            'Color', mnvrColor.L, 'LineWidth' , linewidth, 'HandleVisibility','off');
    end
    if SelectedManeuver_TS.Data(i) == 1
        plot([VehFdbkISOLV_TS.Data(i,1) VehFdbkISOLV_TS.Data(i+1,1)],...
            [VehFdbkISOLV_TS.Data(i,2) VehFdbkISOLV_TS.Data(i+1,2)],...
            'Color', mnvrColor.F, 'LineWidth' , linewidth, 'HandleVisibility','off');
    end
    if SelectedManeuver_TS.Data(i) == 2
        plot([VehFdbkISOLV_TS.Data(i,1) VehFdbkISOLV_TS.Data(i+1,1)],...
            [VehFdbkISOLV_TS.Data(i,2) VehFdbkISOLV_TS.Data(i+1,2)],...
            'Color', mnvrColor.O, 'LineWidth' , linewidth, 'HandleVisibility','off');
    end
    if SelectedManeuver_TS.Data(i) == 3
        plot([VehFdbkISOLV_TS.Data(i,1) VehFdbkISOLV_TS.Data(i+1,1)],...
            [VehFdbkISOLV_TS.Data(i,2) VehFdbkISOLV_TS.Data(i+1,2)],...
            'Color', mnvrColor.A, 'LineWidth' , linewidth, 'HandleVisibility','off');
    end
end
i=1;
yline(5.8-3.9,'--','Color','black','HandleVisibility','off');
yline(5.8-7.7,'--','Color','black','HandleVisibility','off');
ylim([5.8-(3.8+3.9)-(3.8/2) 5.8+0-(2*3.8/4)])
title('Top View (L-Frame)')
xlabel(t,'x [m]','FontSize',fontsize,'Color','black','FontName','Arial')
ylabel(t,'y [m]','FontSize',fontsize,'Color','black','FontName','Arial')
t.TileSpacing = 'loose';
t.Padding = 'tight';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot Acceleration, Velocity & Steering Angles
h1 = figure;
set(h1,'DefaultAxesFontSize',fontsize)
set(h1,'defaultAxesFontName','Arial')
set(h1,'defaultTextFontName', 'Arial')
set(h1,'defaultfigurecolor',[1 1 1])
t = tiledlayout(3,1);

nexttile;
hold on;
plot(MV2Pred_TS, 'DisplayName', '𝛿 Pred.', 'LineWidth' , linewidth,'HandleVisibility','off');
yline(egoSteerAngMax,'--','𝛿_{max}','FontSize', fontsize-3, 'Color','black');
yline(egoSteerAngMin,'--','𝛿_{min}','FontSize', fontsize-3, 'Color','black');
ylim([-1,1])
xlim([0,MV2Pred_TS.Time(end)])
ylabel("\delta [rad]",'Color','black')
% legend('boxoff');

nexttile;
hold on;
yyaxis left;
plot(MV1Actual_TS, 'DisplayName', 'a_{x} Actual', 'LineWidth', linewidth);
ylim([-15,15])
ylabel("a_x [m/s^2]",'Color','black')
yyaxis right;
plot(MV1Pred_TS, 'DisplayName', 'a_{x} Cmd.', 'LineWidth', linewidth);
xlim([0,MV1Pred_TS.Time(end)])
ax = gca;
ax.YAxis(2).Visible = 'off'; % remove y-axis
ax.YAxis(1).Color = 'black';
yline(egoAccMax,'--','a_{max}','FontSize', fontsize-3, 'Color','black','HandleVisibility','off');
yline(egoAccMin,'--','a_{min}','FontSize', fontsize-3, 'Color','black','HandleVisibility','off');
lgnd = legend('NumColumns',2);
% lgnd.Box = 'off';
ylim([-15,15]);

%%% Plot Ego car trajectory(X) %%%%%%%%%%%%%%
nexttile;
hold on;
i = 1;
plot(VehFdbkISO_TS.Time(i),VehFdbkISO_TS.Data(i,4),...
    'Color', 'green','Marker','None','LineWidth', linewidth);
plot(VehFdbkISO_TS.Time(i),VehFdbkISO_TS.Data(i,4),...
    'Color', 'blue','Marker', 'None','LineWidth', linewidth);
plot(VehFdbkISO_TS.Time(i),VehFdbkISO_TS.Data(i,4),...
    'Color', '#CCCC00','Marker','None','LineWidth', linewidth);
plot(VehFdbkISO_TS.Time(i),VehFdbkISO_TS.Data(i,4),...
    'Color', 'red','Marker','None','LineWidth', linewidth);

for i = 1:1:size(VehFdbkISO_TS.Time)-1
    if SelectedManeuver_TS.Data(i) == 0
        plot([VehFdbkISO_TS.Time(i) VehFdbkISO_TS.Time(i+1)],...
            [VehFdbkISO_TS.Data(i,4) VehFdbkISO_TS.Data(i+1,4)],...
            'Color', '#006400','Marker','None','HandleVisibility','off');
    end
    if SelectedManeuver_TS.Data(i) == 1
        plot([VehFdbkISO_TS.Time(i) VehFdbkISO_TS.Time(i+1)],...
            [VehFdbkISO_TS.Data(i,4) VehFdbkISO_TS.Data(i+1,4)],...
            'Color', 'blue','Marker', 'None','HandleVisibility', 'off','LineWidth', linewidth);
    end
    if SelectedManeuver_TS.Data(i) == 2
        plot([VehFdbkISO_TS.Time(i) VehFdbkISO_TS.Time(i+1)],...
            [VehFdbkISO_TS.Data(i,4) VehFdbkISO_TS.Data(i+1,4)],...
            'Color', '#CCCC00','Marker','None','HandleVisibility','off','LineWidth', linewidth);
    end
    if SelectedManeuver_TS.Data(i) == 3
        plot([VehFdbkISO_TS.Time(i) VehFdbkISO_TS.Time(i+1)],...
            [VehFdbkISO_TS.Data(i,4) VehFdbkISO_TS.Data(i+1,4)],...
            'Color', 'red','Marker','None','HandleVisibility','off','LineWidth', linewidth);
    end
end

yline(10,'--','Cruise Speed','FontSize', fontsize-3, 'Color',mnvrColor.L);
yline(5,'--','LV Velocity','FontSize', fontsize-3, 'Color','blue');
yline(4,'--','Abort Velocity','FontSize', fontsize-3, 'Color','red');
legend('Lane Keep','Follow LV', 'Overtake', 'Abort','NumColumns',4)
ylabel("v [m/s]", 'Color', 'black');
ylim([0,15])
xlim([0,VehFdbkISO_TS.Time(end)])

t.XLabel.String = 't [s]';
t.XLabel.Color = 'black';
t.TileSpacing = 'tight';
t.Padding = 'tight';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Adwdwd
h1 = figure;
set(h1,'DefaultAxesFontSize',fontsize)
set(h1,'defaultAxesFontName','Arial')
set(h1,'defaultTextFontName', 'Arial')
set(h1,'defaultfigurecolor',[1 1 1])
hold on;
%%% Plot Rest of time %%%
end_idx = round(size(XPredWorld_TS.Time)/1.5);
plot(XPredWorld_TS.Data(:,1,1), XPredWorld_TS.Data(:,2,1),...
    'Color', 'red',...,
    'LineWidth', linewidth);
for i = 1:2:end_idx
    %%% Plot desired %%%
    %     scatter(RefGoalWorld_TS.Data(i,1), RefGoalWorld_TS.Data(i,2), 10, 'p',...
    %         'MarkerEdgeColor','#008000', 'MarkerFaceColor', '#008000','HandleVisibility','off');
    %     %Plot reference
    %     scatter(XmpcRefWorld_TS.Data(1,1,i), XmpcRefWorld_TS.Data(1,2,i), 10, 'd',...
    %         'MarkerEdgeColor','#CCCC00','MarkerFaceColor', '#CCCC00','HandleVisibility','off');
    %%% Plot Predicted trajectory %%%
    plot(XPredWorld_TS.Data(:,1,i), XPredWorld_TS.Data(:,2,i),...
        'Color', 'red','HandleVisibility','off',...,
        'LineWidth', linewidth);
end
plot(VehFdbkISO_TS.Data(1:end_idx,1), VehFdbkISO_TS.Data(1:end_idx,2),...
    'Color', 'blue','Marker','None',...
    'LineWidth', linewidth);
lgnd = legend('X Pred','X Actual','NumColumns',2);
% lgnd.Box = 'off';
%Plot lanes
yline(-3.9,'--','Color','black','HandleVisibility','off');
yline(-7.7,'--','Color','black','HandleVisibility','off');
ylim([-(3.8+3.9)-(3.8/2) 0-(2*3.8/4)])
ylabel("y [m]",'Color','black')
xlabel("x [m]",'Color','black')
