function reachableSet = generateReachableSet(tf, egoAccMin, egoAccMax, egoSteerAngMin, egoSteerAngMax, v_des, L, l_F,ReachableSetCurveResolution,...
    XFdbk)
%GENERATEREACHABLESET Summary of this function goes here
%   Detailed explanation goes here

x0StoppingDist = [0 0 0 XFdbk(4)];
x0 = [0 0 0 0];
x0StoppingDist = x0;
% x0StoppingDist = x0;
relaxMinAccLimits = 0.5;
relaxMinVelLimits = 0.5;
opts = odeset('RelTol',1e-2,'AbsTol',1e-4, 'Events', @(t,x) vdesReached(t,x,v_des));
optsExceedVdes = odeset('RelTol',1e-2,'AbsTol',1e-4, 'Events', @(t,x) vdesReached(t,x,XFdbk(4)+relaxMinVelLimits));
%     [~,y1,te,ye,ie] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,u00),[0 tf], x0, opts);
delta_range = egoSteerAngMax:-ReachableSetCurveResolution:egoSteerAngMin;
yi = zeros(length(delta_range),4);
yibrake = zeros(length(delta_range),4);

velocityCheck = XFdbk(4);
% velocityCheck = 0;

if velocityCheck < 0
    x0 = [0 0 0 0];
    [~,y2,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMax egoSteerAngMin],L,l_F),[0 tf], x0, opts);
    [~,y3,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMax egoSteerAngMax],L,l_F),[0 tf], x0, opts);
    for i = 1: length(delta_range)
        [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMax delta_range(i)],L,l_F),[0 tf], x0, opts);
        yi(i,:) = y(end,:);
    end
    %     plot(yi(:,1),yi(:,2), 'o');
    pgon = polyshape([y3(:,1)' yi(:,1)' y2(end:-1:1,1)'],[y3(:,2)' yi(:,2)' y2(end:-1:1,2)']);
elseif velocityCheck == 0
    [~,y2,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMax egoSteerAngMin],L,l_F),[0 tf], x0, opts);
    [~,y3,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMax egoSteerAngMax],L,l_F),[0 tf], x0, opts);
    for i = 1: length(delta_range)
        [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMax delta_range(i)],L,l_F),[0 tf], x0, opts);
        yi(i,:) = y(end,:);
    end
    %     plot(yi(:,1),yi(:,2), 'o');
    pgon = polyshape([y3(:,1)' yi(:,1)' y2(end:-1:1,1)'],[y3(:,2)' yi(:,2)' y2(end:-1:1,2)']);
% elseif velocityCheck > 0 && velocityCheck < v_des
else
    [~,y2,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMax egoSteerAngMin],L,l_F),[0 tf], x0, opts);
    [~,y3,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMax egoSteerAngMax],L,l_F),[0 tf], x0, opts);
    [~,y2brake,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin egoSteerAngMin],L,l_F),[0 tf], x0StoppingDist, opts);
    [~,y3brake,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin egoSteerAngMax],L,l_F),[0 tf], x0StoppingDist, opts);
    for i = 1: length(delta_range)
        [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMax delta_range(i)],L,l_F),[0 tf], x0, opts);
        yi(i,:) = y(end,:);
        [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin delta_range(i)],L,l_F),[0 tf], x0StoppingDist, opts);
        yibrake(i,:) = y(end,:);
    end
    %        plot(yi(:,1),yi(:,2), 'o');
    %        plot(yibrake(:,1),yibrake(:,2), 'o');
    pgon_acc = polyshape([y3(:,1)' yi(:,1)' y2(end:-1:1,1)'],[y3(:,2)' yi(:,2)' y2(end:-1:1,2)' ]);
    pgon_brake = polyshape([y3brake(:,1)' yibrake(:,1)' y2brake(end:-1:1,1)'],[y3brake(:,2)' yibrake(:,2)' y2brake(end:-1:1,2)' ]);
    pgon = rmslivers(subtract(pgon_acc, pgon_brake), 0.005);
    
%     if isempty(pgon.Vertices)
%         [~,y2,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin+relaxMinAccLimits egoSteerAngMin],L,l_F),[0 tf], x0, opts);
%         [~,y3,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin+relaxMinAccLimits egoSteerAngMax],L,l_F),[0 tf], x0, opts);
%         [~,y2brake,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin egoSteerAngMin],L,l_F),[0 tf], x0, opts);
%         [~,y3brake,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin egoSteerAngMax],L,l_F),[0 tf], x0, opts);
%         for i = 1: length(delta_range)
%             [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin+relaxMinAccLimits delta_range(i)],L,l_F),[0 tf], x0, opts);
%             yi(i,:) = y(end,:);
%             [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin delta_range(i)],L,l_F),[0 tf], x0, opts);
%             yibrake(i,:) = y(end,:);
%         end
%         %        plot(yi(:,1),yi(:,2), 'o');
%         %        plot(yibrake(:,1),yibrake(:,2), 'o');
%         pgon_acc = polyshape([y3(:,1)' yi(:,1)' y2(end:-1:1,1)'],[y3(:,2)' yi(:,2)' y2(end:-1:1,2)' ]);
%         pgon_brake = polyshape([y3brake(:,1)' yibrake(:,1)' y2brake(end:-1:1,1)'],[y3brake(:,2)' yibrake(:,2)' y2brake(end:-1:1,2)' ]);
%         pgon = rmslivers(subtract(pgon_acc, pgon_brake), 0.005);
%         
%     end
    
% else
%     [~,y2,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin+relaxMinAccLimits egoSteerAngMin],L,l_F),[0 tf], x0StoppingDist, optsExceedVdes);
%     [~,y3,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin+relaxMinAccLimits egoSteerAngMax],L,l_F),[0 tf], x0StoppingDist, optsExceedVdes);
%     [~,y2brake,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin egoSteerAngMin],L,l_F),[0 tf], x0StoppingDist, optsExceedVdes);
%     [~,y3brake,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin egoSteerAngMax],L,l_F),[0 tf], x0StoppingDist, optsExceedVdes);
%     for i = 1: length(delta_range)
%         [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin+relaxMinAccLimits delta_range(i)],L,l_F),[0 tf], x0StoppingDist, optsExceedVdes);
%         yi(i,:) = y(end,:);
%         [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[egoAccMin delta_range(i)],L,l_F),[0 tf], x0StoppingDist, optsExceedVdes);
%         yibrake(i,:) = y(end,:);
%     end
%     %        plot(yi(:,1),yi(:,2), 'o');
%     %        plot(yibrake(:,1),yibrake(:,2), 'o');
%     pgon_acc = polyshape([y3(:,1)' yi(:,1)' y2(end:-1:1,1)'],[y3(:,2)' yi(:,2)' y2(end:-1:1,2)' ]);
%     pgon_brake = polyshape([y3brake(:,1)' yibrake(:,1)' y2brake(end:-1:1,1)'],[y3brake(:,2)' yibrake(:,2)' y2brake(end:-1:1,2)' ]);
%     pgon = rmslivers(subtract(pgon_acc, pgon_brake), 0.005);
    
    
    
end

reachableSet = pgon.Vertices;

end

