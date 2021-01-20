function pgon = generateReachableSet(x0, tf, amin, amax, delta_min, delta_max, v_des, L, l_F)
%GENERATEREACHABLESET Summary of this function goes here
%   Detailed explanation goes here

%     u00 = [amax 0];
    u01 = [amax delta_min];
    u02 = [amax delta_max];
    opts = odeset('RelTol',1e-2,'AbsTol',1e-4, 'Events', @(t,x) vdesReached(t,x,v_des));
%     [~,y1,te,ye,ie] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,u00),[0 tf], x0, opts);
    [~,y2,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[amax delta_min],L,l_F),[0 tf], x0, opts);
    [~,y3,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[amax delta_max],L,l_F),[0 tf], x0, opts);

    delta_range = delta_max:-0.1:delta_min;
    yi = zeros(length(delta_range),4);
    yibrake = zeros(length(delta_range),4);
    for i = 1: length(delta_range)
        [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[amax delta_range(i)],L,l_F),[0 tf], x0, opts);
        yi(i,:) = y(end,:);
        [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[amin delta_range(i)],L,l_F),[0 tf], x0, opts);
        yibrake(i,:) = y(end,:);
    end

    pgon = polyshape([y3(:,1)' yi(:,1)' y2(end:-1:1,1)' yibrake(:,1)'],[y3(:,2)' yi(:,2)' y2(end:-1:1,2)' yibrake(:,2)']);
    


end

