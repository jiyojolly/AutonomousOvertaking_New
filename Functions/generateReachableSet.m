function reachableSet = generateReachableSet(tf, egoAccMin, egoAccMax, egoSteerAngMin, egoSteerAngMax, v_des, L, l_F,ReachableSetCurveResolution,...
    XFdbk, vDesiredFinal)
%GENERATEREACHABLESET Summary of this function goes here
%   Detailed explanation goes here

x0 = [0 0 0 vDesiredFinal];
opts = odeset('RelTol',1e-2,'AbsTol',1e-4, 'Events', @(t,x) vdesReached(t,x,v_des));
delta_range = egoSteerAngMax:-ReachableSetCurveResolution:egoSteerAngMin;
yi = zeros(length(delta_range),4);

[~,y2,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[0 egoSteerAngMin],L,l_F),[0 tf], x0, opts);
[~,y3,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[0 egoSteerAngMax],L,l_F),[0 tf], x0, opts);
for i = 1: length(delta_range)
    [~,y,~,~,~] = ode45(@(t,x) vkinematicmodel_bicycle(t,x,[0 delta_range(i)],L,l_F),[0 tf], x0, opts);
    yi(i,:) = y(end,:);
end
%     plot(yi(:,1),yi(:,2), 'o');
pgon = polyshape([y3(:,1)' yi(:,1)' y2(end:-1:1,1)'],[y3(:,2)' yi(:,2)' y2(end:-1:1,2)']);


reachableSet = pgon.Vertices;

end

