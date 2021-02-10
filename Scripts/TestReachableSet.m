close all;
clear;
SimulationParameters;
x0 = [0 0 0 15];
hold on;
reachableSet = generateReachableSet(tf, egoAccMin, egoAccMax, egoSteerAngMin, egoSteerAngMax, v_des, L, l_F,ReachableSetCurveResolution, x0);
reachableSet
plot(polyshape(reachableSet));
f = @() generateReachableSet(tf, egoAccMin, egoAccMax, egoSteerAngMin, egoSteerAngMax, v_des, L, l_F,ReachableSetCurveResolution, x0); % handle to function
timeit(f)