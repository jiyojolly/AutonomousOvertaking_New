close all;
clear;
SimulationParameters;
x0 = [0 0 0 9];
hold on;
pgon = generateReachableSet(x0, tf, amin, amax, delta_min, delta_max, v_des, L, l_F);
plot(pgon);
f = @() generateReachableSet(x0, tf, amin, amax, delta_min, delta_max, v_des, L, l_F); % handle to function
timeit(f)