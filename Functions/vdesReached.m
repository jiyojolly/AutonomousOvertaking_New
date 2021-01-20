function [position,isterminal,direction] = vdesReached(t,x,v_des)
    position = x(4) - v_des; % The value that we want to be zero
    isterminal = 1;  % Halt integration 
    direction = 0;   % The zero can be approached from either direction
end