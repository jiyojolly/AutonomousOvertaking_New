function [position,isterminal,direction] = vdesReached(t,x,v_des)
    position = [x(4) - v_des; x(4) ; x(4)>0]; % The value that we want to be zero
    isterminal = [1; 1; 1];  % Halt integration 
    direction = [0; 0; 0];   % The zero can be approached from either direction
end