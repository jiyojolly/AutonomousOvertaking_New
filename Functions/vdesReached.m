function [position,isterminal,direction] = vdesReached(t,x,v_des)
    isPositive = double(x(4)>0);
    position = [x(4) - v_des; x(4) ; isPositive]; % The value that we want to be zero
    isterminal = [1; 1; 1];  % Halt integration 
    direction = [1; 0; 0];   % The zero can be approached from either direction
end