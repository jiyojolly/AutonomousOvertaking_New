function [position,isterminal,direction] = vzeroReached(~,x,~)
    position = x(4); % The value that we want to be zero
    isterminal = 1;  % Halt integration 
    direction = 0;   % The zero can be approached from either direction
end