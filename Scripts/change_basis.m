function [ax,ay,az]= change_basis(ax, ay, az, psi)

    v = [ax ay az];
    R = [cos(psi) sin(psi) 0;
         -sin(psi) cos(psi) 0;
         0          0       1 ;];
    v_new = (R*v')';
    ax = v_new(1);
    ay = v_new(2);
    az = v_new(3);
end