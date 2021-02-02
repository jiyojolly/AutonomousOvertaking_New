function U_SafeVeh  = getVehPotential(X, Y, vehPgons)
%GENERATECARPOTENTIAL Summary of this function goes here
%   Detailed explanation goes here
U_SafeVeh = zeros(size(X));

for i = 1:vehPgons.NumVeh
    [d_min, x_d_min, y_d_min] = p_poly_dist(X(:), Y(:), vehPgons.VehPgon(i).X, vehPgons.VehPgon(i).Y, true);
    D_min_mat = reshape(d_min, size(X));
    U_SafeVeh_i = getYukawaPotential(D_min_mat,10, 0.5);
    U_SafeVeh_i(U_SafeVeh_i <= 0) = 100;
    U_SafeVeh = U_SafeVeh + U_SafeVeh_i;
end

