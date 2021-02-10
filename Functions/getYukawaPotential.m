function U_yukawa = getYukawaPotential(Kd, Yukawa_Acar, Yukawa_alpha)
%GETYUKAWAPOTENTIAL Summary of this function goes here
%   Detailed explanation goes here
U_yukawa = Yukawa_Acar * (exp(-Yukawa_alpha*Kd)./Kd);
end

