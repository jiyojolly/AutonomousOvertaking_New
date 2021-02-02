function [G,Gmv,Ge] = constraint_obstcl_avoid_jacobian(X,U,e,data, ellip_coeff)
    p = data.PredictionHorizon;
    Nx = data.NumOfStates;
    Nc = data.PredictionHorizon;
    G = zeros(p,Nx,Nc);
    Nmv = length(data.MVIndex);
    Gmv = zeros(p,Nmv,Nc);
    Ge = zeros(p,1);
%     size(X)
    if nnz(ellip_coeff(1:5)) ~= 0
        x1 = X(2:p+1,1);
        y1 = X(2:p+1,2);
        
        n=ellip_coeff(6);
open_system('AEBTestBenchExample/AEB with Sensor Fusion')
                      
        G(1:p,1,1:p) = diag(ellip_dx(x1,y1));
        G(1:p,2,1:p) = diag(ellip_dy(x1,y1));
    end
end