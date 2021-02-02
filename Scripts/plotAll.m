function plotAll(x_curr, x_ref_curr, obstcl, ellip_coeff, x_pred, mv_pred, ego_car, P_des)

    clf;
    hold on;     

    %Plot Ego car
    if ~isscalar(x_curr)
        scatter(x_curr(1),x_curr(2), 'd')
        pgon = polyshape(ego_car(:,1),ego_car(:,2));
        plot(pgon);
    end

    %Plot MPC Referece  
    if ~isscalar(x_ref_curr)
        scatter(x_ref_curr(1),x_ref_curr(2),'x','MarkerFaceColor',[0 .7 .7])
    end
    
    %Plot Desired position  
    if ~isscalar(P_des)
        scatter(P_des(1),P_des(2),'x','MarkerFaceColor',[0 .7 .7])
    end

    %Plot MPC predicted path      
    if ~isscalar(obstcl) && nnz(x_pred) ~= 0
        scatter(x_pred(:,1),x_pred(:,2),'filled')
    end

    %Plot obstacle
    if ~isscalar(obstcl) && nnz(obstcl) ~= 0
        pgon = polyshape([obstcl(1)-(obstcl(3)/2), obstcl(1)-(obstcl(3)/2), obstcl(1)+(obstcl(3)/2), obstcl(1)+(obstcl(3)/2)],...
                         [obstcl(2)-(obstcl(4)/2), obstcl(2)+(obstcl(4)/2), obstcl(2)+(obstcl(4)/2), obstcl(2)-(obstcl(4)/2)]);
        %Plot fitted ellipse
        n=ellip_coeff(6);
        a = ellip_coeff(1) ; b = ellip_coeff(2); xe = ellip_coeff(3); ye = ellip_coeff(4); phi = -ellip_coeff(5);
        plgn = rotate(pgon,obstcl(5),[xe,ye]);        
        ellip = @(x,y) ((((x-xe).*cos(phi) - (y-ye).*sin(phi)))./a).^n + ((((x-xe).*sin(phi) + (y-ye).*cos(phi)))./b).^n - 1;

        plot(plgn)
        fimplicit(ellip)
    end
end