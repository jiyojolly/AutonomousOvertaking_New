function ellip_coeff = getEllipseCoeff(obstcl, obstcl_ellip_order, car_len, car_wid, inflation_factor)
    
    ellip_coeff = zeros(6,1);
    if nnz(obstcl) ~= 0
        f = nthroot(2,obstcl_ellip_order);          
         
        ellip_coeff(1:5) = [((f*obstcl(3)/2)+(car_len/2))*inflation_factor, ((f*obstcl(4)/2)+(car_wid/2))*inflation_factor, obstcl(1), obstcl(2), deg2rad(obstcl(5))];
        ellip_coeff(6) = obstcl_ellip_order;
    end
end

