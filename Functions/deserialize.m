function z = deserialize(u)

    i=2;j=5;k=2;

    x = u(1:(i*j*k));
    obstacls = reshape(x,[j*k,i]);
    
    z = zeros(5,2,2);
    for n = 1:i
        obstacl = obstacls(:,n);
        obstacl = transpose(reshape(obstacl,[k,j]))
        z(:,:,n) = obstacl;
    end