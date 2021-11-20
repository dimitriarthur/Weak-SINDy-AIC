function x_dot = getValidationData(t,x,x0,Xi,u,n,usesine,polyorder)
    %generate a library function with states and u to compute the matrix
    %                   \dot x = \theta_{lib}*\xi
    library = myPoolData([x0 u],n,polyorder,usesine);
    x_dot= library*Xi;
    x_dot=x_dot';
end