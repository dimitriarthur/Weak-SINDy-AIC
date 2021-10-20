function x_dot = AeropendulumFromSparseDynamics(t,x,x0,Xi,u,n,usesine,polyorder)
    library = myPoolData([x0 u],n,polyorder,usesine);
    x_dot= library*Xi;
    x_dot=x_dot';
end