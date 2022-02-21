function x_dot = AeropendulumSINDy(t,x,u)
    xdot=zeros(2,1);
    theta=x(1);
    dtheta=x(2);
    %x_dot=[1.2314*dtheta ; -2.83*theta - 1.847*theta*u + 1.0691*u.^2 - 2.8323*sin(theta) + 1.9051*cos(u)];
    %x_dot=[1.0040*dtheta ; -3.21 - 12.62*theta + 7.59*dtheta - 4.66*theta.^2 + 14.32*theta*dtheta + 1.6*theta*u - 7.48*dtheta*u + 3.2*u.^2 - 11.6*sin(theta) - 5.39*cos(theta) -3.42*sin(dtheta) + 5.36*cos(dtheta) - 8.53*sin(u) + 6*cos(u)];
    x_dot = [0.994*dtheta; -2.05*theta + 3.33*u - 2.74*theta.^2 + 2.47*theta*u + 3.28*u.^2 + 2.93*sin(dtheta) + 1.18*cos(dtheta) - 1.53*cos(u)];
end 