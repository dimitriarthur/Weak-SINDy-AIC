function x_dot = Aeropendulum(t,x,u)
    xdot=zeros(2,1);
    theta=x(1);
    dtheta=x(2);
    m=0.18;
    l=0.14 ;
    g=9.81;
    c = 0.0836;
    km = 0.2120;
    J = 0.0113;

    x_dot=[dtheta;-c/J*dtheta - (m*l*g*sin(theta))/J + km*l/J*u.^2];
end 