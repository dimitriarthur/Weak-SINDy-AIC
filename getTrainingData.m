function [tResult,xResult,u] = getTrainingData(t_interval,x0,u_control)
   tResult = [];
   xResult = [];
   u=[];
   
    for i=1:numel(t_interval)-1
       %handler to generate the specified control inputs 
       lrz = @(t,x) Aeropendulum(t,x,u_control(i));
       %integration interval
       t=t_interval(i:i+1);
       %solve the ODE for the specified conditions
       [t,x]=ode45(lrz,t,x0);

       %managing variables 
       u=cat(1,u,u_control(i));
       tResult = cat(1, tResult, t(end));
       xResult = cat(1, xResult, x(end,:));

       x0=x(end,:);
    end
    
end