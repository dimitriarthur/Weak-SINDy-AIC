function [tResult,xResult,u,dx] = simulateSystem(t_interval,x0,Xi,u_control,n,usesine,polyorder)
   tResult = [];
   xResult = [];
   dx=zeros(numel(t_interval)-2,2);
   u=[];
   a = 1;
   
   for i=1:numel(t_interval)-2
       %handler to generate the specified control inputs 
       lrz = @(t,x) getValidationData(t,x,x0,Xi,u_control(i),n,usesine,polyorder);
       %integration interval
       t=t_interval(i:i+1);
       %solve the ODE for the specified conditions
       [t,x]=ode45(lrz,t,x0)

       %managing variables 
       u=cat(1,u,u_control(i));
       tResult = cat(1, tResult, t(end));
       xResult = cat(1, xResult, x(end,:));

       x0=x(end,:);
   end
   
end