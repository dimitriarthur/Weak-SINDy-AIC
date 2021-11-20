function [tResult,xResult,u,dx] = getTrainingData(t_interval,x0,u_control)
   addpath('models')
   tResult = [];
   xResult = [];
   dx=zeros(numel(t_interval)-2,2);
   u=[];
   x0_orig=x0;
   a=1;
    for i=1:numel(t_interval)-2
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
    
    xResult_aux = [x0_orig; xResult];
    %computing derivatives 
    for i=2:numel(t_interval)-1
       dx(i-1,:)=Aeropendulum(0.1,xResult_aux(i,:),u_control(i))';
    end
end