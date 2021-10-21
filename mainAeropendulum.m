%% setting parameters

x0 = [1.5882, 0.00015];
t_interval =[0:0.01:100];
%pre-defining an exogenous control input
u_control = [(ones(1,1000))*2.7 ...
             (ones(1,1000))*2 ...
             (ones(1,1000))*2.85 ...
             (ones(1,1000))*2 ...
             (ones(1,1000))*2.5 ...
             (ones(1,1000))*1.5 ...
             (ones(1,1000))*2.85 ...
             (ones(1,1000))*1.3 ...
             (zeros(1,1001)) ...
             (ones(1,1000))*1.8];
         
%% generating data
[tResult, xResult, u,dx] = getTrainingData(t_interval,x0,u_control);
xaug=[xResult u];
%% SINDYc parameters
ModelName = 'SINDYc'; iModel = 1;
Nvar = 3;
n=3;
polyorder = 2;
usesine = 1;
lambda = 0.1;
eps = 0;
%% call training script

my_trainSINDYc
Xi_sparse=[{'theta','dtheta'};num2cell(Xi)]
library_function=[{'coef'};yout];
x_dot_cell = [library_function,Xi_sparse];

%% predicting on new data 
u_control_test = [(ones(1,1000))*1.2 ...
             (ones(1,1000))*2.5 ...
             (ones(1,1000))*2 ...
             (ones(1,1000))*2.75 ...
             (ones(1,1000))*2.85 ...
             (ones(1,1000))*1.2 ...
             (ones(1,1000))*2.7 ...
             (ones(1,1000))*2.5 ...
             (zeros(1,1001)) ...
             (ones(1,1000))*2.85];

[tResult_test,xResult_test,u,dx] = simulateSystem(t_interval,xResult(7000,:),Xi,u_control_test,n,usesine,polyorder);
[tResult_ode, xResult_ode, u_test,dx] = getTrainingData(t_interval,x0,u_control_test);

% to compute AIC
% - vary lambda and obtain multiple models
% - compute error - validation for these models. return error
% - calculate a ranking based on AIC
%%

subplot(2,2,1)
plot(tResult,xResult(:,1),'LineWidth',1)
grid
hold on 
plot(tResult,xResult(:,2),'LineWidth',1)
title('Data generated')
legend('$\theta$','$\dot \theta$','Interpreter','Latex')
%xlim()
hold off
subplot(2,2,2)
plot(tResult,u_control(1:9999))
title('Control Signal - Training')
grid

subplot(2,2,3)
plot(tResult,xResult_ode(:,1),'LineWidth',1)
grid
hold on 
plot(tResult,xResult_ode(:,2),'LineWidth',1)
hold on
plot(tResult,xResult_test(:,1),'LineWidth',1)
hold on
plot(tResult,xResult_test(:,2),'LineWidth',1)
hold off
title('\theta and \dot \theta - ODE vs SINDyC. MSE = 2.8166e-04','fontweight','bold')
legend('$\theta_{ODE}$','$\dot \theta_{ODE}$','$\theta_{SINDYc}$','$\dot \theta_{SINDYc}$','Interpreter','Latex')

subplot(2,2,4)
plot(tResult,u_control_test(1:9999))
title('Control Signal - Test')
grid
%% 