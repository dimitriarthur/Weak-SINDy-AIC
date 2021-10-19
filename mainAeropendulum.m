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
%%

figure()
plot(tResult,xResult(:,1),'LineWidth',1)
grid
hold on 
plot(tResult,xResult(:,2),'LineWidth',1)
legend('$\theta$','$\dot \theta$','Interpreter','Latex')
%xlim()
hold off

%% 