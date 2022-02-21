%% setting parameters

%x0 = [1.5882, 0.00015];
x0 = [-0.1 0];
t_interval =[0:0.01:100];
%pre-defining an exogenous control input
% u_control = [(ones(1,1000))*2.85 ...
%              (ones(1,1000))*2 ...
%              (ones(1,1000))*2.85 ...
%              (ones(1,1000))*2 ...
%              (ones(1,1000))*2.5 ...
%              (ones(1,1000))*1.5 ...
%              (ones(1,1000))*2.85 ...
%              (ones(1,1000))*1.3 ...
%              (zeros(1,1001)) ...
%              (ones(1,1000))*1.8];
 u_control = [(ones(1,1000))*2 ...
             (ones(1,1000))*2.4 ...
             (ones(1,1000))*2.8 ...
             (ones(1,1000))*2.2 ...
             (ones(1,1000))*2.9 ...
             (ones(1,1000))*1.9 ...
             (ones(1,1000))*2.6 ...
             (ones(1,1000))*2.3 ...
             (zeros(1,1001))*1.2 ...
             (ones(1,1000))*2.5];         
% u_control = [(0.5+0.5*tanh((linspace(-10,10,1000)-1)/3))*2.85 ... 
%              (0.5-0.5*tanh((linspace(-10,10,1000)-1)/3))*2.85 ...
%              (0.5+0.5*tanh((linspace(-10,10,1000)-1)/3))*2.2 ...
%              (0.5-0.5*tanh((linspace(-10,10,1000)-1)/3))*2.2...
%              (0.5+0.5*tanh((linspace(-10,10,1000)-1)/3))*2.76 ...
%              (0.5-0.5*tanh((linspace(-10,10,1000)-1)/3))*2.76 ...      
%              (0.5+0.5*tanh((linspace(-10,10,1000)-1)/3))*2.5 ...
%              (0.5-0.5*tanh((linspace(-10,10,1000)-1)/3))*2.5 ...
%              (0.5+0.5*tanh((linspace(-10,10,1000)-1)/3))*2.75 ...
%              (0.5-0.5*tanh((linspace(-10,10,1001)-1)/3))*2.75];
%u_control = [ones(2500,1)*2 ...
%             ones(2500,1)*2.4];             
%% testing Schroeder cosines
N=10001;
d = 100;
u = zeros(1,N);
phi = zeros(1,d);
w = linspace(1,2,d);
t = 1:N;

for i=2:d
    phi(i) = phi(1)-i*(i-1)*pi/d;
end
         
for i=1:d
    u(1,:) = u(1,:) + 2.75*cos(w(i)*t+phi(1,i));
end

%% generating data
addpath('simulation/models')
addpath('simulation')
[tResult, xResult, u,dx] = getTrainingData(t_interval,x0,u_control);


%% SINDYc parameters
ModelName = 'SINDYc'; iModel = 1;
Nvar = 3;
n=3;
polyorder = 2;
usesine = 1;
lambda = 0.1;
eps = 0;
%% call training script
addpath('SINDyC/training')
addpath('SINDyC/library_function')

[yout,Xi,Theta] = myTrainSINDYc(xResult,dx,u,polyorder,n,usesine,lambda);
Xi_sparse=[{'dtheta','ddtheta'};num2cell(Xi)]
library_function=[{'coef'};yout];
x_dot_cell = [library_function,Xi_sparse]
Xi_true = cell2mat(x_dot_cell(2:end,2:3));
Xi_true(3,:)=[1 -7.4];
Xi_true(10:11,2)=[2.627;-21.88]
Xi_sparse_true=[{'dtheta','ddtheta'};num2cell(Xi_true)]
x_dot_cell_true = [library_function,Xi_sparse_true]
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

% to compute AIC - answers our previous question "how to determine the best
% model objectively?"
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
%% testing other cost functions
record_amount = size(Theta)
Xi_ridge = minimize_cost(Theta,0.98,0.0001,dx(1:record_amount(1),:),'Ridge')
Xi_sparse_ridge=[{'dtheta','ddtheta'};num2cell(Xi_ridge)]
library_function=[{'coef'};yout];
x_dot_cell_rige = [library_function,Xi_sparse_ridge]

%% STLS
Xi_STLS = minimize_cost(Theta,0.99,0.001,dx(1:record_amount(1),:),'STLS')
Xi_sparse_STLS=[{'dtheta','ddtheta'};num2cell(Xi_STLS)]
x_dot_cell_STLS = [library_function,Xi_sparse_STLS]

%% BPDN
Xi_BPDN = minimize_cost(Theta,0.9,0.0001,dx(1:record_amount(1),:),'BPDN')
Xi_sparse_BPDN=[{'dtheta','ddtheta'};num2cell(Xi_BPDN)]
x_dot_cell_BPDN = [library_function,Xi_sparse_BPDN]

%% WBPDN
Xi_WBPDN = minimize_cost(Theta,0.9,0.0001,dx(1:record_amount(1),:),'WBPDN')
Xi_sparse_WBPDN=[{'dtheta','ddtheta'};num2cell(Xi_WBPDN)]
x_dot_cell_WBPDN = [library_function,Xi_sparse_WBPDN]

%% adding noise and seeing what happens
addpath('SINDyC/training')
xResult_noise= awgn(xResult,40);
%xResult_noise=movmean(xResult_noise,50)
xaug=[xResult_noise u];
xaug_noisy = xaug
myTrainSINDYc;

Xi_ridge_noise = minimize_cost(Theta,0.98,0.01,dx,'Ridge');
Xi_sparse_ridge_noise=[{'dtheta','ddtheta'};num2cell(Xi_ridge_noise)];
library_function=[{'coef'};yout];
x_dot_cell_ridge = [library_function,Xi_sparse_ridge_noise]

%% computing derivatives
theta_dot = movmean(diff(movmean(xaug_noisy(:,1),50)),50)
theta_double_dot = movmean(diff(theta_dot),50)
dx_noisy = [theta_dot(1:9997,:) theta_double_dot]

%% ploting non corrupted dynamics
subplot(3,1,1)
plot(xaug_non_noisy(:,1),'LineWidth',1.1)
title('$\theta(t)$','Interpreter','Latex','Fontsize',14)
ylabel('rad')
xlabel('Sample Time')

subplot(3,1,2)
plot(xaug_non_noisy(:,2),'LineWidth',1.1)
title('$\dot \theta(t)$','Interpreter','Latex','Fontsize',14)
ylabel('rad/s')
xlabel('Sample Time')

subplot(3,1,3)
plot(dx(:,2),'LineWidth',1.1)
title('$u$','Interpreter','Latex','Fontsize',14)
ylabel('rad')
xlabel('Sample Time')



%% ploting derivatives and smoothed derivative
figure()
plot(xaug_noisy(:,1))
title('\theta - noisy')
xlabel('Sample Time')
ylabel('rad')

figure()

subplot(3,2,1)
plot(diff(movmean(xaug_noisy(:,1),50)))
title('$\dot \theta$ - noisy','Interpreter','Latex')

subplot(3,2,3)
plot(movmean(diff(movmean(xaug_noisy(:,1),50)),50),'LineWidth',1.2)
title('$\dot \theta$ - moving avg','Interpreter','Latex')

subplot(3,2,2)
plot(diff(theta_dot))
ylim([-4e-4,3e-4])
title('$\ddot \theta$ - from $\dot \theta$ moving avg','Interpreter','Latex')

subplot(3,2,4)
plot(movmean(diff(theta_dot),50),'LineWidth',1.2)
ylim([-4e-4,3e-4])
title('$\ddot \theta$ - smoothed','Interpreter','Latex')

subplot(3,2,5)
plot(dx(:,1))
title('$\dot \theta$ - ideal','Interpreter','Latex')

subplot(3,2,6)
plot(dx(:,2))
title('$\ddot \theta$ - ideal','Interpreter','Latex')

%% noisy STLS
Xi_STLS_noise = minimize_cost(Theta(1:9997,:),0.01,0.01,dx_noisy,'STLS');
Xi_sparse_STLS_noise=[{'dtheta','ddtheta'};num2cell(Xi_STLS_noise)];
library_function=[{'coef'};yout];
x_dot_cell_STLS = [library_function,Xi_sparse_STLS_noise]

%% noisy Ridge
Xi_ridge = minimize_cost(Theta(1:9997,:),0.01,0.0001,dx_noisy,'Ridge')
Xi_sparse_ridge=[{'dtheta','ddtheta'};num2cell(Xi_ridge)]
library_function=[{'coef'};yout];
x_dot_cell_rige = [library_function,Xi_sparse_ridge]


%% noisy BPDN

Xi_BPDN_noise = minimize_cost(Theta(1:9997,:),0.99,0.001,dx_noisy,'BPDN');
Xi_sparse_BPDN_noise=[{'dtheta','ddtheta'};num2cell(Xi_BPDN_noise)];
library_function=[{'coef'};yout];
x_dot_cell_BPDN = [library_function,Xi_sparse_BPDN_noise]

%% noisy WBPDN
Xi_WBPDN_noise = minimize_cost(Theta,0.02,0.01,dx_noisy,'WBPDN')
Xi_sparse_WBPDN_noise=[{'dtheta','ddtheta'};num2cell(Xi_WBPDN_noise)]
x_dot_cell_WBPDN_noise = [library_function,Xi_sparse_WBPDN_noise]

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

[tResult_test,xResult_test,u,dx] = simulateSystem(t_interval,xResult_noise(7000,:),Xi_WBPDN_noise,u_control_test,n,usesine,polyorder);
[tResult_ode, xResult_ode, u_test,dx] = getTrainingData(t_interval,xResult_noise(7000,:),u_control_test);




%% supposing that only theta is measurable - computing derivatives with gradient
theta = sgolayfilt(xaug_noisy(:,1),3,51)
theta_dot =  sgolayfilt(gradient(sgolayfilt(xaug_noisy(:,1),3,51),0.01),3,51)
xaug_noisy_filtered_savitzky = [theta theta_dot u]
xaug=xaug_noisy_filtered_savitzky
%% computing angular acceleration
theta_double_dot = sgolayfilt(gradient(theta_dot,0.01),3,51)

%% composing dx_computed vector
dx_noisy_savitzky = [theta_dot theta_double_dot]
dx = dx_noisy_savitzky
%% training sindyc

my_trainSINDYc
Xi_ridge = minimize_cost(Theta,0.98,0.001,dx,'Ridge')
Xi_sparse_ridge=[{'dtheta','ddtheta'};num2cell(Xi_ridge)]
library_function=[{'coef'};yout];
x_dot_cell_rige = [library_function,Xi_sparse_ridge]

%% simulate on validation

[tResult_test,xResult_test,u,dx] = simulateSystem(t_interval,xResult(7000,:),Xi_ridge,u_control_test,n,usesine,polyorder);
[tResult_ode, xResult_ode, u_test,dx] = getTrainingData(t_interval,x0,u_control_test);
%% only output
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
