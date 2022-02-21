%% setting parameters

x0=[0 0];
t_interval = linspace(0,22.5,2250);

%% SINDYc parameters
ModelName = 'SINDYc'; iModel = 1;
Nvar = 3;
n=3;
polyorder = 2;
usesine = 1;
lambda = 0.1;
eps = 0;

%% reading from arduino experiment
addpath("..\Experiment Results")
experiment_13_01_22 = load('experiment_04_01_22.mat');

%%
addpath('..\derivatives')
u_control = (experiment_04_01_22(:,1));
%normalizing in respect to the maximum value (255)
u_control = u_control/255;
theta_noise = experiment_04_01_22(:,2);
% feature engineering
% removing the initial measurement error 0deg is 220 deg in first msrmnt
%%
addpath('..\derivatives')
u_control = data_experiment(:,1);
u_control = u_control/255;
theta_noise = movmean(data_experiment(:,2)*pi/180,20);
theta_dot_noise = (TVRegDiff(theta_noise, 10, 0.000003, [], 'small', 1e-12, 0.01,0,0));

%%
theta_noise = theta_noise - 220;
% converting to rad
%%
theta_noise = theta_noise*pi/180;
theta_dot_noise = (TVRegDiff(theta_noise, 10, 0.0000003, [], 'small', 1e-12, 0.01,0,0));

%% plots
subplot(3,1,1)
plot((theta_noise))
title('$\theta (rad)$ - measured', 'Interpreter', 'latex')
subplot(3,1,2)
plot((theta_dot_noise))
title('$\dot \theta $- raw TVRegdiff', 'Interpreter', 'latex')
subplot(3,1,3)
plot(movmean(theta_dot_noise,5))
title('$\dot \theta $- movmean filter', 'Interpreter', 'latex')

%% composing library function
addpath('..\SINDyC\library_function')
xaug=[(theta_noise) (theta_dot_noise(1:size(theta_noise,1))) u_control(1:size(theta_noise,1))];
Theta=myPoolData(xaug,3,2,1);
Theta_0 = Theta;

%% running algorithms from arduino data
t_interval = linspace(0,14.09,1409);
tobs=t_interval;
tobs=tobs';
xobs=xaug(:,1:2);

%%
addpath('..\WSINDy')  
addpath('..\derivatives')
K = 120 ; %120
p = 3; %2
s = 16; %16
tau=2;
r_whm=30;
useGLS=1;
scale_Theta=0;
wsindy_params = {s, K, p, tau};
lambda=0.9;
n=2;
gamma = 0.3;
test
w_sparse;
% yin,ahat,nVars,polyorder,useFourier
yout = myPoolDataVariableNames({'theta','dtheta','u'},w_sparse,3,2,1);
Xi_sparse=[{'dtheta','ddtheta'};num2cell(w_sparse)]
library_function=[{'coef'};yout];
x_dot_cell = [library_function,Xi_sparse]
