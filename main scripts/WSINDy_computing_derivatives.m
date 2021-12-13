%% setting parameters

x0=[0 0];
t_interval =[0:0.01:100];
%pre-defining an exogenous control input
u_control = [(ones(1,1000))*2.8 ...
             (ones(1,1000))*2.3 ...
             (ones(1,1000))*2.85 ...
             (ones(1,1000))*2.2 ...
             (ones(1,1000))*2.5 ...
             (ones(1,1000))*1.2 ...
             (ones(1,1000))*2.85 ...
             (ones(1,1000))*1.3 ...
             (zeros(1,1001)) ...
             (ones(1,1000))*1.8];
u_control = u_control';  

%% SINDYc parameters
ModelName = 'SINDYc'; iModel = 1;
Nvar = 3;
n=3;
polyorder = 2;
usesine = 1;
lambda = 0.1;
eps = 0;

%% generate data and execute sindy
addpath('../simulation/models')
addpath('../SINDyC/sparsification')

% get training data
[tResult, xResult, u,dx] = getTrainingData(t_interval,x0,u_control);


%% Measuring Theta FROM REAL PLANT - considering that theta dot must be computed - MSE 0.0585

addpath('..\WSINDy')    
theta_noise=awgn(xResult(:,1),40);
theta_dot_noise = TVRegDiff(theta_noise, 10, 0.0009, [], 'small', 1e-12, 0.01,0,0); %0.0009
xaug=[movmean(theta_noise,40) movmean(theta_dot_noise(1:size(theta_noise,1)),10) u_control(1:size(theta_noise,1))];
%xaug=[simout_OL.signals.values(:,2) theta_dot_noise(1:5001,:) simout_OL.signals.values(:,1)];
Theta=myPoolData(xaug,3,2,1);
Theta_0 = Theta;
tobs=t_interval;
tobs=tobs(:,1:9999)';
xobs=xaug(1:9999,:);
K = 120 ; %120
p = 2; %2
s = 16; %16
tau=1;
r_whm=30;
useGLS=1;
scale_Theta=0;
wsindy_params = {s, K, p, tau};
lambda=0.9;
n=2;
gamma = 0.1;
test
w_sparse;
% yin,ahat,nVars,polyorder,useFourier
yout = myPoolDataVariableNames(str_vars,w_sparse,3,2,1);
Xi_sparse=[{'dtheta','ddtheta'};num2cell(w_sparse)]
library_function=[{'coef'};yout];
x_dot_cell = [library_function,Xi_sparse]
