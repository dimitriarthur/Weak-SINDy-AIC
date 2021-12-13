%% setting parameters

%x0 = [1.5882, 0.00015];
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
addpath('simulation/models')
addpath('SINDyC/sparsification')

%true dynamics 
Xi_true = [0 0; 0 0; 1 -7.4; 0 0; 0 0; 0 0; 0 0; 0 0; 0 0; 0 2.63; 0 -21.88; 0 0; 0 0; 0 0; 0 0; 0 0];

% get training data
[tResult, xResult, u,dx] = getTrainingData(t_interval,x0,u_control);

%%
%obtaining library of functions
n_samples = size(xResult);
SNR_array = linspace(30,55,30);
%list of mses comparing the true sparse matrix and the one found by sindy
mse_list = zeros(1,30);
%list of mses comparing the resulting dynamics by simulating sparse
%dynamics found
mse_dynamics_theta = zeros(1,30);
mse_dynamics_theta_dot = zeros(1,30);
%to store all sparse matrices found
Xi_STLS_all=Xi_true;

for i=1:length(SNR_array)
    %adding some noise to the signal
    xaug=[awgn(xResult,SNR_array(i)) u_control(1:n_samples(1),:)];
    %lib function
    Theta=myPoolData(xaug,3,2,1);
    Xi_STLS_noise = minimize_cost(Theta,0.9,0.007,awgn(dx,SNR_array(i)),'STLS');
    mse_list(:,i) = mse(Xi_true,Xi_STLS_noise);
    
    [tResult_test,xResult_test,u,dx] = simulateSystem(t_interval,[0 0],Xi_STLS_noise,u_control,n,usesine,polyorder);
    [tResult_ode, xResult_ode, u_test,dx] = getTrainingData(t_interval,[0 0],u_control);
    
    mse_dynamics_theta(:,i) = mse(xResult_test(:,1),xResult_ode(:,1));
    mse_dynamics_theta_dot(:,i) = mse(xResult_test(:,2),xResult_ode(:,2));
    Xi_STLS_all = [Xi_STLS_all Xi_STLS_noise]
    i
end
%% mse plots

subplot(3,1,1)
plot(SNR_array,mse_list,'LineWidth',1.2)
ylim([-10 230])
title('MSE - Sparse Matrix')
ylabel('MSE')
xlabel('SNR Level')
grid
subplot(3,1,2)

plot(SNR_array,mse_dynamics_theta,'LineWidth',1.2)
title('MSE - Dynamics - \theta')
ylabel('MSE')
xlabel('SNR Level')
grid
subplot(3,1,3)

plot(SNR_array,mse_dynamics_theta_dot,'LineWidth',1.2)
title('MSE - Dynamics - d\theta/dt')
ylabel('MSE')
xlabel('SNR Level')
grid
%% computing derivatives via TVRegDiff
%dx_noisy = TVRegDiff( theta_noise, 20, 0.05, [], 'small', 1e-6, 0.01, 0, 0);
%dx_dx_noisy = TVRegDiff(dx_noisy, 20, 0.03, [], 'small', 1e-6, 0.01, 0, 0);
Xi_STLS_all_deriv=Xi_true;

for i=1:length(SNR_array)
    theta_noise = awgn(xResult(:,1),SNR_array(i)); 
    theta_dot_noise = TVRegDiff( theta_noise, 20, 0.05, [], 'small', 1e-6, 0.01, 0, 0);
    theta_double_dot_noise = TVRegDiff( theta_dot_noise, 20, 0.03, [], 'small', 1e-6, 0.01, 0, 0);
    xaug =[theta_noise theta_dot_noise(1:n_samples(1),:) u_control(1:n_samples(1),:)];
    
    Theta=myPoolData(xaug,3,2,1);
    
    Xi_STLS_noise = minimize_cost(Theta,0.9,0.007,[theta_dot_noise(1:9999,:) theta_double_dot_noise(1:9999,:)],'STLS');
    mse_list(:,i) = mse(Xi_true,Xi_STLS_noise);
    
    [tResult_test,xResult_test,u,dx] = simulateSystem(t_interval,[0 0],Xi_STLS_noise,u_control,n,usesine,polyorder);
    [tResult_ode, xResult_ode, u_test,dx] = getTrainingData(t_interval,[0 0],u_control);
    
    mse_dynamics_theta_deriv(:,i) = mse(xResult_test(:,1),xResult_ode(:,1));
    mse_dynamics_theta_dot_deriv(:,i) = mse(xResult_test(:,2),xResult_ode(:,2));
    Xi_STLS_all_deriv = [Xi_STLS_all_deriv Xi_STLS_noise]
    i
end
%% mse plots

subplot(3,1,1)
plot(SNR_array,mse_list,'LineWidth',1.2)
ylim([-10 230])
title('MSE - Sparse Matrix')
ylabel('MSE')
xlabel('SNR Level')
grid
subplot(3,1,2)

plot(SNR_array,mse_dynamics_theta_deriv,'LineWidth',1.2)
title('MSE - Dynamics - \theta')
ylabel('MSE')
xlabel('SNR Level')
grid
subplot(3,1,3)

plot(SNR_array,mse_dynamics_theta_dot_deriv,'LineWidth',1.2)
title('MSE - Dynamics - d\theta/dt')
ylabel('MSE')
xlabel('SNR Level')
grid


%% Measuring theta - considering that the dot theta can be measured
addpath('..\WSINDy')    
theta_noise = awgn(xResult(:,1),40)
theta_dot_noise = awgn(dx(:,1),40)
%% library and execution
xaug=[movmean(theta_noise,20) theta_dot_noise u_control(1:size(theta_noise,1))];
Theta=myPoolData(xaug,3,2,1)
Theta_0 = Theta;
tobs=t_interval;
tobs=tobs(:,1:9999)';
n=2;
K = 100 ; %120
p = 2; %2
s = 16; %16
tau=1;
wsindy_params = {s, K, p, tau};
gamma = 0.07;
test
w_sparse

%% preprocessing - 
%theta_dot_noise = TVRegDiff( theta_noise, 20, 0.0009, [], 'small', 1e-6, 0.01, 0, 0); %0.0009
%xaug=[movmean(theta_noise,40) theta_dot_noise(1:size(theta_noise,1)) u_control(1:size(theta_noise,1))];
xaug=[movmean(theta_noise,40) dx(:,1) u_control(1:size(theta_noise,1))];
Theta=myPoolData(xaug,3,2,1);


%% Measuring Theta FROM REAL PLANT - considering that theta dot must be computed - MSE 0.0585

addpath('..\WSINDy')    
theta_noise=awgn(xResult(:,1),40);
theta_dot_noise = TVRegDiff(theta_noise, 10, 0.0009, [], 'small', 1e-12, 0.01,0,0); %0.0009
xaug=[movmean(theta_noise,40) theta_dot_noise(1:size(theta_noise,1)) u_control(1:size(theta_noise,1))];
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
wsindy_params = {s, K, p, tau};
lambda=0.9;
n=2;
gamma = 0.1;
test
w_sparse

%% Measuring Theta - considering that theta dot must be computed - MSE 0.0489

addpath('..\WSINDy')    
theta_dot_noise = TVRegDiff( theta_noise, 20, 0.21, [], 'small', 1e-6, 0.01, 0, 0); %0.0009
xaug=[theta_noise theta_dot_noise(1:size(theta_noise,1)) u_control(1:size(theta_noise,1))];
Theta=myPoolData(xaug,3,2,1);
Theta_0 = Theta;
tobs=t_interval;
tobs=tobs(:,1:9999)';
K = 120 ; %120
p = 2; %2
s = 16; %16
tau=1;
wsindy_params = {s, K, p, tau};
gamma = 0.007;
gamma_array=linspace(0.0001,0.8,100);
mse_list_gamma=zeros(1,max(size(gamma_array)));

for i=1:length(gamma_array)
   gamma=gamma_array(i);
   test
   mse(w_sparse,Xi_true);
   mse_list_gamma(:,i) = mse(w_sparse,Xi_true);
   i
end


