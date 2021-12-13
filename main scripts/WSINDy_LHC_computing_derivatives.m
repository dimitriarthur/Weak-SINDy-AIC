addpath('../excitation signals')
LHC_amplitude_Thold
x0=[0 0];
sampling_time=0.01;
t_interval = linspace(0,sum(random_design_points(:,2)), sum(random_design_points(:,2))/sampling_time);
[tResult, xResult, u,dx] = getTrainingData(t_interval,x0,u_aprbs);
u_aprbs=u_aprbs';

subplot(3,1,1)
plot(u_aprbs)
grid
ylabel('$u(t)$','Interpreter','Latex')
subplot(3,1,2)
plot(xResult(:,1))
grid
ylabel('$\theta(rad)$','Interpreter','latex')
subplot(3,1,3)
plot(dx(:,1))
grid
ylabel('$\dot \theta(rad)$','Interpreter','latex')
xlabel('Sampling Instant')

%% Measuring Theta FROM REAL PLANT - considering that theta dot must be computed - MSE 0.0585

addpath('..\WSINDy')    
theta_noise=awgn(xResult(:,1),40);
theta_dot_noise = TVRegDiff(theta_noise, 10, 0.009, [], 'small', 1e-12, 0.01,0,0); %0.0009

%% plotting derivatives comparative 


%%
xaug=[movmean(theta_noise,40) (theta_dot_noise(1:size(theta_noise,1))) u_aprbs(1:size(theta_noise,1))];
Theta=myPoolData(xaug,3,2,1);
Theta_0 = Theta;
tobs=t_interval;
tobs=tobs(:,1:max(size(xaug)))';
xobs=xaug(1:max(size(xaug)),:);
K = 120 ; %120
p = 2; %2
s = 16; %16
tau=1;
r_whm=30;
useGLS=1;
scale_Theta=0;
wsindy_params = {s, K, p, tau};
lambda=0.99;
n=2;
gamma = 0.17;
test
w_sparse
str_vars={'theta','dtheta','u'};

% function calling - parameters order: yin,ahat,nVars,polyorder,useFourier
yout = myPoolDataVariableNames(str_vars,w_sparse,3,2,1);
Xi_sparse=[{'dtheta','ddtheta'};num2cell(w_sparse)]
library_function=[{'coef'};yout];
x_dot_cell = [library_function,Xi_sparse]