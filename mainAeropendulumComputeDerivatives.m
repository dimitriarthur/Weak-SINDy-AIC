%% setting parameters

%x0 = [1.5882, 0.00015];
x0=[0 0];
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

% u_control = [linspace(2.7,2.4,1000)...
%              (ones(1,1000))*2.4 ...
%              linspace(2.4,2,1000)...
%              (ones(1,1000))*2 ...
%              linspace(2,2.85,1000)...
%              (ones(1,1000))*2.85 ...
%              linspace(2.85,2,1000)...
%              (ones(1,1000))*2 ...
%              linspace(2,2.5,1000)...
%              (ones(1,1000))*2.5 ...
%              linspace(2.5,1.5,1000)...
%              (ones(1,1000))*1.5 ...
%              linspace(1.5,2.85,1000)...
%              (ones(1,1000))*2.85 ...
%              linspace(2.85,1.3,1000)...
%              (ones(1,1000))*1.3 ...
%              linspace(1.3,0,1000)...
%              (ones(1,1000))*0 ...
%              linspace(0,1.8,1001)...
%              (ones(1,1000))*1.8];
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
%% testing Schroeder cosines
N=10001;
d = 5;
u = zeros(1,N);
phi = zeros(1,d);
w = linspace(1,2,d);
t = 1:N;

for i=2:d
    phi(i) = phi(1)-i*(i-1)*pi/d;
end
         
for i=1:d
    u(1,:) = u(1,:) + cos(w(i)*t+phi(1,i));
end
%% as
u_control=u_control'
%% generating data
[tResult, xResult, u,dx_no_noise] = getTrainingData(t_interval,x0,u_control);
theta = xResult(:,1);
%theta_noisy= sgolayfilt(awgn(theta,30),3,51)
theta_noise = awgn(theta,30)
theta_dot_noise = awgn(dx_no_noise(:,1),30)
xaug=[theta_noise theta_dot_noise u_control(1:size(theta_noise,1))]
Theta=myPoolData(xaug,3,2,1)
Theta_0 = Theta;
tobs=t_interval;
tobs=tobs(:,1:9999)';
K = 120;
p = 2; 
s = 16;
tau=1;
wsindy_params = {s, K, p, tau};
gamma = 0.07;
%% computing derivatives - TVRegDiff
clear dx xt 
for i = 1:size(theta_noise,2)
        dx(:,i) = TVRegDiff( theta_noise(:,i), 10, 0.00000003, [], 'small', 1e12, 0.01, 1, 1 ); %.00002
    end
    dx = dx(2:end,:);
    
    for i = 1:size(theta_noise,2)
        xt(:,i) = cumsum(dx(:,i))*0.01;
        xt(:,i) = xt(:,i) - (mean(xt(50:end-50,i)) - mean(theta_noise(50:end-50,i)));
    end
    xt = xt(50:end-51,:);
    dx = dx(50:end-51,:);  % trim off ends (overly conservative)
    xaug = [xt u(50:end-51)];
    dx(:,size(theta_noise,2)+1) = 0*dx(:,size(theta_noise,2));
   
    
%% computing derivatives - sgolay
%theta_dot_noisy_sgolay = sgolayfilt(gradient(sgolayfilt(theta_noisy,3,51),0.01),3,51)
%theta_double_dot_noisy_sgolay = sgolayfilt(gradient(theta_dot_noisy_sgolay,0.01),3,51)
%dx_sgolay_deriv = [theta_dot_noisy_sgolay movmean(theta_double_dot_noisy_sgolay,100)]
%dx = dx_sgolay_deriv
u_control = u_control'
u_control = u_control(1:9999,:)
xaug = [theta_noisy theta_dot_noisy_sgolay u_control(1:9899,1)]
%% test savitzky golay derivative
%[b,g] = sgolay(10,101)
[b,g] = sgolay(5,101)
subplot(2,1,1)
plot(theta_dot_noisy_sgolay)
hold on
ylabel('$\dot \theta$','Interpreter','Latex')
plot(dx_no_noise(:,1))
ylim([-2 2])
legend('obtained by derivative','ideal')

subplot(2,1,2)
plot(conv(theta_noisy_movmean,factorial(2)/(-0.01)^2*g(:,3),'same'))
ylabel('$\ddot \theta$','Interpreter','Latex')
hold on 
plot(dx_no_noise(:,2))

legend('obtained by derivative','ideal')
%% computing derivative - movmean
theta_noisy_movmean = movmean(theta_noisy,55);
theta_dot_noisy_movmean = movmean(gradient(movmean(theta_noisy,55),0.01),50);
theta_double_dot_noisy_movmean = movmean(gradient(movmean(theta_dot_noisy_movmean,55),0.01),50);

%% composing vector of measurements - movmean
xau_noisy_movmean = [theta_noisy_movmean theta_dot_noisy_movmean];
xaug = [xau_noisy_movmean u];    
dx_noisy_movmean = [theta_dot_noisy_movmean theta_double_dot_noisy_movmean];
dx = dx_noisy_movmean;

%% composing vector of measurements - sgolay
xaug_noisy_filtered_savitzky = [theta_noisy theta_dot_noisy_sgolay u];
xaug = [xaug_noisy_filtered_savitzky u];
dx_noisy_savitzky = [theta_dot_noisy theta_double_dot_noisy];

%% generating theta from xaug
addpath('SINDyC/training')
myTrainSINDYc;

%% composing vector of derivatives
dx_noisy_savitzky = [theta_dot_noisy theta_double_dot_noisy];
dx = dx_noisy_savitzky
%% Obtaining Sparse Representations from Computed Derivatives

% noisy STLS
Xi_STLS_noise = minimize_cost(Theta,0.99,0.01,dx,'STLS');
Xi_sparse_STLS_noise=[{'dtheta','ddtheta'};num2cell(Xi_STLS_noise)];
library_function=[{'coef'};yout];
x_dot_cell_STLS = [library_function,Xi_sparse_STLS_noise]

%% noisy Ridge
Xi_ridge = minimize_cost(Theta,0.99,0.01,dx_no_noise,'Ridge')
Xi_sparse_ridge=[{'dtheta','ddtheta'};num2cell(Xi_ridge)]
library_function=[{'coef'};yout];
x_dot_cell_rige = [library_function,Xi_sparse_ridge]


%% noisy BPDN

Xi_BPDN_noise = minimize_cost(Theta,0.99,0.000001,dx,'BPDN');
Xi_sparse_BPDN_noise=[{'dtheta','ddtheta'};num2cell(Xi_BPDN_noise)];
library_function=[{'coef'};yout];
x_dot_cell_BPDN = [library_function,Xi_sparse_BPDN_noise]

%% noisy WBPDN
Xi_WBPDN_noise = minimize_cost(Theta,0.99,0.01,dx,'WBPDN')
Xi_sparse_WBPDN_noise=[{'dtheta','ddtheta'};num2cell(Xi_WBPDN_noise)]
x_dot_cell_WBPDN_noise = [library_function,Xi_sparse_WBPDN_noise]

%% evaluating on new data

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

[tResult_test,xResult_test,u,dx] = simulateSystem(t_interval,xResult_noise(7000,:),Xi_ridge,u_control_test,n,usesine,polyorder);
[tResult_ode, xResult_ode, u_test,dx] = getTrainingData(t_interval,xResult_noise(7000,:),u_control_test);

%% ploting comparatives of derivatives

subplot(2,1,1)
plot(dx_no_noise(:,1))
ylabel('$\dot \theta$ - rad/s','Interpreter','Latex')
hold on
plot(dx_noisy_movmean(:,1))
legend('ideal','obtained by derivative')

subplot(2,1,2)
plot(dx_no_noise(:,2))
ylabel('$\ddot \theta - rad/s^2$','Interpreter','Latex')
title('$\dot \theta$','Interpreter','Latex')
title('')
hold on
plot(dx_noisy_movmean(:,2))
legend('ideal','obtained by derivative')