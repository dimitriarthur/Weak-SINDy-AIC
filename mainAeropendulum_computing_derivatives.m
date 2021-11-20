%% defining a smoother control signal
x0 = [0 0];
t_interval =[0:0.01:100];

u_control_non_smooth = [(ones(1,1000))*2.85 ...
             (ones(1,1000))*2 ...
             (ones(1,1000))*2.85 ...
             (ones(1,1000))*2 ...
             (ones(1,1000))*2.5 ...
             (ones(1,1000))*1.5 ...
             (ones(1,1000))*2.85 ...
             (ones(1,1000))*1.3 ...
             (zeros(1,1001)) ...
             (ones(1,1000))*1.8];
         
         
u_control = [(0.5+0.5*tanh((linspace(-10,10,1000)-1)/3))*2.85 ... 
             (0.5-0.5*tanh((linspace(-10,10,1000)-1)/3))*2.85 ...
             (0.5+0.5*tanh((linspace(-10,10,1000)-1)/3))*1.8 ...
             (0.5-0.5*tanh((linspace(-10,10,1000)-1)/3))*1.8...
             (0.5+0.5*tanh((linspace(-10,10,1000)-1)/3))*2.76 ...
             (0.5-0.5*tanh((linspace(-10,10,1000)-1)/3))*2.76 ...      
             (0.5+0.5*tanh((linspace(-10,10,1000)-1)/3))*2.5 ...
             (0.5-0.5*tanh((linspace(-10,10,1000)-1)/3))*2.5 ...
             (0.5+0.5*tanh((linspace(-10,10,1000)-1)/3))*2.6 ...
             (0.5-0.5*tanh((linspace(-10,10,1001)-1)/3))*2.6];
         
%% generating data
[~, xResult_non_smooth, ~,dx_non_smooth] = getTrainingData(t_interval,x0,u_control_non_smooth);
[tResult, xResult, u,dx] = getTrainingData(t_interval,x0,u_control);
%% SINDYc parameters
ModelName = 'SINDYc'; iModel = 1;
Nvar = 3;
n=3;
polyorder = 2;
usesine = 1;
lambda = 0.1;
eps = 0;

%% visualizing data
subplot(4,2,1)
plot(u_control_non_smooth,'LineWidth',1)
ylabel('$u_{ns}(t)$','Interpreter','Latex')
xlim([0 10000])
grid

subplot(4,2,3)
plot(xResult_non_smooth(:,1),'LineWidth',1,'color','#D95319')
ylabel('$\theta(rad)$','Interpreter','Latex')
xlim([0 10000])
grid

subplot(4,2,5)
plot(xResult_non_smooth(:,2),'LineWidth',1,'color','#7E2F8E')
ylabel('$\dot \theta(rad)$','Interpreter','Latex')
xlim([0 10000])
grid

subplot(4,2,7)
plot(dx_non_smooth(:,2),'LineWidth',1,'color','#77AC30')
ylabel('$\ddot \theta(rad)$','Interpreter','Latex')
xlim([0 10000])
grid

subplot(4,2,2)
plot(u_control,'LineWidth',1)
ylabel('$u(t)$','Interpreter','Latex')
xlim([0 10000])
grid

subplot(4,2,4)
plot(xResult(:,1),'LineWidth',1,'color','#D95319')
ylabel('$\theta(rad)$','Interpreter','Latex')
grid

subplot(4,2,6)
plot(xResult(:,2),'LineWidth',1,'color','#7E2F8E')
ylabel('$\dot \theta(rad)$','Interpreter','Latex')
grid

subplot(4,2,8)
plot(dx(:,2),'LineWidth',1,'color','#7E2F8E')
ylabel('$\dot \theta(rad)$','Interpreter','Latex')
grid

%% generating Theta 
[yout,Xi,Theta] = myTrainSINDYc(xResult,dx,u,polyorder,n,usesine,lambda);

%% STLS
record_amount = size(Theta)
Xi_STLS = minimize_cost(Theta,0.99,0.001,dx(1:record_amount(1),:),'STLS')
Xi_sparse_STLS=[{'dtheta','ddtheta'};num2cell(Xi_STLS)];
library_function=[{'coef'};yout];
x_dot_cell_STLS = [library_function,Xi_sparse_STLS]

%% Ridge
Xi_ridge = minimize_cost(Theta,0.98,0.0001,dx(1:record_amount(1),:),'Ridge')
Xi_sparse_ridge=[{'dtheta','ddtheta'};num2cell(Xi_ridge)];
library_function=[{'coef'};yout];
x_dot_cell_rige = [library_function,Xi_sparse_ridge]


%% BPDN
Xi_BPDN = minimize_cost(Theta,0.96,0.00001,dx(1:record_amount(1),:),'BPDN')
Xi_sparse_BPDN=[{'dtheta','ddtheta'};num2cell(Xi_BPDN)];
x_dot_cell_BPDN = [library_function,Xi_sparse_BPDN]

%% WBPDN
Xi_WBPDN = minimize_cost(Theta,0.9,0.0001,dx(1:record_amount(1),:),'WBPDN')
Xi_sparse_WBPDN=[{'dtheta','ddtheta'};num2cell(Xi_WBPDN)];
x_dot_cell_WBPDN = [library_function,Xi_sparse_WBPDN]


%% adding noise
xResult_noise= awgn(xResult,40)
xResult_noise_filt_movmean = movmean(xResult_noise,30)
theta_double_dot_noisy = movmean(gradient(xResult_noise_filt_movmean(:,2),0.01),100);
dx_noisy = [xResult_noise_filt_movmean(:,2) theta_double_dot_noisy]

%% visualizing derivatives
plot(dx(:,1))
hold on
plot(dx_noisy(:,1))
legend('$\dot \theta$','$\dot \theta_{noisy}$','Fontsize',14,'Interpreter','Latex')

figure()
plot(dx(:,2))
hold on
plot(dx_noisy(:,2))
legend('$\ddot \theta$','$\ddot \theta_{noisy}$','Fontsize',14,'Interpreter','Latex')

%% generating Theta 
[yout,Xi,Theta] = myTrainSINDYc(xResult_noise_filt_movmean,dx_noisy,u,polyorder,n,usesine,lambda);

%% STLS
record_amount = size(Theta)
Xi_STLS = minimize_cost(Theta,0.9,0.001,dx_noisy(1:record_amount(1),:),'STLS')
Xi_sparse_STLS=[{'dtheta','ddtheta'};num2cell(Xi_STLS)];
library_function=[{'coef'};yout];
x_dot_cell_STLS = [library_function,Xi_sparse_STLS]

%% Ridge
Xi_ridge = minimize_cost(Theta,0.99,0.002,dx_noisy(1:record_amount(1),:),'Ridge');
Xi_sparse_ridge=[{'dtheta','ddtheta'};num2cell(Xi_ridge)];
library_function=[{'coef'};yout];
x_dot_cell_rige = [library_function,Xi_sparse_ridge]
mse_list(cont) = mse(b,Xi_ridge);

%% BPDN
Xi_BPDN = minimize_cost(Theta,1,0.001,dx_noisy(1:record_amount(1),:),'BPDN')
Xi_sparse_BPDN=[{'dtheta','ddtheta'};num2cell(Xi_BPDN)];
x_dot_cell_BPDN = [library_function,Xi_sparse_BPDN]
