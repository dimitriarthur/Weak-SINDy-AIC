reg=0.00000001;
signal = theta_noise;
mse_list = zeros(1,250);
reg_list = zeros(1,250);
var_list = zeros(1,250);

for i=1:250
    sampling_time = 0.01;
    derivative = TVRegDiff(signal(1:9998,:), 10, reg, [], 'small', 1e-12, 0.01,0,0);
    integrated_signal = cumtrapz(sampling_time,derivative);
    mse_integration = immse(integrated_signal,signal);
    mse_list(1,i) = mse_integration;
    reg_list(1,i) = reg;
    var_list(1,i) = var(derivative);
    reg = reg + 0.1*mse_integration;
end
%%
minimum_index = find(mse_list==min(min(mse_list)))
minimum_index_derivative = TVRegDiff(signal, 10, reg_list(1,minimum_index), [], 'small', 1e-12, 0.01,0,0);

%%

subplot(2,1,1)
plot(var_list)
ylabel('var')
subplot(2,1,2)
plot(mse_list)
ylabel('mse')

%% plotting mse and reg params
subplot(3,1,1)
plot(reg_list,mse_list)
xlim([reg_list(1,1) reg_list(1,max(size(reg_list)))])
grid

title('MSE Through Iterations')
ylabel('MSE')
xlabel('Regularization')

subplot(3,1,2)
plot(cumtrapz(sampling_time,minimum_index_derivative))
hold on
plot(signal)
xlim([0 1000])
ylabel('$\theta$','Interpreter','latex')
xlabel('Sampling Instant')
legend('Integrated $\theta$ from $\dot \theta$','Original $\theta$','Interpreter','Latex')
title('Opmtimal Regularization Parameter in Terms of MSE = 8.0054e-04')
grid

subplot(3,1,3)
plot(dx(1:1000,1))
xlim([0 1000])
hold on
plot(minimum_index_derivative)
grid
legend('Original $\dot \theta$','Computed $\dot \theta$','Interpreter','Latex')


%% comparing to a naive approach towards the reg param
naive_reg_derivative = TVRegDiff(signal, 10, 0.0009, [], 'small', 1e-12, 0.01,0,0);
naive_integral = cumtrapz(sampling_time,naive_reg_derivative);

subplot(2,1,1)
plot(naive_integral)
title('Comparison between integrals - MSE = 0.4039')
hold on
plot(signal)
grid
legend('Integrated $\theta$ from $\dot \theta$','Original $\theta$','Interpreter','Latex')
ylabel('$\theta$','Interpreter','Latex')
xlim([0 1000])

subplot(2,1,2)
plot(dx(1:1000,1))
hold on
plot(naive_reg_derivative)
xlim([0 1000])
grid
legend('Original $\dot \theta$','Computed $\dot \theta$','Interpreter','Latex')
ylabel('$ \dot\theta$','Interpreter','Latex')
