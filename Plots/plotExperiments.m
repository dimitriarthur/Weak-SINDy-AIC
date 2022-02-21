subplot(2,1,1)
plot(simout_open_loop.signals.values(:,1))
grid
ylim([1.3 2.7])
xlim([0 3400])
ylabel('$u(V)$','Interpreter','latex')
subplot(2,1,2)
plot(simout_open_loop.signals.values(:,2),'color','#D95319')
grid
xlim([0 3400])
ylabel('$\theta(rad)$','Interpreter','latex')