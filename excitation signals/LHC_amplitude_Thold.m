a=linspace(1.2,2.85,10);
t=linspace(5,12,10);
design_points = zeros(max(size(a))^2,2);
row=0;

for i=1:max(size(a))
   for j=1:max(size(a))
       row=row+1;
       design_points(row,:)=[a(i) t(j)];
   end
end
random_design_points = design_points(randperm(size(design_points, 1)), :);


scatter(design_points(:,1),design_points(:,2),'+')
xlim([1 3])
ylim([4 13])
xlabel('$u(V)$','Interpreter','Latex')
ylabel('$T_h(s)$','Interpreter','Latex')
u_aprbs = [];

for i=1:max(size(random_design_points))
    u_aprbs =  [u_aprbs random_design_points(i,1)*ones(1,round(random_design_points(i,2)/0.01))];
end

% [tResult, xResult, u,dx] = getTrainingData(linspace(0,sum(random_design_points(:,2)), sum(random_design_points(:,2))/0.01),x0,u_aprbs);