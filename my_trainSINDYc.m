% Sparse regression
clear Theta Xi
Theta = myPoolData(xaug,n,polyorder,usesine); %ok
Theta_norm = zeros(size(Theta,2),1); %ok

%determining how many functions there is
m = size(Theta,2);
lambda=0.99;
Xi = sparsifyDynamics(Theta,dx,lambda,n-1);

%if n == 3
%    str_vars = {'theta','theta_dot','u'};
%elseif n == 4
%    str_vars = {'x','y','z','u'};
%elseif n == 6
%    str_vars = {'x1','x2','x3','x4','x5','u'};    
%end

%for i = 1:size(Theta,2)
%   Xi(i,:) = Xi(i,:)./Theta_norm(i);
%end
str_vars={'theta','dtheta','u'};
yout = myPoolDataVariableNames(str_vars,Xi,3,2,1);