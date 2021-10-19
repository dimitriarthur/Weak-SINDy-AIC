% Sparse regression
clear Theta Xi
Theta = myPoolData(xaug,n,polyorder,usesine); %ok
Theta_norm = zeros(size(Theta,2),1); %ok

%this corresponds to a normalization on each column of the function library
%for i = 1:size(Theta,2)
%   Theta_norm(i) = norm(Theta(:,i));
%   Theta(:,i) = Theta(:,i)./Theta_norm(i);
%end

%determining how many functions there is
m = size(Theta,2);

%if exist('lambda_vec') == 1
%    Xi = sparsifyDynamicsIndependent(Theta,dx,lambda_vec,n-1);
%else
%end
Xi = sparsifyDynamics(Theta,dx,lambda,n-1);

if n == 3
    str_vars = {'theta','theta_dot','u'};
elseif n == 4
    str_vars = {'x','y','z','u'};
elseif n == 6
    str_vars = {'x1','x2','x3','x4','x5','u'};    
end

for i = 1:size(Theta,2)
   Xi(i,:) = Xi(i,:)./Theta_norm(i);
end

yout = poolDataLIST(str_vars,Xi,n,polyorder,usesine);