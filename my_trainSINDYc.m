% Sparse regression
clear Theta Xi
Theta = poolData(xaug,n,polyorder,usesine);
Theta_norm = zeros(size(Theta,2),1); %zeros(size(Theta,2),1);ones
for i = 1:size(Theta,2)
   Theta_norm(i) = norm(Theta(:,i));
   Theta(:,i) = Theta(:,i)./Theta_norm(i);
end
m = size(Theta,2);

if exist('lambda_vec') == 1
    Xi = sparsifyDynamicsIndependent(Theta,dx,lambda_vec,n-1);
else
    Xi = sparsifyDynamics(Theta,dx,lambda,n-1);
end


if n == 3
    str_vars = {'x','y','u'};
elseif n == 4
    str_vars = {'x','y','z','u'};
elseif n == 6
    str_vars = {'x1','x2','x3','x4','x5','u'};    
end

for i = 1:size(Theta,2)
   Xi(i,:) = Xi(i,:)./Theta_norm(i);
end

yout = poolDataLIST(str_vars,Xi,n,polyorder,usesine);