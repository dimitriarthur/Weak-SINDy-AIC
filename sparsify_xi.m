function xi_sparse = sparsify_xi_stls(xi,theta,sparsif_coeff,lambda,x_dot,n,regression_type)
    for k=1:10
        smallinds = (abs(xi)<sparsif_coeff);   % find small coefficients
        xi(smallinds)=0;                % and threshold
        for ind = 1:n                   % n is state dimension
            biginds = ~smallinds(:,ind);
            % Regress dynamics onto remaining terms to find sparse Xi
            if strcmp(regression_type,'STLS')
                xi(biginds,ind) = theta(:,biginds)\x_dot(:,ind); 
            elseif strcmp(regression_type,'Ridge') 
                xi(biginds,ind) = pinv(theta(:,biginds)'*theta(:,biginds) - lambda*eye(min(size(theta(:,biginds)))))*theta(:,biginds)'*x_dot(:,ind);
            elseif strcmp(regression_type,'BPDN') 
                xi(biginds,ind) = pinv(theta(:,biginds)'*theta(:,biginds))*(theta(:,biginds)'*x_dot(:,ind)-lambda*ones(min(size(theta(:,biginds))),1)/2)
            end
        end
    end
    xi_sparse=xi;
end