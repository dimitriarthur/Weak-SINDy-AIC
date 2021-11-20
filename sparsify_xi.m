function xi_sparse = sparsify_xi(xi,theta,sparsif_coeff,lambda,x_dot,n,regression_type)
    W = ones(min(size(theta)),2);    
    for k=1:10
        smallinds = (abs(xi)<sparsif_coeff);   % find small coefficients
        xi(smallinds)=0;                % and threshold
        for ind = 1:n                   % n is state dimension
            biginds = ~smallinds(:,ind);
            % Regress dynamics onto remaining terms to find sparse Xi. It
            % varies with the type of minimization
            if strcmp(regression_type,'Ridge') 
                xi(biginds,ind) = pinv(theta(:,biginds)'*theta(:,biginds) - lambda*eye(min(size(theta(:,biginds)))))*theta(:,biginds)'*x_dot(:,ind);
            elseif strcmp(regression_type,'STLS')
                xi(biginds,ind) = theta(:,biginds)\x_dot(:,ind); 
            elseif strcmp(regression_type,'BPDN') 
                xi(biginds,ind) = pinv(theta(:,biginds)'*theta(:,biginds))*(theta(:,biginds)'*x_dot(:,ind)-lambda*ones(min(size(theta(:,biginds))),1)/2);
            elseif strcmp(regression_type,'WBPDN')
                xi(biginds,ind) = pinv(theta(:,biginds)'*theta(:,biginds))*(theta(:,biginds)'*x_dot(:,ind)-lambda/2*W(biginds,ind));
                W=1./(abs(xi).^4 + 10e-2);
            end
        end
    end
    xi_sparse=xi;
end