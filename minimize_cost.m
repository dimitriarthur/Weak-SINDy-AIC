function xi_sparse = minimize_cost(theta,sparsif_coeff,lambda,x_dot,min_type)
    n=2;
    if strcmp(min_type,'Ridge')
        % matrix formulation of the minimum of the cost function
        xi_sparse = pinv(theta'*theta - lambda*eye(min(size(theta))))*theta'*x_dot;
        xi_sparse = sparsify_xi_ridge(xi_sparse,theta,sparsif_coeff,lambda,x_dot,2);
    elseif strcmp(min_type,'STLS')
        % least squares basic formulation
        xi_sparse = theta\x_dot;  % initial guess: Least-squares
        % lambda is our sparsification knob.
        xi_sparse = sparsify_xi_stls(xi_sparse,theta,sparsif_coeff,x_dot,2);
    elseif strcmp(min_type,'BPDN')
        xi_sparse=pinv(theta'*theta)*(theta'*x_dot-lambda*ones(min(size(theta)),1)/2);
        xi_sparse = sparsify_xi_bpdn(xi_sparse,theta,sparsif_coeff,lambda,x_dot,2);
    end
end