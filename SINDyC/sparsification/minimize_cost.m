function xi_sparse = minimize_cost(theta,sparsif_coeff,lambda,x_dot,min_type)
    n=2;
    if strcmp(min_type,'Ridge')
        % matrix formulation of the minimum of the cost function
        xi_sparse = pinv(theta'*theta - lambda*eye(min(size(theta))))*theta'*x_dot;
        xi_sparse = sparsify_xi(xi_sparse,theta,sparsif_coeff,lambda,x_dot,2,'Ridge');
    elseif strcmp(min_type,'STLS')
        % least squares basic formulation
        xi_sparse = theta\x_dot;  % initial guess: Least-squares
        % lambda is our sparsification knob.
        xi_sparse = sparsify_xi(xi_sparse,theta,sparsif_coeff,lambda,x_dot,2,'STLS');
    elseif strcmp(min_type,'BPDN')
        xi_sparse=pinv(theta'*theta)*(theta'*x_dot-lambda*ones(min(size(theta)),1)/2);
        xi_sparse = sparsify_xi(xi_sparse,theta,sparsif_coeff,lambda,x_dot,2,'BPDN');
    elseif strcmp(min_type,'WBPDN')
        xi_sparse=pinv(theta'*theta)*(theta'*x_dot-lambda*ones(min(size(theta)),1)/2);
        xi_sparse = sparsify_xi(xi_sparse,theta,sparsif_coeff,lambda,x_dot,2,'WBPDN');
    end
end