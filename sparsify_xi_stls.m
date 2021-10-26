function xi_sparse = sparsify_xi_stls(xi,theta,sparsif_coeff,x_dot,n)
    for k=1:10
        smallinds = (abs(xi)<sparsif_coeff);   % find small coefficients
        xi(smallinds)=0;                % and threshold
        for ind = 1:n                   % n is state dimension
            biginds = ~smallinds(:,ind);
            % Regress dynamics onto remaining terms to find sparse Xi
            xi(biginds,ind) = theta(:,biginds)\x_dot(:,ind); 
        end
    end
    xi_sparse=xi;
end