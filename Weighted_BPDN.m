function [x_sharp, Error_RNIE, Error_RTIE] = Weighted_BPDN(A, b, K, omega, lambda, Options)

% Weighted Basis Pursuit DeNoise (BPDN)  Solve weighted BPDN via ADMM
% This function is written by Wendong Wang (d.w.sylan@gmail.com) according to the function at following web:
%           http://web.stanford.edu/~boyd/papers/admm/basis_pursuit/basis_pursuit.html 
%
% x_sharp = Weighted_BPDN(A, b, rho, alpha)
%
% Solves the following problem via ADMM:
%
%   minimize     ||x||_{w£¬1}+(1/(2*lambda))\|b-Ax\|_{2}^2 
%
% The solution is returned in the vector optimal_solution.


[m, n] = size(A);
Kc = setdiff(1:n, K);

nu = Options.nu;
MAX_ITER = Options.MAX_ITER;
ERROR_BOUND = Options.ERROR_BOUND;
x_Original = Options.x_Original;

x_Old = zeros(n,1);
y_Old = zeros(n,1);
z_Old = zeros(n,1);

 Error_RNIE = zeros(MAX_ITER, 1);    % Relative Neighboring Iteration Error (RNIE)
 Error_RTIE = zeros(MAX_ITER, 1);    % Relative True Iteration Error (RTIE)
 
for k = 1:MAX_ITER
    
  % x-Update
    lambda_mu = lambda*nu;
    p = y_Old - z_Old;
    p0 = A'*b + lambda_mu*p;
    p1 = A*p0;
    A0 = eye(m) + A*A'/lambda_mu;
    x_New = p0 - A'*(A0\p1)/lambda_mu;
    x_New = x_New/lambda_mu;
    
  % y-Update
    y_New =zeros(n, 1);
    g = x_New + z_Old;
    g_K = g(K);
    g_Kc = g(Kc);
    y_New(K) = shrinkage(g_K, omega/nu);
    y_New(Kc) = shrinkage(g_Kc, 1/nu);
    
  % z-Update
    z_New = z_Old + x_New - y_New;
  
  % termination checks
    Relative_Error = norm(x_New - x_Old, 2)/max(norm(x_New, 2), 1);
    Error_RNIE(k) = Relative_Error;
    Error_RTIE(k) = norm(x_New - x_Original, 2)/norm(x_Original, 2);

    
    if Relative_Error < ERROR_BOUND
        Error_RNIE = Error_RNIE(1:k);
        Error_RTIE = Error_RTIE(1:k);
         break
    else
        x_Old = x_New;
        y_Old = y_New;
        z_Old = z_New;
    end

end
x_sharp = x_New;

end


