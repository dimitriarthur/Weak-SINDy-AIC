function [result, err, flop] = l1norm_rw(A, b, lambda, step)
p = 0.9;
m = size(A,1);
n = size(A,2);
x = (A'*A+0.0000001*eye(n))^-1*A'*b;
res = norm(b - A*x);
res_rec = res;
res_rec_a = res;
itr = 0;
itr_limit = 1000;
flop = 0;
x_rec = x;
while itr < itr_limit
    w = p.*(abs(x).^(p-1));
    flop = flop + n;
    djdx = 2*A'*A*x - 2*A'*b + lambda*w.*sign(x);
    flop = flop + 2*n*m+2*m*n*m + 2*m*n + 4*n;
    x = x - step*djdx;
    res = norm(b - A*x);
    res_rec_a = [res_rec_a, res];
    if res < res_rec
       res_rec = res;
       x_rec = x;
    end
%     if res < error
%        res_rec = res;
%        x_rec = x;
%        break;
%     end
%     if mod(itr,100)==0
%         fprintf('This is iteration %f, error %f\n', itr,res);
%     end
    itr = itr + 1;
end
result = x;
err = res_rec;