function xh=CS_RL1(y,A,iter)
%if there are more columns than lines -> N = max(size(A)), M = min(size(A))
%else, N=min(size(A))
N=max(size(A));
M=min(size(A));
Li=(M/(4*log(N/M)));
y=y(:);
W=ones(N,1);   %intialize weight vector
QW=diag(W);    %initialize weight matrix
% delta=0.01;
for i=1:iter
    QWt=inv(QW);
    %A=A'
    At=A*QWt;
    %pinv_at=At;
    x0=A'*y;  %solução estimada vi LS
    xh=l1eq_pd(x0,At,[],y,1e-3);
    delta=max(norm(xh,Li),1e-3) ;%automatic dynamic update delta
%     delta should be set slightly smaller than the expected nonzero magnitudes of x0. 
%     In general, the recovery process tends to be reasonably robust to the choice of delta.--原文中的话
    xh=QWt*xh;
    %xh=Qwt(1:M,1:M)
    QW=diag(1./(abs(xh)+delta));
    
end

end
