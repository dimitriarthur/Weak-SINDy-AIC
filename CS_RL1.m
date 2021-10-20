function xh=CS_RL1(y,A,iter)
N=max(size(A));
M=min(size(A));
Li=(M/(4*log(N/M)));
y=y(:);
W=ones(N,1);   %intialize weight vector
QW=diag(W);    %initialize weight matrix
% delta=0.01;
for i=1:iter
    QWt=inv(QW);
    A=A'
    At=A*QWt;
    pinv_at=At;
    x0=pinv(At)'*y;  %最小二乘解估计一个初始值
    xh=l1eq_pd(x0,At',[],y,1e-3);
    delta=max(norm(xh,Li),1e-3) ;%动态更新 自适应更新delta
%     delta should be set slightly smaller than the expected nonzero magnitudes of x0. 
%     In general, the recovery process tends to be reasonably robust to the choice of delta.--原文中的话
    xh=QWt*xh;
    %xh=Qwt(1:M,1:M)
    QW=diag(1./(abs(xh)+delta));
    
end

end
