function [IC] = ICcalculations(abserror, p, N)
% this function takes in time series from model, xm
% time series from "real" data, xr
% number of parameters (or library functions), p
% N number of data points (# time series)

% loglikelyhood assuming normally destributed error 
logL = -N*log(abserror'*abserror/N)/2;
errorout =abserror'*abserror/N;
if p>0
aic = -2*logL + 2*p;
aic_c = aic + 2*p*(p+1)/(N-p-1);
% if N/p < 40
%     disp('N/p <40 aic_c is correct limit')
%     fprintf('need %d measurements for aic to be correct limit \n', p*40)
% end
bic = -2*logL + 2*p*log(N);

IC.aic = aic;
IC.bic = bic;
IC.aic_c= aic_c;
%stop


elseif p==0 % calculate with hard p=0 to avoid divergence.
IC.aic =-2*logL;
IC.bic = -2*logL;
IC.aic_c = -2*logL;

else
    error('negative number of parameters?')
end