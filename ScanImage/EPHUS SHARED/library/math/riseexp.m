function result = riseexp(beta, x)
% RISEEXP   - Exponential curve for fitting FRAP recoveries without offset (use with NLINFIT).
%   RISEEXP is a function for use with nlinfit.
%   Generic function for creating a 1 - exponential of the form:
%
%   y = amp - amp*exp(-x/tau)
%
%   See also NLINFIT, FRAPCURVE

amp = beta(1);
tau = beta(2);
result = amp*(1- exp(-x./tau));
