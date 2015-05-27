function result = frapcurve(beta, x)
% FRAPCURVE   - Exponential curve for fitting FRAP recoveries (use with NLINFIT).
%   FRAPCURVE is a function for use with nlinfit.
%   Generic function for creating a rising exponential of the form:
%
%   y = a*(1-exp(-b*x)) + c
%
%   See also NLINFIT, RISEEXP

b1 = beta(1);
b2 = beta(2);
b3 = beta(3);
result = b1*(1- exp(-b2 * x))+ b3;
