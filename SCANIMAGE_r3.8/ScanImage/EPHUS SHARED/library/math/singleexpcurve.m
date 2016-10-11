function result = singleexpcurve(beta, x)
% SINGLEEXPCURVE   - Rising Exponential with (use with NLINFIT).
%   SINGLEEXPCURVE is a function for use with nlinfit.
%   Generic function for creating a rising exponential with offset of the form:
%
%   y = a + b*exp(c*x)
%
%   See also NLINFIT

b1 = beta(1);
b2 = beta(2);
b3 = beta(3);
result = b1 + b2*exp(b3 * x);
