function result = singleexp(beta, x)
% SINGLEEXP   - Decaying Exponential without offset (use with NLINFIT).
%   SINGLEEXP is a function for use with nlinfit.
%   Generic function for creating a decaying exponential of the form:
%
%   y = amp*exp(-a*x)
%
%   See also NLINFIT, FALLEXP

b1 = beta(1);
b2 = beta(2);
result = b1*(exp(-b2 * x));
