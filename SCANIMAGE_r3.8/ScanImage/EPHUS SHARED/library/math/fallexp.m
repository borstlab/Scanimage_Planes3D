function result = fallexp(beta, x)
% FALLEXP   - Decaying Exponential without offset (use with NLINFIT).
%   FALLEXP is a function for use with nlinfit.
%   Generic function for creating a decaying exponential of the form:
%
%    y = amp*exp(-x/tau)
%
%   One input parameter beta(2) is a time constant, as opposed to its inverse
%   as in SINGLEEXP, which may be appealing in certain applications.
%
%   See also NLINFIT, SINGLEEXP

amp = beta(1);
tau = beta(2);
result = amp*exp(-x./tau);
