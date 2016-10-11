function yhat = hyperbola(beta, x)
% HYPERBOLA - Generates Hyperbola for fitting (use with NLINFIT).
%   HYPERBOLA is a function for use with nlinfit.
%   Generic function for creating a hyperbola of the form:
%
%   y = a*x/(b + x)
%
%   Useful for modeling linear binding relationships such as Michaelis-Mentin
%   Binding.
%
%   See also NLINFIT, HILLPLOT

b1 = beta(1);
b2 = beta(2);

yhat = (b1*x)./(b2+x);