function yhat=hillplot(beta,x)
% HILLPLOT   - Sigmoidal curve for fitting cooperative binding (use with NLINFIT).
%   HILLPLOT is a function for use with nlinfit.
%   Generic function for creating a sigmoidal curve of the form:
%
%   y = (x^n)/(K + x^n);
%
%   Used to model cooperative processes like in the generic Hill equation
%   where:
% 	    n = Hill coefficent
% 	    K = Kd (Dissociation Constant)
%   When n = 1, it is a HYPERBOLA.
%
%   See also NLINFIT, HYPERBOLA, ANALOGINPUT, progmanager

n=beta(1);
K=beta(2);
scaling=beta(3);
offset=beta(4);

yhat=(scaling*(x.^n))./(K+x.^n)+offset;
