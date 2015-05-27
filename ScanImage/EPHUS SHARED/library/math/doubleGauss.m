function y=doubleGauss(beta, x)
% DOUBLEGAUSS   - Double Gaussian Generator (use with NLINFIT).
%   DOUBLEGAUSS is a function for use with nlinfit.
%   Generic function for creating a double Gaussian of the form:
%
%   y = amp1*exp(-(x-xshift1)^2/(2*stdev1^2))+amp2*exp(-(x-xshift2)^2/(2*stdev2^2))
%
%   See also NLINFIT

amp1=beta(1);
xshift1=beta(2);
stdev1=beta(3);
amp2=beta(4);
xshift2=beta(5);
stdev2=beta(6);

y= amp1.*exp(-(x-xshift1).^2/(2*stdev1.^2))+amp2.*exp(-(x-xshift2).^2/(2*stdev2.^2));

