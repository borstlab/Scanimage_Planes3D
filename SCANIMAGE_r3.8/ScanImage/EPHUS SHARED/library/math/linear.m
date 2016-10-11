function result = linear(beta, x)
% LINEAR   - General linear function (use with NLINFIT). 
%   LINEAR is a function for use with nlinfit.
%   Generic function for creating a linear function of the form:
%
%   y = m*x + b
%
%   See also NLINFIT

b1 = beta(1);
b2 = beta(2);
result = b1*x+b2;
