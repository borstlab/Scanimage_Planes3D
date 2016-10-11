function out = getvariables(obj,alias_name,var_name)
%GETVARIABLES   - @program method for outputting GUI variables.
%   GETVARIABLES(obj,alias_name) will return the structure of all variables
%   for the alias_name specified.
%
%   GETVARIABLES(obj,alias_name,var_name) will return the value of the
%   variable var_name for the alias_name specified.
%
%   See also 

out=[];
if nargin < 2
   error('@program/getvariables: too few inputs.');
elseif nargin == 2
    out=obj.aliases.(alias_name).variables;
else
    out=obj.aliases.(alias_name).variables.(var_name);
end
