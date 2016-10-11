function [out] = get(obj,param)
%GET   - Overloaed method for @progmanager class.
%   GET will get properties from an object.  
%   GET(obj) outputs the entire object as a STRUCT.
%   GET(obj,param) outputs the field specified by param.
%
%   See also PROGMANAGER, STRUCT

out=[];
if nargin == 1
    out=struct(obj);
elseif nargin == 2
    out=obj.(param);
else
    error('@program/get: too many inputs.');
end
