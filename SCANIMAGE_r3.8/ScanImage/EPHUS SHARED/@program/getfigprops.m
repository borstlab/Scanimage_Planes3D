function out = getfigprops(obj,alias_name)
%GETFIGPROPS   - @program method for outputting GUI figure properties.
%   GETFIGPROPS(obj,alias_name) will a cell array of param value pairs that
%   can be used to set the figure property of a GUI.
%
%   See also 

out=[];
if nargin < 2
   error('@program/getfigprops: too few inputs.');
elseif nargin == 2
    propnames=fieldnames(obj.aliases.(alias_name).fig_props);
    out=cell(1,2*length(propnames));
    out(1:2:end)=propnames;
    out(2:2:end)=struct2cell(obj.aliases.(alias_name).fig_props);
end
