function [out] = getstructure(obj,varargin)
% GETSTRUCTURE   - Accesses structure or variable from global through object.
%   GETSTRUCTURE will look at the current progmanagerglobal and get the
%   variable specified in varargin.
%
%   Example:   
%         >> copy=getstructure(object,'internal');
%         
%         copy = 
%         version: 'daq'
%         
%         returns the same thing as:
% 
%         >> programmanager.internal 
%           
%         copy = 
%         version: 'daq'
% 
%   See also

out=[];
if nargin < 2
    out=evalin('base',obj.name);
else
    newvarargin=cell(1,2*length(varargin));
    newvarargin(1:2:end)={'.'};
    newvarargin(2:2:end)=varargin;
    out=evalin('base',[obj.name newvarargin{:}]);
end
