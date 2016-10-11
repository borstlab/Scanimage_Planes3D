% progmanager/getMainGh - Allows subprograms to conveniently access graphics objects within their parent.
%
% SYNTAX
%  value = getMainGh(progmanager, hObject, tag)
%  value = getMainGh(progmanager, hObject, tag, propertyName)
%   progmanager - A program manager instance.
%   hObject - The handle to the subprogram.
%   tag - The tag of the graphics object to access.
%   propertyName - An optional argument, which will retrieve a specific property of the tagged object.
%   value - The graphics object handle or requested property value.
%
% USAGE
%
% NOTES
%
% CHANGES
%
% Created 2/16/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function [value] = getMainGh(this, hObject, tag, varargin)
global progmanagerglobal;

value = [];

udata = get(getParent(hObject, 'figure'), 'UserData');

value = getGlobalGh(this, tag, progmanagerglobal.programs.(udata.progname).mainGUIname, udata.progname);
if length(varargin) > 1
    value = {get(value, varargin{:})};
elseif length(varargin) == 1
    value = get(value, varargin{1});
end

return;