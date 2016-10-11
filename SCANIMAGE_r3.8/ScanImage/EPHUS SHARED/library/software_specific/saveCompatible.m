% saveCompatible - Callthrough to Matlab's `save` that sets the version compatibility flag as necessary to open files in v6 mode.
%
% SYNTAX
%  See Matlab's `save` function.
%
% USAGE
%  Use to ensure binary compatibility with Matlab v6.x.
%
% NOTES
%  See TO071906D.
%
% CHANGES
%  TO030210B - Incremented the version, to enable compression of resulting MAT files (only when using Matlab r2007+). -- Tim O'Connor 3/2/10
%  VI030810A - Use v70 instead of v73, as v73 (HDF5) does not implement compression for releases 2006b-2007b. 
%
% Created 7/19/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function saveCompatible(varargin)

saveStr = 'save(';
for i = 1 : length(varargin)
    if i < length(varargin)
        saveStr = [saveStr '''' varargin{i} ''', '];
    else
        saveStr = [saveStr '''' varargin{i} ''''];
    end
end

verstring = version;
if str2num(verstring(1:3)) >= 7.5
    saveStr = [saveStr ', ''-v7'');']; %VI030810A %TO030210B
elseif str2num(verstring(1:3)) >= 7.2
    saveStr = [saveStr ', ''-v6'');'];
else
    saveStr = [saveStr ');'];
end

% fprintf(1, 'saveCompatible: saveStr = ''%s''\n', saveStr);
evalin('caller', saveStr);

return;