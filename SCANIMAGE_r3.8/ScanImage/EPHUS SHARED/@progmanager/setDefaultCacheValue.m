% PROGMANAGER/setDefaultCache - Retrieve a default value from the @progmanager's cache.
%
% SYNTAX
%  setDefaultCache(progmanager, name, value)
%    progmanager - The program manager object.
%    name - The name of the default value.
%    value - The value of the new default.
%
% NOTES
%  See TO120705D.
%
% CHANGES
%  TO071906D: Make sure `save` makes Matlab v6 compatible files. -- Tim O'Connor 7/19/06
%
% Created 12/7/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function value = setDefaultCache(this, name, value)

progmanagerPath = fileparts(which('progmanager/progmanager'));
if exist(fullfile(progmanagerPath, 'progmanagerDefaultCache.mat')) == 2
    loadedCache = load(fullfile(progmanagerPath, 'progmanagerDefaultCache.mat'), '-mat');
    defaults = loadedCache.defaults;
    defaults.(name) = value;
else
    defaults = [];
end
saveCompatible(fullfile(progmanagerPath, 'progmanagerDefaultCache.mat'), 'defaults', '-mat');%TO071906D

return;