% PROGMANAGER/getDefaultCache - Retrieve a default value from the @progmanager's cache.
%
% SYNTAX
%  value = getDefaultCache(progmanager, name)
%    progmanager - The program manager object.
%    name - The name of the default value.
%    value - The value of the default, if it exists, empty otherwise.
%
% NOTES
%  See TO120705D.
%
% CHANGES
%  TO120905N - Watch out for non-existent values. -- Tim O'Connor 12/9/05
%
% Created 12/7/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function value = getDefaultCache(this, name)

value = [];%TO120905N

progmanagerPath = fileparts(which('progmanager/progmanager'));
if exist(fullfile(progmanagerPath, 'progmanagerDefaultCache.mat')) == 2
    loadedCache = load(fullfile(progmanagerPath, 'progmanagerDefaultCache.mat'), '-mat');
    if isfield(loadedCache.defaults, name) %TO120905N
        value = loadedCache.defaults.(name);
    end
end

return;