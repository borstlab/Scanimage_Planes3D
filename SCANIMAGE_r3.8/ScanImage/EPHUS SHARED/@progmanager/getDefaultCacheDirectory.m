% PROGMANAGER/getDefaultCacheDirectory - Retrieve a default value from the @progmanager's cache.
%
% SYNTAX
%  value = getDefaultCacheDirectory(progmanager, name)
%    progmanager - The program manager object.
%    name - The name of the default cached directory.
%    value - The path of the default directory, if it exists, `pwd` otherwise.
%
% NOTES
%  Calls through to getDefaultCacheValue, then checks it if is a valid directory.
%  The returned result will always be a valid directory.
%
% CHANGES
%
% Created 3/9/06 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function value = getDefaultCacheDirectory(this, name)

value = getDefaultCacheValue(this, name);
if isempty(value) | exist(value) ~= 7
    value = pwd;
end

return;