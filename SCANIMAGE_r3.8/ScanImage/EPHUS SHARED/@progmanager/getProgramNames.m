% @progmanager/getProgramNames - Get a list of all running programs.
%
% SYNTAX
%  programNames = getProgramNames(progmanager)
%    progmanager - The programmanager object instance.
%    programNames - A cell array of strings, listing all running programs.
%
% USAGE
%
% NOTES
%
% CHANGES
%
% Created 9/7/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function programNames = getProgramNames(this)
global progmanagerglobal;

programNames = {};
if ~isempty(progmanagerglobal.programs)
    programNames = fieldnames(progmanagerglobal.programs);
end

return;