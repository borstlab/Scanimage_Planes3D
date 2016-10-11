% PROGMANAGER/displayVersionDialog
%
% SYNTAX
%  displayVersionDialog(progmanager)
%  displayVersionDialog(progmanager, hObject)
%   progmanager - Program Manager.
%   hObject - The handle of the current program.
%
% USAGE
%  Opens a dialog box with version information for the entire distribution, and potentially for the currently active GUI/program.
%
% NOTES
%  See TO111908G.
%
% CHANGES
%
% Created 11/19/08 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2008
function displayVersionDialog(this, varargin)
global progmanagerglobal;

versionFile = fopen('ephus_version.tag', 'r');
ephusVersion = fgetl(versionFile);
fclose(versionFile);
versionString = sprintf('Ephus Version: %s', ephusVersion);
if ~isempty(varargin)
    hObject = getParent(varargin{1}, 'figure');
    programName = getProgramName(this, hObject);
    guiName = getGUIName(this, hObject);
    programVersion = feval(progmanagerglobal.programs.(programName).guinames.(guiName).funchandle, 'getVersion', ...
                progmanagerglobal.programs.(programName).guinames.(guiName).fighandle, [], ...
                progmanagerglobal.programs.(programName).guinames.(guiName).fighandle);
    versionString = sprintf('%s\n%s Version: %s', versionString, programName, num2str(programVersion));
end

msgbox(versionString, 'Version Information', 'Custom', imread('ephus_icon.bmp'));

return;