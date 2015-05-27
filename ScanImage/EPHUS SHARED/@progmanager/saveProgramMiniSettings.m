% PROGMANAVER/saveProgramMiniSettings
%
% SYNTAX
%  saveProgramMiniSettings(progmanager, hObject)
%  saveProgramMiniSettings(progmanager, hObject, filename)
%   progmanager - Program Manager.
%   hObject - Program handle.
%   filename - The file to save into.
%
% USAGE
%
% NOTES
%  Copy & paste from saveProgramSettings.m.
%
% CHANGES
%
% Created 7/15/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function saveProgramMiniSettings(this, hObject, varargin)
global progmanagerglobal;

% filename = 'C:\Matlab704\work\test.settings';
if isempty(varargin)
    [pathname filename] = uiputfile('*.settings', 'Save configuration as...');
    if ~endsWithIgnoreCase(filename, '.settings')
        filename = [filename '.settings'];
    end
    filename = fullfile(pathname, filename);
else
    filename = varargin{1};
end

settings = getProgramMiniSettings(this, hObject);

saveCompatible(filename, 'settings');%TO071906D

%Clean up.
guinames = fieldnames(settings);
for i = 1 : length(guinames)
    delete(settings.(guinames{i}));
end

return;