% PROGMANAGER/saveConfigurations - Save all the configurations of all open programs, including GUI positions.
%
% SYNTAX
%  saveConfigurations(progmanager)
%  saveConfigurations(progmanager, location)
%    progmanager - The program manager object.
%    locations - The disk location in which to save the configuration, if not supplied a prompt will be issued.
%
% NOTES
%
% CHANGES
%  TO120705D: Created a general function for handling default caching, `getDefaultCacheValue`. -- Tim O'Cononr 12/7/05
%  TO120905E: Changed 'loadPath' to 'savePath. -- Tim O'Connor 12/9/05
%  TO071906D: Make sure `save` makes Matlab v6 compatible files. -- Tim O'Connor 7/19/06
%  TO043008A: Handle cancellation of the save operation and/or invalid save directories. -- Tim O'Connor 4/30/08
%
% Created 11/18/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function saveConfigurations(this, varargin)
global progmanagerglobal;

%TO120705D: Use `getDefaultCacheValue` now. -- Tim O'Connor 12/7/05
lastConfigDir = getDefaultCacheValue(this, 'lastConfigDir');
% defaults = [];
% progmanagerPath = fileparts(which('progmanager/progmanager'));
% if exist(fullfile(progmanagerPath, 'progmanagerDefaultCache.mat')) == 2
%     loadedCache = load(fullfile(progmanagerPath, 'progmanagerDefaultCache.mat'), '-mat');
%     lastConfigDir = loadedCache.defaults.lastConfigDir;
% else
%     lastConfigDir = pwd;
% end
%TO043008A
if isempty(lastConfigDir)
    lastConfigDir = pwd;
end

if exist(lastConfigDir, 'dir') ~= 7
    lastConfigDir = pwd;
%     warning('Cached configuration directory default is corrupted or missing, using `pwd` instead: %s', lastConfigDir);
end

if ~isempty(varargin)
    savePath = varargin{1};
else
    savePath = uigetdir(lastConfigDir, 'Select (or create) a directory in which to save this configuration.');
    %TO043008A
    if length(savePath) == 1
        if savePath == 0
            return;
        end
    end
end
if exist(savePath, 'dir') ~= 7
    error('Invalid progmanager configuration save directory: %s', savePath);
end
%Prompt for overwrites.
noOverwrite = 1;
while exist(fullfile(savePath, 'guiMetaInfo.mat'), 'file') == 2 & noOverwrite
    overwrite = questdlg(sprintf('%s already exists. Overwrite?', savePath), 'Overwrite Existing Config', 'Yes', 'No', 'Cancel', 'No');
    switch lower(overwrite)
        case 'yes'
            noOverwrite = 0;
        case 'no'
            savePath = uigetdir(lastConfigDir, 'Select (or create) a directory in which to save this configuration.');
        case 'cancel'
            return;
        otherwise
            error('Unrecognized option: %s', overwrite);
    end
end

% defaults.lastConfigDir = savePath;
% save(fullfile(progmanagerPath, 'progmanagerDefaultCache.mat'), 'defaults', '-mat');
setDefaultCacheValue(this, 'lastConfigDir', savePath);%TO120705D, TO120905E
progmanagerGuisConfig = [];

programs = fieldnames(progmanagerglobal.programs);
for i = 1 : length(programs)
    fprintf(1, '%s - Saving configuration for %s.\n', datestr(now), programs{i});
    guiNames = fieldnames(progmanagerglobal.programs.(programs{i}).guinames);
    for j = 1 : length(guiNames)
        progmanagerGuisConfig.(programs{i}).guis.(guiNames{j}).position = get(progmanagerglobal.programs.(programs{i}).guinames.(guiNames{j}).fighandle, 'Position');
        progmanagerGuisConfig.(programs{i}).guis.(guiNames{j}).visible = get(progmanagerglobal.programs.(programs{i}).guinames.(guiNames{j}).fighandle, 'Visible');
    end

    try
        saveProgramSettings(this, progmanagerglobal.programs.(programs{i}).guinames.(progmanagerglobal.programs.(programs{i}).mainGUIname).fighandle, ...
            fullfile(savePath, [progmanagerglobal.programs.(programs{i}).mainGUIname '.settings']));
    catch
        warning('Failed to properly save settings for program ''s'': %s', programs{i}, lasterr);
    end
end

saveCompatible(fullfile(savePath, 'guiMetaInfo.mat'), 'progmanagerGuisConfig', '-mat');%TO071906D

fprintf(1, '%s - Configurations saved in %s.\n', datestr(now), savePath);

return;