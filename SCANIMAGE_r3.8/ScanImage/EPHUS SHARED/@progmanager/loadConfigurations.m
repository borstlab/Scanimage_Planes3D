% PROGMANAGER/loadConfigurations - Load all the configurations of all open programs, including GUI positions.
%
% SYNTAX
%  loadConfigurations(progmanager)
%  loadConfigurations(progmanager, location)
%    progmanager - The program manager object.
%    locations - The disk location from which to load the configuration, if not supplied a prompt will be issued.
%
% NOTES
%
% CHANGES
%  TO120705D: Created a general function for handling default caching, `getDefaultCacheValue`. -- Tim O'Cononr 12/7/05
%  TO120905O: Only change the x and y values for 'Position', not the width and height. -- Tim O'Connor 12/9/05
%  TO030906A: Added `getDefaultCacheDirectory` method, for ease of retrieving cached directories. -- Tim O'Connor 3/9/06
%  TO090706E: Modified to work with configurations that lack gui metadata. -- Tim O'Connor 9/7/06
%  TO091406A: If settings are not found, just skip them. -- Tim O'Connor 9/14/06
%  TO033008B: Only try loading settings if the file exists in the first place. -- Tim O'Connor 3/30/08
%
% Created 11/18/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function loadConfigurations(this, varargin)
global progmanagerglobal;


if ~isempty(varargin)
    loadPath = varargin{1};
else
    %TO030906A: Moved this inside the else statement, as well as added `getDefaultCacheDirectory`. -- Tim O'Connor 3/9/06
    % %TO120705D: Use `getDefaultCacheValue` now. -- Tim O'Connor 12/7/05
    lastConfigDir = getDefaultCacheDirectory(this, 'lastConfigDir');
    % % defaults = [];
    % % progmanagerPath = fileparts(which('progmanager/progmanager'));
    % % if exist(fullfile(progmanagerPath, 'progmanagerDefaultCache.mat')) == 2
    % %     loadedCache = load(fullfile(progmanagerPath, 'progmanagerDefaultCache.mat'), '-mat');
    % %     lastConfigDir = loadedCache.defaults.lastConfigDir;
    % % else
    % %     lastConfigDir = pwd;
    % % end
    % if exist(lastConfigDir) ~= 7
    %     lastConfigDir = pwd;
    % %     warning('Cached configuration directory default is corrupted or missing, using `pwd` instead: %s', lastConfigDir);
    % end
    
    loadPath = uigetdir(lastConfigDir, 'Choose a directory from which to load this configuration.');  
    if length(loadPath) == 1
        if loadPath == 0
            return;
        end
    end
end
if exist(loadPath) ~= 7
    error('Invalid progmanager configuration load directory: %s', loadPath);
end

%TO090706E
progmanagerGuisConfig = [];
if exist(fullfile(loadPath, 'guiMetaInfo.mat')) == 2
    loadedGuiMetaInfo = load(fullfile(loadPath, 'guiMetaInfo.mat'), 'progmanagerGuisConfig', '-mat');
    progmanagerGuisConfig = loadedGuiMetaInfo.progmanagerGuisConfig;
end
configFiles = dir(fullfile(loadPath, '*.settings'));
for i = 1 : length(configFiles)
    configFiles(i).name = configFiles(i).name(1:end-9);
end
if ~isempty(progmanagerGuisConfig)
    programs = union(fieldnames(progmanagerGuisConfig), {configFiles(:).name});
else
    programs = {configFiles(:).name};
end
% programs = fieldnames(progmanagerGuisConfig);

runningPrograms = fieldnames(progmanagerglobal.programs);
for i = 1 : length(programs)
    fprintf(1, '%s - Mounting configuration for %s.\n', datestr(now), programs{i});
    if ~ismember(programs{i}, runningPrograms)
        fprintf(1, 'Warning: Found configuration metadata for program ''%s'' which is not currently running.\n', programs{i});
    else
        try
            %TO090706E
            if ~isempty(progmanagerGuisConfig)
                if isfield(progmanagerGuisConfig, programs{i})
                    guiNames = fieldnames(progmanagerGuisConfig.(programs{i}).guis);
                    runningGuis = fieldnames(progmanagerglobal.programs.(programs{i}).guinames);
                    for j = 1 : length(guiNames)
                        if ~ismember(guiNames{j}, runningGuis)
                            fprintf(1, 'Warning: Found configuration metadata for gui ''%s:%s'', which is not currently running.\n', programs{i}, guiNames{j});
                        else
                            %TO120905O - Only change the x, y values for position, not the width and height. -- Tim O'Connor 12/9/05
                            pos = get(progmanagerglobal.programs.(programs{i}).guinames.(guiNames{j}).fighandle, 'Position');
                            pos(1:2) = progmanagerGuisConfig.(programs{i}).guis.(guiNames{j}).position(1:2);
                            set(progmanagerglobal.programs.(programs{i}).guinames.(guiNames{j}).fighandle, 'Position', pos);
                            set(progmanagerglobal.programs.(programs{i}).guinames.(guiNames{j}).fighandle, 'Visible', progmanagerGuisConfig.(programs{i}).guis.(guiNames{j}).visible);
% fprintf(1, '%s:%s\n  Position: %s\n  Visible: %s\n', programs{i}, guiNames{j}, mat2str(progmanagerGuisConfig.(programs{i}).guis.(guiNames{j}).position), ...
%     progmanagerGuisConfig.(programs{i}).guis.(guiNames{j}).visible);
                        end
                    end
                end
            end
        catch
            fprintf(2, 'Warning: Failed to properly load GUI metadata for program ''%s'': %s', programs{i}, lasterr);
        end

        %TO091406A - If settings are not found, just skip them. -- Tim O'Connor 9/14/06
        if exist(fullfile(loadPath, [progmanagerglobal.programs.(programs{i}).mainGUIname '.settings'])) == 2
            loadedSettings = load(fullfile(loadPath, [progmanagerglobal.programs.(programs{i}).mainGUIname '.settings']), '-mat');
            %TO033008B - Moved `setProgramSettings` inside the `if exist...` statement. Only try loading them if the file exists in the first place.
            try
                setProgramSettings(progmanager, progmanagerglobal.programs.(programs{i}).guinames.(progmanagerglobal.programs.(programs{i}).mainGUIname).fighandle, ...
                    loadedSettings.settings);
            catch
                warning('Failed to properly load settings for program ''%s'': %s', programs{i}, getLastErrorStack);
            end
        else
            fprintf('%s - No settings found for ''%s'' - %s\n', datestr(now), progmanagerglobal.programs.(programs{i}).mainGUIname, fullfile(loadPath, [progmanagerglobal.programs.(programs{i}).mainGUIname '.settings']));
        end
    end
end

setDefaultCacheValue(this, 'lastConfigDir', loadPath);%TO120705D

setWindowsMenuItems(this, 'toggle');

fprintf(1, '%s - Configurations loaded from %s\n', datestr(now), loadPath);

return;