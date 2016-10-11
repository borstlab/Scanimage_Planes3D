% PROGMANAGER/loadConfig - Load the configuration of an individual program.
%
% SYNTAX
%  loadConfig(progmanager, hObject)
%  loadConfig(progmanager, hObject, location)
%    progmanager - The program manager object.
%    locations - The disk location from which to load the configuration, if not supplied a prompt will be issued.
%
% NOTES
%  See TO062306F.
%  This is a copy & paste of progmanager/loadConfigurations.m.
%
% CHANGES
%  TO062306F: Allow individual program configurations to be saved. -- Tim O'Connor 6/23/06
%  TO091106H: Added a missing '%' in a format string for a warning message. -- Tim O'Connor 9/11/06
%
% Created 6/23/06 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function loadConfig(this, hObject, varargin)
global progmanagerglobal;

%TO120705D: Use `getDefaultCacheValue` now. -- Tim O'Connor 12/7/05
lastConfigDir = getDefaultCacheValue(this, 'lastConfigDir');

if exist(lastConfigDir) ~= 7
    lastConfigDir = pwd;
%     warning('Cached configuration directory default is corrupted or missing, using `pwd` instead: %s', lastConfigDir);
end

if ~isempty(varargin)
    loadPath = varargin{1};
else
    [loadFile, loadPath] = uigetfile(fullfile(lastConfigDir, '*.settings'), 'Select a file from which to load this configuration.');
    if length(loadFile) == 1 & length(loadPath) == 1
        if loadFile == 0 & loadPath == 0
            return;
        end
    end
    loadFile = fullfile(loadPath, loadFile);
end
if exist(loadFile) ~= 2
    error('Invalid progmanager configuration save file: %s', loadFile);
end

setDefaultCacheValue(this, 'lastConfigDir', loadPath);%TO120705D, TO120905E

%TO020405b - This makes life a lot easier.
if strcmpi(class(hObject), 'program') %TO122205A
    program_name = get(hObject, 'program_name');
    gui_name = get(hObject, 'main_gui_name');
    hObject = progmanagerglobal.programs.(program_name).(gui_name).guihandles.(lower(program_name));
elseif ishandle(hObject)
    fighandle=getParent(hObject,'figure');
    UserData=get(fighandle,'UserData');
    gui_name=UserData.guiname; % gui name.
    program_name=UserData.progname;
else
    %TO091405C - This used to just say "Invalid program/gui handle." -- Tim O'Connor
    error('Invalid program/gui handle. Class of expected handle: %s\n%s', class(hObject), getStackTraceString);
end

loadedGuiMetaInfo = [];
loadedSettings = load(loadFile, '-mat');

try
    if ~isfield(loadedSettings.settings, program_name)
        fprintf(1, '@progmanager/loadConfig: No configuration found for program ''%s'' in ''%s''.\n', program_name, loadFile);
        return;
    end
    
    setProgramSettings(progmanager, hObject, loadedSettings.settings);
    fields = fieldnames(loadedSettings.settings);
    for i = 1 : length(fields)
        delete(loadedSettings.settings.(fields{i}));
    end

    if isfield(loadedSettings, 'progmanagerGuisConfig')
        progmanagerGuisConfig = loadedSettings.progmanagerGuisConfig;
    else
        loadedGuiMetaInfo = load(fullfile(loadPath, 'guiMetaInfo.mat'), 'progmanagerGuisConfig', '-mat');
        progmanagerGuisConfig = loadedGuiMetaInfo.progmanagerGuisConfig;
    end
    if ~isempty(loadedSettings)
        guiNames = fieldnames(progmanagerglobal.programs.(program_name).guinames);
        for j = 1 : length(guiNames)
            set(progmanagerglobal.programs.(program_name).guinames.(guiNames{j}).fighandle, 'Position', progmanagerGuisConfig.(program_name).guis.(guiNames{j}).position);
            set(progmanagerglobal.programs.(program_name).guinames.(guiNames{j}).fighandle, 'Visible', progmanagerGuisConfig.(program_name).guis.(guiNames{j}).visible);
        end
    else
        fprintf(1, '@progmanager/loadConfig: No GUI layout information available for program ''%s'' in file ''%s''.\n', program_name, loadFile);
    end
catch
    warning('Failed to properly load settings for program ''%s'': %s', program_name, getLastErrorStack);%TO091106H
end

fprintf(1, '%s - Configuration for %s loaded from %s.\n', datestr(now), program_name, loadFile);

return;