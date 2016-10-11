% PROGMANAGER/saveConfig - Save the configuration of an individual program.
%
% SYNTAX
%  saveConfig(progmanager, hObject)
%  saveConfig(progmanager, hObject, location)
%    progmanager - The program manager object.
%    locations - The disk location in which to save the configuration, if not supplied a prompt will be issued.
%
% NOTES
%  See TO062306F.
%  This is a copy & paste of progmanager/saveConfigurations.m.
%
% CHANGES
%  TO062306F: Allow individual program configurations to be saved. -- Tim O'Connor 6/23/06
%  TO071906D: Make sure `save` makes Matlab v6 compatible files. -- Tim O'Connor 7/19/06
%
% Created 6/23/06 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function saveConfig(this, hObject, varargin)
global progmanagerglobal;

%TO120705D: Use `getDefaultCacheValue` now. -- Tim O'Connor 12/7/05
lastConfigDir = getDefaultCacheValue(this, 'lastConfigDir');

if exist(lastConfigDir) ~= 7
    lastConfigDir = pwd;
%     warning('Cached configuration directory default is corrupted or missing, using `pwd` instead: %s', lastConfigDir);
end

if ~isempty(varargin)
    savePath = varargin{1};
else
    [saveFile, savePath] = uiputfile(fullfile(lastConfigDir, '*.settings'), 'Select a file in which to save this configuration.');
    if length(saveFile) == 1 & length(savePath) == 1
        if saveFile == 0 & savePath == 0
            return;
        end
    end
    if ~endsWithIgnoreCase(saveFile, '.settings')
        saveFile = [saveFile '.settings'];
        if exist(saveFile) == 2
            overwrite = questdlg(sprintf('%s already exists. Overwrite?', saveFile), 'Overwrite Existing Config', 'Yes', 'No', 'No');
            if strcmpi(overwrite, 'No')
                return;
            end
        end
    end
    
    saveFile = fullfile(savePath, saveFile);
end

setDefaultCacheValue(this, 'lastConfigDir', savePath);%TO120705D, TO120905E
progmanagerGuisConfig = [];

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

guiNames = fieldnames(progmanagerglobal.programs.(program_name).guinames);
for j = 1 : length(guiNames)
    progmanagerGuisConfig.(program_name).guis.(guiNames{j}).position = get(progmanagerglobal.programs.(program_name).guinames.(guiNames{j}).fighandle, 'Position');
    progmanagerGuisConfig.(program_name).guis.(guiNames{j}).visible = get(progmanagerglobal.programs.(program_name).guinames.(guiNames{j}).fighandle, 'Visible');
end

try
    saveProgramSettings(this, hObject, saveFile);
    saveCompatible(saveFile, 'progmanagerGuisConfig', '-append', '-mat');%TO071906D
catch
    warning('Failed to properly save settings for program ''s'': %s', program_name, lasterr);
end

fprintf(1, '%s - Configuration for %s saved in %s.\n', datestr(now), program_name, saveFile);

return;