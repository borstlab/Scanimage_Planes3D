function [ userDir ] = ephus_util_newUserFolder( varargin )
%EPHUS_UTIL_NEWUSERFOLDER Creates a new user folder
%
%   Arguments:
%       doWithConfigs:  an optional, boolean argument; if true, the generated user folder will 
%       include hardware-specific settings, such as an i-steps pulsejacker cyle, hotswitches, 
%       and a standard configuration; an example "init" (aka startup) file will also be included.
%
%   1. Prompts the user to select an existing, empty folder (e.g. C:\DATA\Jane)
%   2. Puts default "settings" folder tree into the selected folder
%       a. Unzips the ephustempxyz.zip settings archive into a temporary folder, C:\ephustempxyz
%       b. Clones the temporary settings to the selected folder
%       c. Remove the temporary folder
%   3. Customizes the XSG settings
%       a. Prompt user for initials, used for experiment name
%   4. Ask user whether to load the "standard" configuration
%
% 2010-06-09 -- Ben Suter
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global progmanagerglobal

doWithConfigs = false;
if nargin > 0
    doWithConfigs = varargin{1};
end

% Create a folder for this user and fill with standard folders and settings
startPath = 'C:\DATA\';
userDir = uigetdir(startPath, 'Choose a new folder to contain your Ephus data and settings. It must currently be empty. E.g. C:\DATA\Jane');
if userDir == 0
    return;
end

list = dir(fullfile(userDir, '*'));
if numel(list) > 2
    % An empty directory listing contains two items: "." and ".."
    error('The new user folder must be empty');
end

tempDir = 'C:\ephustempxyz';
if exist(tempDir)
    error('A folder called %s already exists. Either delete that folder, or rename it, and then retry.', tempDir);
end

if doWithConfigs
    % this archive has hotswitches, configs, isteps pulsejacker cycle, and an example init file
    stdUserFolderArchive = 'ephustempxyz-withconfigs.zip';
else
    stdUserFolderArchive = 'ephustempxyz.zip';
end

if ~exist(stdUserFolderArchive)
    error('Unable to find the standard user folder template, which should be a ZIP file on your Matlab path: %s', stdUserFolderArchive);
end

fns = unzip(stdUserFolderArchive, 'C:\');
% The unzip command is buggy - it sets the files to read-only, so we need to change that ourselves
for i=1:numel(fns)
    fileattrib(fns{i}, '+w');
end

% Clone the template settings
ephus_util_cloneSettings(tempDir, userDir);

% And remove the temporary folder created when we unzipped
rmdir(tempDir, 's');


% Customize the XSG experiment prefix (loop until valid initials or cancelled)
while ( true )
    allowed = [ 'A':'Z', 'a':'z' ];
    answers = inputdlg('Prefix for experiments, typically your initials, e.g. JD (or leave blank to choose later)', 'New standard user folder');
    if isempty(answers)
        break; % user cancelled
    elseif all(ismember(answers{1}, allowed))
        expPrefix = answers{1};
        % Modify the xsg.settings file to include this prefix
        xsgSettingsFile = fullfile(userDir, 'settings\configs\standard', 'xsg.settings');
        xsg = load(xsgSettingsFile, '-mat');
        settings = xsg.settings;
        try
            set(settings.xsg, 'initials', expPrefix);
        catch ME
            warning('Unable to update the initials field of the xsg.settings: %s', getLastErrorStack(ME));
        end
        save(xsgSettingsFile, 'settings', '-mat');
        break;
    else
        % Invalid expPrefix, it does not match the XSG validation rules for this field
        uiwait(msgbox('Please use only letters', 'Invalid initials', 'warn', 'modal'));
    end
end

if doWithConfigs && isprogram(progmanager, 'xsg') % only if Ephus is running and we created configs
    % Optionally, load the new user's standard configuration
    % TODO: Will the new config dir be cached?
    response = questdlg('Would you like to load this new user''s standard configuration now?','Load configuration');
    if strcmp(response, 'Yes')
        stdConfig = fullfile(userDir, 'settings\configs\standard');
        loadConfigurations(progmanager, stdConfig);

        % If the screen is smaller than recommended, ensure all windows are visible
        ssz = get(0, 'ScreenSize');
        if ssz(3) < 1920 || ssz(4) < 1200
            disp('The standard configuration assumes a 1920x1200 resolution screen.');
            disp('Your screen appears smaller, so windows have been shifted, to avoid losing them off-screen');

            progs = fieldnames(progmanagerglobal.programs);
            for p=1:numel(progs)
                guis = fieldnames(progmanagerglobal.programs.(progs{p}).guinames);
                for g=1:numel(guis)
                    movegui(progmanagerglobal.programs.(progs{p}).guinames.(guis{g}).fighandle, 'onscreen');
                end
            end
        end

        % Now try to load the XSG configuration as well
        if ~isempty(expPrefix)
            try
                % Allow
                setGlobal(progmanager, 'configurationEnabled', 'xsg', 'xsg', 1);
                setProgramSettings(progmanager, ...
                    progmanagerglobal.programs.xsg.guinames.(progmanagerglobal.programs.xsg.mainGUIname).fighandle, ...
                    settings);
                setGlobal(progmanager, 'configurationEnabled', 'xsg', 'xsg', 0);
                experimentSavingGui('configurationEnabled_Callback', xsg_getHandle(), [], []); % indicate checkbox clicked, to update color
            catch
                warning('Failed to properly load settings for program ''xsg'': %s', getLastErrorStack);
            end
        end
    end
end

end




