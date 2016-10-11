function saveUserSettingsPath
global state
try
    userPath=state.userSettingsPath;
    scanimagepath=fileparts(which('scanimage'));
    save(fullfile(scanimagepath,['lastUserPath.mat']),'userPath', '-mat');
catch
    beep;
    disp('Unable to save user file path');
end



