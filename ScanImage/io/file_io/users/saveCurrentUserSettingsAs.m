function varargout=saveCurrentUserSettingsAs

global state

[fname, pname]=uiputfile('*.usr', 'Choose user settings file',state.userSettingsPath); %VI111110A

if ~isnumeric(fname)
    [~,f,e] = fileparts(fname);
    if strcmpi(e,'.usr')
        state.userSettingsName = f;
    else
        state.userSettingsName = [f e];
    end 
    state.userSettingsPath=pname;
    saveCurrentUserSettings;
end
