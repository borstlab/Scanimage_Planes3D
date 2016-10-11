function out=openAndLoadUserSettings
% Allows user to select a settings file (*.ini) from disk and loads it
% Author: Bernardo Sabatini
out=0;

global state gh
status=state.internal.statusString;
setStatusString('Loading user settings...');

[fname, pname]=uigetfile('*.usr', 'Choose user settings file to load',state.userSettingsPath); %VI112310A
if ~isnumeric(fname)
    [~,~,ext] = fileparts(fname);    
    if isempty(ext) || ~strcmpi(ext,'.usr')
        fprintf(2,'WARNING: Invalid file extension provided. Cannot open USR file.\n');
        setStatusString('Can''t open file...');
        return
    end		
    openusr(fullfile(pname, fname));
    %cd(state.userSettingsPath); %VI111110A
    %%%VI120109A: Removed%%%%%%
    %     %TPMOD
    %     if isdir(state.userFcnGUI.UserFcnPath)
    %         files=dir([state.userFcnGUI.UserFcnPath '*.m']);
    %         state.userFcnGUI.UserFcnFiles = sortrows({files.name}'); % Sort names
    %         if ~isempty(state.userFcnGUI.UserFcnFiles)
    %             set(gh.userFcnGUI.UserFcnBrowser,'Value',1,'String',state.userFcnGUI.UserFcnFiles);
    %         else
    %             set(gh.userFcnGUI.UserFcnBrowser,'Value',1,'String',' ');
    %         end
    %     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end


setStatusString(status);
