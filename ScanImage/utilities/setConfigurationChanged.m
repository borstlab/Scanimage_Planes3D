function setConfigurationChanged()
%SETCONFIGURATIONCHANGED 1) Set configurationNeedsSaving and configurationChanged flags and 2) update GUI props accordingly

global state gh

%Set Configuration Changed flag
state.internal.configurationChanged=1;
set(gh.configurationControls.pbApplyConfig,'Enable','on','ForegroundColor',[0 .5 0]);
turnOffExecuteButtons('state.internal.configurationChanged');

%Flag need to save, as well
setConfigurationNeedsSaving();
