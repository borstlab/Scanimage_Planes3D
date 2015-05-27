function resetConfigurationNeedsSaving()
%RESETCONFIGURATIONNEEDSSAVING Reset configurationNeedsSaving flag and update GUI props accordingly

global state gh

state.internal.configurationNeedsSaving = 0;
set(gh.configurationControls.pbSaveConfig, 'Enable','off','ForegroundColor',[0 0 0]);
set(gh.configurationControls.configurationName,'BackgroundColor',get(0,'defaultUIControlBackgroundColor'));
