function turnOnMenusFocus
% Controls to turn back on following FOCUS mode acquisition
%% CHANGES
%   VI101708A: imageGUI controls are now left on during FOCUS acquistion, so no need to turn them back on here
%   VI011609A: Changed state.init.pockelsOn to state.init.eom.pockelsOn -- Vijay Iyer 1/16/09
%   VI012511A: Use toggleGUI for cycleGUI controls; no cycle controls on MainControls anymore -- Vijay Iyer 1/25/11
%
%% ***********************************************
global gh state
%Added TP
set(gh.mainControls.Settings,'Enable','on');
set(gh.mainControls.File,'Enable','on');
%set(get(gh.imageGUI.figure1,'Children'),'Enable','on'); %VI101708A
%updateImageGUI; %VI101708A
%toggleGUI(gh.acquisitionGUI.figure1,'On');
%reconcileStandardModeSettings(true); %VI092210A: Removed %VI011310A
toggleCycleGUI([],[],[],'On'); %VI012511A
%set(gh.mainControls.cyclePosition, 'Enable', 'On'); %VI012511A
%set(gh.mainControls.positionToExecuteSlider, 'Enable', 'On'); %VI012511A
%imageControls('cbShowCrosshair_Callback',gh.imageControls.cbShowCrosshair);
%TPMODPockels
if state.init.eom.pockelsOn %VI011609A
    set([gh.powerControl.Settings gh.powerControl.maxPower_Slider],'Enable','on');  %TPMODPockels
end

%%%VI020209A%%%%%%%%%%
set(gh.configurationControls.pbLoadConfig,'Enable','On');
if state.internal.configurationNeedsSaving
    setConfigurationNeedsSaving(); %Refresh GUI changes associated with positive flag state
end
%%%%%%%%%%%%%%%%%%%%%%

%%%VI092710A/VI102810A%%%%
%VI102810A: Remove this for now...maybe add back in as an option (e.g. to support snapshot mode)
%if ~isempty(state.files.fastConfigAutoStartCachedConfig)    
%     loadStandardModeConfig(state.files.fastConfigAutoStartCachedConfig);
%     state.files.fastConfigAutoStartCachedConfig = '';    
%end
%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(gh.mainControls.figure1); %VI070208A
