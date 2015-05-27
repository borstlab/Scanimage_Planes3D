function turnOnMenus
%% function turnOnMenus
% Controls to turn back on following GRAB/LOOP mode acquisition
%
%% CHANGES
%   VI012511A: Use toggleGUI for cycleGUI controls; no cycle controls on MainControls anymore -- Vijay Iyer 1/25/11
%% **************************************************

%
global gh state
%Added TP
set(gh.mainControls.Settings,'Enable','on');
set(gh.mainControls.File,'Enable','on');
set(setdiff(findobj(gh.imageControls.figure1,'-not','Type','uipanel'),gh.imageControls.figure1),'Enable','on'); %VI022009A
enableLUTControls();
% state.internal.showRotBox=state.internal.oldRotBoxString;
% updateMainControlSize;
% set(gh.mainControls.showrotbox,'Enable','on');
set(gh.mainControls.pbBase, 'Enable', 'on');
set(gh.mainControls.pbSetBase, 'Enable', 'on');
%toggleGUI(gh.acquisitionGUI.figure1,'On');
%reconcileStandardModeSettings(true); %VI092210A: Removed %VI011310A
toggleCycleGUI([],[],[],'on');
%set(gh.mainControls.cyclePosition, 'Enable', 'On'); %VI012511A
%set(gh.mainControls.positionToExecuteSlider, 'Enable', 'On'); %VI012511A
%imageControls('cbShowCrosshair_Callback',gh.imageControls.cbShowCrosshair);
%TPMODPockels
if state.init.eom.pockelsOn %VI011609A
    enableEomGui(1);    %TPMODPockels
end
%%%VI020209A%%%%%%
if state.internal.showCfgGUI
    seeGUI('gh.configurationControls.figure1');
end
%%%%%%%%%%%%%%%%%%


%%%VI102810A/VI092710A%%%%
%VI102810A: Remove this for now...maybe add back in as an option (e.g. to support snapshot mode)
% if ~isempty(state.files.fastConfigAutoStartCachedConfig)    
%     loadStandardModeConfig(state.files.fastConfigAutoStartCachedConfig);
%     state.files.fastConfigAutoStartCachedConfig = '';    
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(gh.mainControls.figure1); %VI070208A
