function turnOffMenus
%% NOTES
%   This function is called during GRAB/LOOP/SNAPSHOT acquisitions, preventing actions from occuring during acquisition
%% CHANGES
%   VI020309A: Hide configuration GUI now as well, but don't toggle the Show/Hide button -- Vijay Iyer 2/2/09
%   VI021309A: Hide alignment GUI -- Vijay Iyer 2/13/09
%   VI012511A: Use toggleGUI for cycleGUI controls; no cycle controls on MainControls anymore -- Vijay Iyer 1/25/11
%
%% ******************************************

global gh state
%Added TP
set(gh.mainControls.Settings,'Enable','off');
set(gh.mainControls.File,'Enable','off');
set(setdiff(findobj(gh.imageControls.figure1,'-not','Type','uipanel'),gh.imageControls.figure1), 'Enable', 'Off'); %VI022009A
% state.internal.oldRotBoxString=state.internal.showRotBox;
% if strcmp(state.internal.oldRotBoxString,'<<')
%     state.internal.showRotBox='>>';
%     updateMainControlSize;
% end
% set(gh.mainControls.showrotbox,'Enable','off');
set(gh.mainControls.pbBase, 'Enable', 'Off');
set(gh.mainControls.pbSetBase, 'Enable', 'Off');
%toggleGUI(gh.acquisitionGUI.figure1,'Off');
toggleCycleGUI([],[],[],'off');
%set(gh.mainControls.cyclePosition, 'Enable', 'Off'); %VI012511A
%set(gh.mainControls.positionToExecuteSlider, 'Enable', 'Off'); %VI012511A
hideGUI('gh.configurationControls.figure1'); %VI012909A
updateGUIByGlobal('state.internal.showAlignGUI','Value',0,'Callback',1); %VI021309A
enableEomGui(0);    %TPMODPockels


