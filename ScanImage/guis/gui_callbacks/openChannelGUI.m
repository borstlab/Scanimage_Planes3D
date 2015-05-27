function openChannelGUI
%% function openChannelGUI

%% CHANGES
% VI043008A Vijay Iyer 4/30/2008 -- Handle the 'focus-only merge' option
% VI111708A Vijay Iyer 11/17/2008 -- Handle the 'blue as gray' option
% VI011109A Vijay Iyer 1/09/2009 -- Remove merge handling from this function, restoring function to 3.0 form
%
%% ******************************************
global state gh

updateCurrentFigure;    

%%%VI011109A: Removed %%%%%%%%%%%
% if state.acq.channelMerge
%     set(gh.channelGUI.cbMergeFocusOnly,'Enable','on'); %VI043008A
%     set(gh.channelGUI.cbMergeBlueAsGray,'Enable','on'); %VI111708A
% else
%     set(gh.channelGUI.cbMergeFocusOnly,'Enable','off'); %VI043008A
%     set(gh.channelGUI.cbMergeBlueAsGray,'Enable','off'); %VI111708A
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

seeGUI('gh.channelGUI.figure1');