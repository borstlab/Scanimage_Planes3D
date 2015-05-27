function updateChannelMergeParameters(handle)
%% function updateChannelMergeParameters(handle)
% Callback function that handles update to the channel merge display parameters 
%
%% NOTES
%   Being an INI-named callback allows this to be called during either a GUI control or CFG/USR file loading event
%
%% CREDITS
%   Created 1/11/09, by Vijay Iyer
%% ******************************************************************
global state gh

%Determine whether to update the merge figure visibility
updateMergeFigure = isfield(state.internal,'MergeFigure'); % && strcmpi(get(gh.channelGUI.figure1,'Visible'),'off');

if state.acq.channelMerge
    if updateMergeFigure
        set(state.internal.MergeFigure,'Visible','on');
    end
    set(gh.channelGUI.cbMergeFocusOnly,'Enable','on'); 
    set(gh.channelGUI.stMergeColor,'Enable','on');
else
    if updateMergeFigure
        set(state.internal.MergeFigure,'Visible','off');
    end
    set(gh.channelGUI.cbMergeFocusOnly,'Enable','off'); 
    set(gh.channelGUI.stMergeColor,'Enable','off');
end

state.acq.mergeColor=[]; 
for i = 1:state.init.maximumNumberOfInputChannels
    state.acq.mergeColor = [state.acq.mergeColor eval(['state.acq.mergeColor' num2str(i)])]; 
    if state.acq.channelMerge
        set(gh.channelGUI.(['pmMergeColor' num2str(i)]),'Enable','on');
    else
        set(gh.channelGUI.(['pmMergeColor' num2str(i)]),'Enable','off');
    end            
end



        
        