function updateTriggerParameters(handle)
%% function updateTriggerParameters
%   Handles updates to parameters in the Trigger GUI
%
%% NOTES
%   Function is an INI-file callback, so it is invoked either upon adjusting the GUI control or loading a CFG file
%
%% CHANGES
%   VI123109A: Use updateExternallyTriggered() to handle possible dependent state changes to state.acq.externallyTriggered -- Vijay Iyer 12/31/09
%   VI123109B: Disable stack controls in case where pure next trigger is employed
%   VI090810A: Start trigger menu no longer allows nothing to be selected (if any sources are available) -- Vijay Iyer 9/8/10
%   VI090810B: Disallow Next Trigger Only mode if no next trigger is selected -- Vijay Iyer 9/8/10
%
%% CREDITS
%   Created 9/17/09, by Vijay Iyer.
%% *******************************************************************************************************

global state gh

if isempty(state.init.hDAQmx) %Don't bother executing until after setupDAQDevices_Common() 
    return;
end


% %Handle case where no external next trigger is available - overrides any CFG settings
if isempty(state.acq.nextTrigInputTerminal)
    state.acq.pureNextTriggerMode = 0;
    updateGUIByGlobal('state.acq.pureNextTriggerMode');
    set(gh.triggerGUI.cbPureNextTrigger,'Enable','off');
else
	 set(gh.triggerGUI.cbPureNextTrigger,'Enable','on');
end

%Handle 'Next Trigger Only' mode
startTrigControls = get(gh.triggerGUI.pnlStartTrig,'Children');
if state.acq.pureNextTriggerMode
    
    %%%VI090810B%%%%%
	if isempty(state.acq.nextTrigInputTerminal)
        msgbox('Cannot enable Next Trigger Only mode with no Next Trigger Source selected','No Next Trigger Source','warn');
        state.acq.pureNextTriggerMode = 0;
        updateGUIByGlobal('state.acq.pureNextTriggerMode');
        updateTriggerParameters();
        return;        
	end
    %%%%%%%%%%%%%%%%%
    
    set(startTrigControls,'Enable','off');
    set(gh.triggerGUI.pmNextTrigNextMode,'Enable','off'); %Force 'advance' mode    

    %state.acq.startTrigInputTerminalGUI = 1; %VI090810A
    state.acq.nextTrigNextModeGUI = 2;   %Force 'advance' mode    
    updateGUIByGlobal('state.acq.nextTrigNextModeGUI');    
    
    %%%VI123109A: Removed %%%%%%
    %     %Force state.acq.externallyTriggered state var on
    %     state.acq.externallyTriggered = 1;
    %     updateGUIByGlobal('state.acq.externallyTriggered');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%VI090810A: Removed %%%%%
    %Hide external/internal control
    %set(gh.mainControls.tbExternalTrig,'Enable','off');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %VI123109B: Disable stack controls
    si_toggleStackAcquisitionAvailability(false);
    
else
	set(startTrigControls,'Enable','on');
	set(gh.triggerGUI.pmNextTrigNextMode,'Enable','on'); %Allow selection of 'arm' or 'advance'
    
    %%%VI090810A: Removed %%%%
    %Restore external/internal trigger control
    %set(gh.mainControls.tbExternalTrig,'Enable','on');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %VI123109B: Enable stack controls
    si_toggleStackAcquisitionAvailability(true);
    
end   

%Decode GUI controls with 'direct translation' -- the menu string is the value to store locally
% directTranslationControls = {'startTrigInputTerminal' 'nextTrigInputTerminal' 'startTrigEdge' 'nextTrigEdge'};
directTranslationControls = {'startTrigEdge' 'nextTrigEdge'};
for i=1:length(directTranslationControls)
    guiControls = getGuiOfGlobal(['state.acq.' directTranslationControls{i} 'GUI']);
    guiControls = cellfun(@(x)eval(x),guiControls);    
    options = get(guiControls,'String');
    idx = state.acq.([directTranslationControls{i} 'GUI']);
    state.acq.(directTranslationControls{i}) = options{idx};       
end

%Decode other GUI controls
state.acq.nextTrigAutoAdvance = state.acq.nextTrigNextModeGUI - 1; %Converts from 1-based to 0-based indexing
state.acq.nextTrigStopImmediate = double(~(state.acq.nextTrigStopModeGUI - 1)); %Store as double, to allow proper CFG handling

%Update display based on mode
if state.acq.nextTrigAutoAdvance
    gapAdvanceEnable = 'on';
else
    gapAdvanceEnable = 'off';
end
set(gh.triggerGUI.cbGapAdvance,'Enable',gapAdvanceEnable);

%Update internal flag variables 
state.internal.gapFreeAdvanceNext = (~isempty(state.acq.nextTrigInputTerminal) && state.acq.nextTrigAutoAdvance && ~state.acq.nextTrigAdvanceGap);

%Update dependent properties
updateExternallyTriggered(); %VI123109A


