function alterDAQ_NewNumberOfFrames
%% function alterDAQ_NewNumberOfFrames
% Function that handles change in the number of frames
%
%% NOTES
%   Virtually all functionality has been removed. Not clear if the remaining EOM change flag is even still needed. -- Vijay Iyer 9/9/09
%
%% MODIFICATIONS
% VI031108A Vijay Iyer 3/11/08 -- Use infinite acquisition in GRAB mode now...
% VI011609A Vijay Iyer 1/16/09 -- Changed state.init.pockelsOn to state.init.eom.pockelsOn 
% VI082809A Vijay Iyer 8/28/09 -- Changes to use new DAQmx interface
% VI090309A Vijay Iyer 9/3/09 -- No need to change sampQuantSampPerChan property anymore, or to stopGrab() (not sure why this was ever needed) -- Vijay Iyer 9/3/09
%
%% *******************************************************

global state

%%%VI090309A: Removed %%%%%%%%%%%%%
% stopGrab;
% % GRAB output: set number of frames in GRAB output object to drive mirrors
% %set(state.init.ao2, 'RepeatOutput', (state.acq.numberOfFrames -1)); %VI082809A
% state.init.hAO.set('sampQuantSampPerChan', state.init.hAO.get('bufOutputBufSize') * state.acq.numberOfFrames); %VI082809A: This sets # of repeats
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%TPMODPockels
if state.init.eom.pockelsOn == 1			% and pockel cell, if on %VI011609A
    state.init.eom.changed(:) = 1;
end

% GRAB acquisition: set up total acquisition duration
%set(state.init.ai, 'SamplesPerTrigger', state.internal.samplesPerFrame*state.acq.numberOfFrames); %VI031108A
%set(state.init.ai,'SamplesPerTrigger',inf); %VI031108A %VI082809A: No longer needed, as this is set once at onset via DAQmx_Val_ContSamps
