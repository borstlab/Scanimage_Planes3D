function flushAOData(refresh)
%% function flushAOData
% Function that clears previous buffers. It also reloads data to output device buffers...
%
%% SYNTAX
%   flushAOData(refresh)
%       refresh: <OPTIONAL; Default=true> If true, current mirror data is loaded to output buffers after clear operation.
%
%% NOTES
%   Completely rewritten to use new DAQmx interface. To see prior version, see .MOLD file. -- Vijay Iyer 8/25/09
%
%% CHANGES
%   VI090909A: Handle cases where call is made before any mirror data has been created -- Vijay Iyer 9/9/09
%   VI091309A: Removed superfluous call to set 'sampQuantSampPerChan' property, as this is set in startXXX() methods -- Vijay Iyer 9/13/09
%   VI091509A: Removed unneeded call to Task.stop(). Task should be stopped in all cases leading up to this function. -- Vijay Iyer 9/15/09
%   VI070810A: Add option that allows refresh operation to be skipped -- Vijay Iyer 7/8/10
%
%% CREDITS
% Created 9/1/09, by Vijay Iyer
%
%% **************************************************************************************

global state 

%%%VI070810A
if nargin < 1 || isempty(refresh)
    refresh = true;
end

%Handle the data 'flushing'
tasks = [state.init.hAO];
if state.init.eom.pockelsOn
    tasks(end+1) = state.init.eom.hAO;
end

%stop(tasks); %VI091509A
tasks.control('DAQmx_Val_Task_Unreserve'); %This is what actually clears the data

%Reload output buffer (replacing previous putDataFocus/Grab() calls here)
if refresh && isfield(state.acq,'mirrorDataOutput') %VI070810A %VI090909A
    state.init.hAO.writeAnalogData(state.acq.mirrorDataOutput);
    %state.init.hAO.set('sampQuantSampPerChan', state.init.hAO.get('bufOutputBufSize') * state.acq.numberOfFrames); %VI091309A: Removed  %This sets # of repeats
end
