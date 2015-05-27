function initializeTIFStreamName()
%% function initializeTIFStreamName()
%   Initializes name of TIF stream, handling case of chunked files
%
%% NOTES
%   This is a direct cut&paste from startGrab(), allowing code to be invoked from startGrab() and nextTriggerFcn() -- Vijay Iyer 9/20/09
%
%   TODO: Revisit and determine whether this functionality could/should be moved into acquisitionStartedFcn() -- Vijay Iyer 9/20/09
%
%% CREDITS
%   Created 9/20/09, by Vijay Iyer
%% ***************************************************************************

global state

if ~isinf(state.acq.framesPerFile)
    state.files.tifStreamFileName  = [state.files.fullFileName '_001.tif']; %VI070109A
else
    state.files.tifStreamFileName = [state.files.fullFileName '.tif']; %VI070109A
end