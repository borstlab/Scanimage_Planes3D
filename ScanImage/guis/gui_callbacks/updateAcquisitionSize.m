function  updateAcquisitionSize(varargin)
%% function  updateAcquisitionSize(varargin)
%Shared callback logic for handling updates to the number of frames and/or number of slices
%
%   updateAquisitionSize()
%   updateAcquisitionSize(h)
%   updateAcquisitionSize(preallocateMem)
%       h: (Optional) handle to uicontrol that led to this function execution (via a callback)
%       preallocateMem: <OPTIONAL> Logical indicating, if true, to call preallocateMemory() at end of call
%
%% NOTES
%   Function rewritten from scratch -- see MOLD file for earlier version. -- Vijay Iyer 1/6/11
%
%% CREDITS
%   Created 1/6/11, by Vijay Iyer
%% *********************************************************

global state


if ~nargin || ishandle(varargin{1})
    preallocateMem = true;
else 
    preallocateMem = varargin{1};
end

if preallocateMem 
    preallocateMemory;
end

%Flag all Pockels cells, so they regenerate data for the right # of frames.
%TODO: Review whether this flagging is actually needed/desired
if state.init.eom.pockelsOn
    state.init.eom.changed(:) = 1;
end

updateExternallyTriggered(); 