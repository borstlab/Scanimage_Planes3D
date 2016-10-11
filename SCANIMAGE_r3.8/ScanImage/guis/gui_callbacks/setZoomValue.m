function setZoomValue(zoomVal,blockListener)
%% function setZoomValue(zoomVal)
% Sets zoom to a new value, accounting for all constraints and possible side-effects
%
%% SYNTAX
%   setZoomValue(zoomVal)
%       zoomVal: New zoom value
%       blockListener: <Default=false> If true, call to updateRSPs() is suppressed
%% NOTES
%   This helper was created to unify handling in mainControls and in genericKeyPressFuncton()
%   
%% CREDITS
%   Created 1/09/09, by Vijay Iyer
%% ***************************************************

global state

%oldZoomFactor = state.acq.zoomFactor;

if state.hSI.isSubUnityZoomAllowed
	state.acq.zoomFactor = zoomVal;
else
	state.acq.zoomFactor = max(zoomVal,1);
end

% %Flag if the fill fraction (line period) may have changed due to the zoom factor change
% if state.acq.zoomFactor >= state.acq.baseZoomFactor && oldZoomFactor < state.acq.baseZoomFactor
%     state.internal.fillFracChange = 1;
% elseif state.acq.zoomFactor < state.acq.baseZoomFactor
%     state.internal.fillFracChange = 1;
% else
%     state.internal.fillFracChange = 0;
% end

%This updates GUI controls, and makes acquisition parameter changes (if any)
updateZoom();

if nargin < 2 || ~blockListener
    updateRSPs();
end