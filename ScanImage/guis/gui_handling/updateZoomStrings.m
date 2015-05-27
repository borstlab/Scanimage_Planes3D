function updateZoomStrings
% This functin updates the strings in the zoom buttons
% to be correct with respect to the 
% actual zoom setting.

global state gh
zoomstr= num2str(state.acq.zoomFactor);

if length(zoomstr)==1
    zoomstr=['00' zoomstr];
elseif length(zoomstr)==2
    zoomstr=['0' zoomstr];
elseif length(zoomstr)>3
    zoomstr=zoomstr(1:3);
end

state.acq.zoomhundreds=str2num(zoomstr(1));
state.acq.zoomtens=str2num(zoomstr(2));
state.acq.zoomones=str2num(zoomstr(3));
updateGUIByGlobal('state.acq.zoomhundreds');
updateGUIByGlobal('state.acq.zoomtens');
updateGUIByGlobal('state.acq.zoomones');