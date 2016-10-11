function setStatusString(st,color)
% This function sets the status field in mainControls.

    if nargin < 2 || isempty(color)
        color = [0 0 0];
    end

	global state gh;
    
	state.internal.statusString=st;
    set(gh.mainControls.statusString,'ForegroundColor',color);
	updateGUIByGlobal('state.internal.statusString');
	