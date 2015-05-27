function startZoom

% Function that will start the aiZoom DAQ device.

	global state
	if ~state.init.autoReadZoom
		state.acq.rboxZoomSetting=-1;
		return	
	end
	
	status=state.internal.statusString;
	setStatusString('Reading Zoom...');

	start(state.init.aiZoom);

	while strcmp(state.init.aiZoom.Running, 'On')
	end
	setStatusString(status);
