function closeEpiShutter
	global state
	if ~isempty(state.shutter.epiShutterLine)
		putvalue(state.shutter.epiShutterLine, ~state.shutter.epiShutterOpen);
	end