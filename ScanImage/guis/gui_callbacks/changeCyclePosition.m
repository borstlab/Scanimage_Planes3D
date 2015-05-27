function changeCyclePosition(handle)
	global state
	state.internal.position=floor(state.internal.position);
	updateGUIByGlobal('state.internal.position');
	saveCurrentCyclePosition;
	loadCurrentCyclePosition;
			