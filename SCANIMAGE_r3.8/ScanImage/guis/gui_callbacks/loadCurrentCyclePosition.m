function loadCurrentCyclePosition(pos)

	global state
	
	if nargin<1
		pos=state.internal.position;
	end

	if ~iscell(state.cycle.cycleParts)
		newCycleLength;
		disp(['changeCyclePosition: Impossible error: state.cycle.cycleParts is not cellarray']);
		return
	end

	state.cycle.cycleConfigName=state.cycle.cycleParts{pos};
	state.cycle.cycleConfigPath=state.cycle.cyclePaths{pos};
	state.cycle.repeats=state.cycle.cycleRepeats(pos);
	state.cycle.xStep=state.cycle.cycleDX(pos);
	state.cycle.yStep=state.cycle.cycleDY(pos);
	state.cycle.zStep=state.cycle.cycleDZ(pos);
	state.cycle.timeDelay=state.cycle.cycleTimeDelay(pos);
	state.cycle.returnHome=state.cycle.cycleReturnHome(pos);
	state.cycle.startingPosition=state.cycle.cycleStartingPosition(pos);
	state.cycle.averaging=state.cycle.cycleAveraging(pos);
	state.cycle.numberOfZSlices=state.cycle.cycleNumberOfZSlices(pos);
	state.cycle.numberOfFrames=state.cycle.cycleNumberOfFrames(pos);
	state.cycle.zStepPerSlice=state.cycle.cycleZStepPerSlice(pos);
	state.internal.oldPosition=pos;

	global gh
	updateAllGUIVars(gh.cycleGUI);

