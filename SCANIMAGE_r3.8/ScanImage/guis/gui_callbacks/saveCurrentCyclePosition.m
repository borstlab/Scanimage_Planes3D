function saveCurrentCyclePosition

	global state
	
	if ~iscell(state.cycle.cycleParts)
		newCycleLength;
		disp(['changeCyclePosition: Impossible error: state.cycle.cycleParts is not cellarray']);
		return
	end

	pos=state.internal.oldPosition;
	state.cycle.cycleParts{pos}=state.cycle.cycleConfigName;
	state.cycle.cyclePaths{pos}=state.cycle.cycleConfigPath;
	state.cycle.cycleRepeats(pos)=state.cycle.repeats;
	state.cycle.cycleDX(pos)=state.cycle.xStep;
	state.cycle.cycleDY(pos)=state.cycle.yStep;
	state.cycle.cycleDZ(pos)=state.cycle.zStep;
	state.cycle.cycleTimeDelay(pos)=state.cycle.timeDelay;
	state.cycle.cycleReturnHome(pos)=state.cycle.returnHome;
	state.cycle.cycleStartingPosition(pos)=state.cycle.startingPosition;
	state.cycle.cycleAveraging(pos)=state.cycle.averaging;
	state.cycle.cycleNumberOfZSlices(pos)=state.cycle.numberOfZSlices;
	state.cycle.cycleNumberOfFrames(pos)=state.cycle.numberOfFrames;
	state.cycle.cycleZStepPerSlice(pos)=state.cycle.zStepPerSlice;
	
	state.internal.oldPosition=pos;

