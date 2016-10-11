function newCycleLength()
	global state
	
	if ~iscell(state.cycle.cycleParts)
		state.cycle.cycleParts={''};
		state.cycle.cyclePaths={''};
	end
	len=length(state.cycle.cycleParts);
	if len<state.cycle.length
		for i=(len+1):state.cycle.length
			state.cycle.cycleParts(i)={''};
			state.cycle.cyclePaths(i)={''};
			state.cycle.cycleRepeats(i)=1;
			state.cycle.cycleDX(i)=0;
			state.cycle.cycleDY(i)=0;
			state.cycle.cycleDZ(i)=0;
			state.cycle.cycleTimeDelay(i)=10;
			state.cycle.cycleReturnHome(i)=1;
			state.cycle.cycleStartingPosition(i)=0;
			state.cycle.cycleAveraging(i)=0;
			state.cycle.cycleNumberOfZSlices(i)=1;
			state.cycle.cycleNumberOfFrames(i)=1;
			state.cycle.cycleZStepPerSlice(i)=0.5;
		end
	elseif len>state.cycle.length & len>=1
		state.cycle.cycleParts(state.cycle.length+1:len)=[];
		state.cycle.cyclePaths(state.cycle.length+1:len)=[];
		state.cycle.cycleRepeats(state.cycle.length+1:len)=[];
		state.cycle.cycleDX(state.cycle.length+1:len)=[];
		state.cycle.cycleDY(state.cycle.length+1:len)=[];
		state.cycle.cycleDZ(state.cycle.length+1:len)=[];
		state.cycle.cycleTimeDelay(state.cycle.length+1:len)=[];
		state.cycle.cycleReturnHome(state.cycle.length+1:len)=[];
		state.cycle.cycleStartingPosition(state.cycle.length+1:len)=[];
		state.cycle.cycleAveraging(state.cycle.length+1:len)=[];
		state.cycle.cycleNumberOfZSlices(state.cycle.length+1:len)=[];
		state.cycle.cycleNumberOfFrames(state.cycle.length+1:len)=[];
		state.cycle.cycleZStepPerSlice(state.cycle.length+1:len)=[];
	end
	
		