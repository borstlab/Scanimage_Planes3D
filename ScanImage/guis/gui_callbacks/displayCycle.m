function displayCycle
	global state;
	
	pos=1;
	
	for pos=1:state.cycle.length
		disp(['POSITION #' num2str(pos) ' uses configuration ''' state.cycle.cycleParts{pos} '''']);
		disp(['  repeats: ' num2str(state.cycle.cycleRepeats(pos))]);
		if state.cycle.cycleAbsoluteX(pos)==1
			disp(['  x: ' num2str(state.cycle.cycleDX(pos)) ' microns will be the position after the acquisition']);
		else
			if state.cycle.cycleDX(pos)~=0
				disp(['  dx: ' num2str(state.cycle.cycleDX(pos)) ' microns moved after each acquisition']);
			end	
		end
		if state.cycle.cycleAbsoluteY(pos)==1
			disp(['  y: ' num2str(state.cycle.cycleDY(pos)) ' microns will be the position after the acquisition']);
		else
			if state.cycle.cycleDY(pos)~=0
				disp(['  dy: ' num2str(state.cycle.cycleDY(pos)) ' microns moved after each acquisition']);
			end
		end
		if state.cycle.cycleAbsoluteZ(pos)==1
			disp(['  z: ' num2str(state.cycle.cycleDZ(pos)) ' microns will be the position after the acquisition']);
		else
			if state.cycle.cycleDZ(pos)~=0
				disp(['  dz: ' num2str(state.cycle.cycleDZ(pos)) ' microns moved after each acquisition']);
			end
		end
		disp(['  pause: ' num2str(state.cycle.cycleTimeDelay(pos)) ' sec after each acquisition']);
		disp('');
	end