function chooseCycleConfig
	[fname, pname]=uigetfile('*.cfg', 'Choose configuration');

	if isnumeric(fname)
		return
	end
	
	periods=findstr(fname, '.');
	if any(periods)								
		fname=fname(1:periods(1)-1);
	else
		disp('chooseCycleConfig: Error: found file name without extension');
		return
	end

	global state
	state.cycle.cycleConfigName=fname;
	state.cycle.cycleConfigPath=pname;
	state.internal.cycleChanged=1;
	
