function varargout=saveCurrentCycleAs
% Saves current cycle under new name
	global state

	startPath = state.hSI.getLastPath('cycleLastPath');
	[fname, pname]=uiputfile({'*.cyc'},'Choose CYC File...',startPath);
	if ~isnumeric(fname)						
        [~,f,e] = fileparts(fname);
        if strcmpi(e,'.cyc')
            state.cycle.cycleName = f;
        else
            state.cycle.cycleName = [f e];
        end
		state.cycle.cyclePath=pname;
        
		updateGUIByGlobal('state.cycle.cycleName');
		updateGUIByGlobal('state.cycle.cyclePath');
		saveCurrentCycle;
	end
