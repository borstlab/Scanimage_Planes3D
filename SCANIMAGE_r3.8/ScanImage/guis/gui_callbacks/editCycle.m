function editCycle
	global gh state
	if ~state.cycle.cycleOn
		seeGUI('gh.standardModeGUI.figure1');
	else
		seeGUI('gh.cycleGUI.figure1');
	end
