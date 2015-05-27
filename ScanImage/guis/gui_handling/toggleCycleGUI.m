function toggleCycleGUI(~,~,~,mode)
	global state gh;

	CFG_NAME_COL_IDX = 1;

	if nargin < 4 || isempty(mode)
		if state.cycle.cycleOn
			mode = 'on';
		else
			mode = 'off';
		end
		force = false;
	else
		force = true;
	end

	excludeable = {gh.cycleGUI.pbCycleCountInc gh.cycleGUI.pbCycleCountDec gh.cycleGUI.pbCycleIterationInc ...
                    gh.cycleGUI.pbCycleIterationDec gh.cycleGUI.pbCycleReset gh.cycleGUI.pbDropRow gh.cycleGUI.stIterationsPerLoop gh.cycleGUI.etIterationsPerLoop};

	if strcmpi(mode,'on')
		if ~state.cycle.cycleOn
			set(gh.cycleGUI.cbCycleOn,'Enable','on');
			return;
		end
		
		exclude = {};
		
		if state.cycle.autoReset
			exclude = [exclude excludeable];
		end
		
		% the 'add' and 'remove' buttons' state is dependent on the which
		% column is selected, and what that cell contains...
		indices = selectedTableCells(gh.cycleGUI.tblCycle);
		tableData = get(gh.cycleGUI.tblCycle,'Data');
		if isempty(indices)
			exclude = [exclude gh.cycleGUI.pbAdd gh.cycleGUI.pbClear];
		else
			if any(indices(:,2) == CFG_NAME_COL_IDX)
				exclude = [exclude gh.cycleGUI.pbClear];
				if isempty(tableData{indices(1),indices(2)})
					exclude = [exclude gh.cycleGUI.pbClear];
				else
					exclude = [exclude gh.cycleGUI.pbAdd];
				end
			else
				exclude = [exclude gh.cycleGUI.pbAdd];
			end
		end
		
		toggleGUI(gh.cycleGUI.figure1,mode,exclude);
		
		% explicitly set the count and cycle-name textfields to inactive
		set([gh.cycleGUI.etCycleLength gh.cycleGUI.etCycleName],'Enable','inactive');
		set([gh.cycleGUI.etCycleLength gh.cycleGUI.etCycleName],'BackgroundColor',[.831 .816 .784]);
	else
		toggleGUI(gh.cycleGUI.figure1,'off',gh.cycleGUI.cbCycleOn);
	end
end