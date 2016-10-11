function out=openAndLoadCycle
	out=0;

	global state
	if state.internal.cycleChanged==1
        %%%VI102909A%%%%%
        if ~isempty(state.cycle.cycleName)
            questString = ['Do you want to save changes to ' state.cycle.cycleName '?'];
        else
            questString = 'Do you want to save the currently configured cycle?';
        end
        %%%%%%%%%%%%%%%%%
		button = questdlg(questString,'Save changes?','Yes','No','Cancel','Yes');
		if strcmp(button, 'Cancel')
			disp(['*** LOAD CYCLE CANCELLED ***']);
			return
		elseif strcmp(button, 'Yes')
			disp(['*** SAVING CURRENT CYCLE = ' state.cycle.cyclePath '\' state.cycle.cycleName ' ***']);
			flag=saveCurrentCycle;
			if ~flag
				disp(['loadCycle: Error returned by saveCurrentCycle.  Cycle may not have been saved.']);
				return
			end
		end
    end

    %%%VI110109A: Removed%%%
    % 	if ~isempty(state.cycle.cyclePath)
    % 		cd(state.cycle.cyclePath)
    % 	end
    %%%%%%%%%%%%%%%%%%%%%%%
    
    [fname, pname]=uigetfile('*.cyc', 'Choose cycle file to load', state.cycle.cyclePath); %VI110109A
    if ~isnumeric(fname)
        [~,~,ext] = fileparts(fname);
        if isempty(ext) || ~strcmpi(ext,'.cyc')
            fprintf(2,'WARNING: Invalid file extension provided. Cannot open CYC file.\n');
            setStatusString('Can''t open file...');
            return
        end
        
        loadCycle(fname, pname);
        %cd(state.cycle.cyclePath); %VI110109A
        changePositionToExecute(0);
    end
    
