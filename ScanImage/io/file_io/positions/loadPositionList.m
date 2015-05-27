function loadPositionList
	[fname, pname]=uigetfile('*.pos', 'Choose position list file');
	if ~isnumeric(fname)
		global state
		periods=findstr(fname, '.');
		if any(periods)								
			fname=fname(1:periods(1)-1);
		end		
		
		setStatusString('Loading position list...');
		load(fullfile(pname, [fname '.pos']), '-mat');			% load file as MATLAB workspace file
		state.motor.positionVectors=positionVectors;
		if state.motor.position>size(state.motor.positionVectors,1)
			state.motor.position=size(state.motor.positionVectors,1);
			updateGUIByGlobal('state.motor.position');
        end
        
        %%%VI051111A%%%%
        if any(any(isnan(state.motor.positionVectors(:,state.motor.dimensionsAllMask))))
            currPosn = motorGetPosition();
                        
            for i=1:size(state.motor.positionVectors,1)
                neededDimensions = isnan(state.motor.positionVectors(i,:)) & state.motor.dimensionsAllMask(1:3); %With current implementation, position vectors are always 3-vectors
                state.motor.positionVectors(i,neededDimensions) = currPosn(neededDimensions);                
            end            
        end        
        %%%%%%%%%%%%%%%%
        
		setStatusString('Position list loaded...');
		disp('The following position list was loaded:');
		%listPositions;
        motorPositionList();
	end
