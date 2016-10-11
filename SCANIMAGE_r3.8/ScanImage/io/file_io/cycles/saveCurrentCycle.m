function saveCurrentCycle()
%SAVECURRENTCYCLE Saves the currently defined cycle to a text file.
    global state gh;

    success = false;
    
    globalParamTags = {'cycleName' 'cycleLength' 'numCycleRepeats' 'returnHomeAtCycleEnd' 'restoreOriginalConfig'};
    cycleParamTags = {'configName' 'iterationDelay' 'motorAction' 'motorActionID' 'roiNum' 'power'};
    acqParamTags = {'repeatPeriod' 'numberOfRepeats' 'numberOfZSlices' 'zStepSize' 'numberOfFrames' 'numAvgFramesSave' 'framesPerFile' 'framesPerFileLock'};
	
	if isempty(state.cycle.cyclePath) || ~isdir(state.cycle.cyclePath)
		saveCurrentCycleAs;
		return
	end
	
	% open the file
	try
		[fID, message] = fopen(fullfile(state.cycle.cyclePath,[state.cycle.cycleName '.cyc']), 'wt');
	catch ME
		error('Unable to open file.');
	end

	if fID < 0
	   error('Unable to open file: %s.',message); 
	end

	% write all global params
	for paramName = globalParamTags
		paramName = paramName{:};
		fprintfSmart(fID,paramName,state.cycle.(paramName));
		fprintf(fID,'\n');
	end

	tableDataSize = size(state.cycle.cycleTableStruct);
	for j = 1:tableDataSize(2)
		fprintf(fID,'%d\t',j);

		% write all cycle params
		for paramName = cycleParamTags
			paramName = paramName{:};
			if isfield(state.cycle.cycleTableStruct(j),paramName) && ~isempty(state.cycle.cycleTableStruct(j).(paramName))
				%fprintf(fID,'%s\t%s\t',paramName,state.cycle.cycleTableStruct(j).(paramName));
				fprintfSmart(fID,paramName,state.cycle.cycleTableStruct(j).(paramName));
			end
		end

		% write all acq params
		for paramName = acqParamTags
			paramName = paramName{:};
			if isfield(state.cycle.cycleTableStruct(j),paramName) && ~isempty(state.cycle.cycleTableStruct(j).(paramName))
				%fprintf(fID,'%s\t%s\t',paramName,state.cycle.cycleTableStruct(j).(paramName));   
				fprintfSmart(fID,paramName,state.cycle.cycleTableStruct(j).(paramName));
			end
		end

		% write the config path
		if (j <= length(state.cycle.cycleConfigPaths)) && ~isempty(state.cycle.cycleConfigPaths{j})
			fprintf(fID,'%s\t%s\t','configPath',state.cycle.cycleConfigPaths{j});
		end

		% write any user-specified params
		for paramName = state.cycle.cycleTableColumnsUserAdded
			paramName = paramName{:};
			if ~isempty(state.cycle.cycleTableStruct(j).(strrep(paramName,'.','DOT')))
				%fprintf(fID,'%s\t%s\t',paramName,state.cycle.cycleTableStruct(j).(strrep(paramName,'.','DOT')));
				fprintfSmart(fID,paramName,state.cycle.cycleTableStruct(j).(strrep(paramName,'.','DOT')));
			end
		end

		fprintf(fID,'\n');
	end

	fclose(fID);
	
	updateGUIByGlobal('state.cycle.cycleName');
end

function fprintfSmart(fID,paramName,val)

	if isnumeric(val)
		formatString = '%s\t%d\t';
	elseif ischar(val)
		formatString = '%s\t%s\t';
	elseif islogical(val)
		formatString = '%s\t%s\t';
		if val
			val = 'true';
		else
			val = 'false';
		end
	else
		return;
	end
	
	fprintf(fID,formatString,paramName,val);

%     scalarExp = '^\d+';
%     posArrayExp = '\[\d+\.*\d* \d+\.*\d* \d+\.*\d*\]';
%     stringExp = '.*';
%     
%     if isnumeric(val)
%         fprintf(fID,'%s\t%d',paramName,val);
%     else
%         if regexp(val,scalarExp)
%             formatString = '%s\t%d';
%         elseif regexp(val,posArrayExp)
%             formatString = '%s\t%s';
%         elseif regexp(val,stringExp)
%             formatString = '%s\t%s';
%         else
%             return;
%         end
% 
%         fprintf(fID,formatString,paramName,val);
% 
%     end
end
