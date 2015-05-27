function updateCycleTable(tableData,doSuppressGUIUpdate)
%UPDATECYCLETABLE Updates cycleGUI's uitable data (and reflects these changes in the 'state.cycle.cycleTableStruct' data store).
	global state gh;

    if nargin < 2 || isempty(doSuppressGUIUpdate)
        doSuppressGUIUpdate = false; 
    end
    
	if nargin < 1 || isempty(tableData)
    	% no argument given, so clear/reset the table

 		% reset the GUI uitable
		tableData = repmat(state.cycle.cycleTableColumnDefaults,2,1);
		set(gh.cycleGUI.tblCycle,'ColumnName',state.cycle.cycleTableColumnsPretty');

 		% reset the config paths array and list of user-added columns
 		state.cycle.cycleConfigPaths = cell(1,2);
 		state.cycle.cycleTableColumnsUserAdded = {};
	end
	
    if ~doSuppressGUIUpdate
        % write the data to the uitable
         set(gh.cycleGUI.tblCycle,'Data',tableData);
    end

	% reset the internal data store
	state.cycle.cycleTableStruct = struct('motorAction',{'Pos #','Pos #'});

	% parse 'tableData' and add this data to internal data store
	dims = size(tableData);
    hWaitbar = waitbar(0,'Updating Cycle Table...');
	for j = 1:dims(1);
		row = tableData(j,:);
		for i = 1:length(row)
			columnName = state.cycle.cycleTableColumns(i);
			state.cycle.cycleTableStruct(j).(columnName{1}) = row{i};
            waitbar((j*length(row) + i)/(dims(1)*length(row)),hWaitbar);
		end
    end
    close(hWaitbar);
	
	state.cycle.cycleLength = dims(1);
	
    if ~doSuppressGUIUpdate
        selectedTableCells('cycle',[]);
        updateGUIByGlobal('state.cycle.cycleLength');
    end
end

