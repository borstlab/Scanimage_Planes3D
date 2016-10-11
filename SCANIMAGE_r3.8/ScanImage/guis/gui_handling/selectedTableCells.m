function varargout = selectedTableCells(table,indices)
% A helper function to set/get the uitable's selected cells.
%	cellIndices = selectedCells() returns the currently selected cell indices.
%	selectedCells(cellIndices) sets the currently selected cell indices.
%		NOTE: 'cellIndices' is assumed to be in the format of 'eventdata.Indices'
	global gh;
	persistent selectedIndices;
	
	if ~strcmp(class(selectedIndices),'containers.Map') && isempty(selectedIndices)
		selectedIndices = containers.Map('KeyType','double','ValueType','any');
	end

	if nargin < 1 || isempty(table)
		error('You must specify a table');
	elseif ischar(table)
		% user gave a table name--determine its graphics handle:
		switch table
			case 'userFcns'
				table = gh.userFunctionsGUI.tblUserFcns;
			case 'usrOnlyFcns'
				table = gh.userFunctionsGUI.tblUSROnlyFcns;
			case 'overrideFcns'
				table = gh.userFunctionsGUI.tblOverrideFcns;
			case {'cycle' 'cycleTable'}
				table = gh.cycleGUI.tblCycle;
			case {'position' 'posn'}
				table = gh.positionGUI.tblPosition;
            case 'roi'
				table = gh.roiGUI.tblROI;
		end
	end
	
	if nargin == 1
		if selectedIndices.isKey(table)
			varargout{1} = selectedIndices(table);
		else
			varargout{1} = [];
		end
	elseif nargin == 2
		selectedIndices(table) = indices;
	end

end