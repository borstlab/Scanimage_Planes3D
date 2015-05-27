function varargout = cycleGUI(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @cycleGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @cycleGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

% --- Executes just before roiGUI is made visible.
function cycleGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to roiGUI (see VARARGIN)

% Choose default command line output for roiGUI
handles.output = hObject;

%Initialize PropControls

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = cycleGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function tblCycleCellSelect_Callback(h,eventdata,handles)
global gh;

CFG_NAME_COL_IDX = 1; % The index for the 'Config Name' column

persistent robot;
if isempty(robot)
	robot = most.testing.Robot();
end

% store the selected cell indices
selectedTableCells(h,eventdata.Indices);

if ~isempty(eventdata.Indices)
    % determine if any add/remove buttons need to be enabled/disabled
    indices = eventdata.Indices;
    tableData = get(h,'Data');

    setRemoveButtonState('on');
    
    if numel(indices) == 2 % single cell selected
        if indices(2) == CFG_NAME_COL_IDX % cfg name column selected
            if isempty(tableData{indices(1),indices(2)})
                setClearButtonState('off');
            else
                setClearButtonState('on');
            end
            setAddButtonState('on');
        else
            setAddButtonState('off');
            if isempty(tableData{indices(1),indices(2)})
                setClearButtonState('off');
            else
                setClearButtonState('on');
            end
        end
    else % multiple cells selected
        if sum(indices(:,2)./CFG_NAME_COL_IDX - 1) == 0 % a cute way of determining that all selected cells are in the 'Config Name' column
            % if there are any existing entries, enable the 'remove' button, otherwise enable the 'add' button
            emptyResults = cellfun(@isempty,tableData(indices(:,1),CFG_NAME_COL_IDX),'UniformOutput',false);
            if all([emptyResults{:}])
                setAddButtonState('on');
                setClearButtonState('off');
            else
                setAddButtonState('off');
                setClearButtonState('on');
            end
        else


            setAddButtonState('off');
            setClearButtonState('on');
        end
    end
else
    setClearButtonState('off');
    setAddButtonState('off');
    setRemoveButtonState('off');
end


function setAddButtonState(mode)
global gh;
set(gh.cycleGUI.pbAdd,'Enable',mode);

function setClearButtonState(mode)
global gh;
set(gh.cycleGUI.pbClear,'Enable',mode);

function setRemoveButtonState(mode)
global gh;
set(gh.cycleGUI.pbDropRow,'Enable',mode);


% --------------------------------------------------------------------
function tblCycleCellEdit_Callback(h,eventdata,handles,isCalledFromHandler)
global state gh

if nargin < 4 || isempty(isCalledFromHandler)
	isCalledFromHandler = false;
end

indices = eventdata.Indices;
columnName = state.cycle.cycleTableColumns{indices(2)};
tableData = get(gh.cycleGUI.tblCycle,'Data');

% Handle multiple-select edits
if ~isCalledFromHandler
	selectedIndices = selectedTableCells('cycle');
	if numel(selectedIndices) > 2
		multipleSelectEditHandler(eventdata);
		return;
	end
else
	% Add the multiple-select data to the table
	tableData{indices(1),indices(2)} = eventdata.NewData;
	state.cycle.cycleTableStruct(indices(1)).(columnName) = eventdata.NewData;
end

% If the user deletes a numeric cell's contents (via editing), Matlab sets it as NaN for some reason.  This fixes that 'bug'.
if isempty(eventdata.EditData) && (~isempty(isnan(eventdata.NewData)) && isnan(eventdata.NewData))
	eventdata.NewData = [];
	tableData{eventdata.Indices(1),eventdata.Indices(2)} = [];
end

valid = true;
columnNumber = eventdata.Indices(2);
columnTypes = get(gh.cycleGUI.tblCycle,'ColumnFormat');

% ensure valid data types
if ~isempty(eventdata.NewData)
	if strcmp(columnTypes(columnNumber),'numeric')
		if ~isnumeric(eventdata.NewData) || isnan(eventdata.NewData) %length(str2num(eventdata.NewData)) ~= 1 || isnan(eventdata.NewData) || ischar(eventdata.NewData)
			valid = false;
			message = 'Type mismatch.';
		end
	elseif strcmp(columnTypes(columnNumber),'char') && columnNumber ~= 1 % constraint doesn't apply to 'config' column
        if isnumeric(eventdata.NewData) %TMW: This shouldn't happen!!!
            fprintf(2,'WARNING: Matlab converted string to number clairvoyantly.\n');
        elseif isempty(str2num(eventdata.NewData)) || ~isnumeric(str2num(eventdata.NewData))
			valid = false;
			message = 'Type mismatch.';
		end
	end

	% enforce column-specific constraints
	if valid
		switch columnName
			case 'motorAction'
				% reset the 'motorActionID' field
				tableData{eventdata.Indices(1),eventdata.Indices(2)+1} = '';

			case 'motorActionID'
				motorAction = tableData{eventdata.Indices(1),eventdata.Indices(2) - 1};
				switch motorAction
                    case {'Posn #' 'ROI #'}
						actionID = str2double(eventdata.NewData);
						if isnan(actionID) || ~isscalar(actionID)
							valid = false;
							message = 'Please specify a valid motor action #.';
                        else
                            if strcmp(motorAction,'Posn #')
                                if ~state.hSI.roiIsValidPosition(actionID)
                                    valid = false;
                                    message = 'Please specify a valid motor Position #.';
                                end
                            elseif strcmp(motorAction,'ROI #')
                                if ~state.hSI.roiIsValidROI(actionID)
                                    valid = false;
                                    message = 'Please specify a valid ROI #.';
                                end
                            end
                        end
                    case {'Step'}
                        if ~isempty(eventdata.NewData)
							val = parseVectorString(eventdata.NewData,'motor step');
							if isempty(val)
								valid = false;
								message = ['Invalid value.'];
							else
								tableData{eventdata.Indices(1),eventdata.Indices(2)} = val;
								eventdata.NewData = val;
							end
						end
                end
                
			case 'power'
				if ~isempty(eventdata.NewData)
					val = parseVectorString(eventdata.NewData,'power');
					if isempty(val)
						valid = false;
						message = ['Value must be of length ' num2str(state.init.eom.numberOfBeams) '.'];
					else
						tableData{eventdata.Indices(1),eventdata.Indices(2)} = val;
						eventdata.NewData = val;
					end
				end
		end
	end

	if valid
		if get(gh.cycleGUI.cbApplyToAll,'Value')
			% update the uipanel
			tableData = get(gh.cycleGUI.tblCycle,'Data');
			[tableData{:,eventdata.Indices(2)}] = deal(eventdata.NewData);

			% update the internal data structure
			dims = size(tableData);
			[state.cycle.cycleTableStruct(1:dims(1)).(columnName)] = deal(eventdata.NewData);
		else
			state.cycle.cycleTableStruct(indices(1)).(columnName) = eventdata.NewData;
		end
    else        
		state.hSI.fullError('Invalid cycle data.',['Invalid Cycle data entry: ' message]);
		tableData{eventdata.Indices(1),eventdata.Indices(2)} = state.cycle.cycleTableColumnDefaults{eventdata.Indices(2)};
	end
end

set(gh.cycleGUI.tblCycle,'Data',tableData);


function outputString = parseVectorString(val,varName)
global state;

if regexp(val,'^\[.*\]$')
	tokens = regexpi(val,'([\-0-9\.]+|NaN)*','tokens');
	if ~isempty(tokens)
		val = [];
		for i=1:length(tokens)
			token = str2double(tokens{i});
			if isnan(token) && ~strcmp(varName,'power') % the 'power' column is the only one that should allow NaNs
				token = 0;
			end
			val = [val token];
		end

		% deal with too-short vector lengths
		if strcmp(varName,'motor step')
			padVal = 0;
            if state.motor.dimensionsXYZZ
                padLength = 4;
                if state.motor.motorZEnable
                    nonPadIdx = 4;
                else
                    nonPadIdx = 3;
                end
            else
                padLength = 3;
                nonPadIdx = 3;
            end
		elseif strcmp(varName,'power')
			padVal = NaN;
			padLength = state.init.eom.numberOfBeams;
            nonPadIdx = 1;
		end

		if length(val) < padLength
            if length(val) ~= length(nonPadIdx)
                %state.hSI.consoleError('Invalid input string.');
                outputString = '';
                return;
            end
            nonPadVal = val;
            val = repmat(padVal,1,padLength);
            val(nonPadIdx) = nonPadVal;
        elseif length(val) > padLength
			outputString = '';
			%state.hSI.consoleError('Invalid input string.');
			return;
		end
		outputString = ['[' num2str(val) ']'];
		return;
	end
elseif ~isempty(str2num(val))
	% user entered a scalar value--wrap it in brackets, and re-call (to force padding to the appropriate vector length).
	outputString = parseVectorString(['[' val ']'],varName);
	return;
end
outputString = '';
%state.hSI.consoleError('Invalid input string.');


function multipleSelectEditHandler(eventdata)
global gh;

indices = selectedTableCells('cycle');

for row = indices'
	eventdata.Indices = row';
	tblCycleCellEdit_Callback(gh.cycleGUI.tblCycle,eventdata,[],true);
end
	
function tblCycle_KeyPressFcn(h, eventdata, handles)

% handle a 'delete' or 'backspace' keystroke
cellIndices = selectedTableCells(h);

if strcmp(eventdata.Key,'delete')
	zlclClearCells(cellIndices);
end

% --------------------------------------------------------------------
function pbLoad_Callback(hObject, eventdata, handles)
loadCycle();

% --------------------------------------------------------------------
function pbSave_Callback(hObject, eventdata, handles)
saveCurrentCycleAs();

% --------------------------------------------------------------------
function addConfigFile(h,eventdata,handles,fileName)
	global state gh

    [~,filenameNoExtension,~] = fileparts(fileName);
    updateTableCell(filenameNoExtension,eventdata.Indices);
    
    % doctor 'eventdata' and manually invoke the cell edit callback to store the config name to the cycle table struct
	eventdata.EditData = filenameNoExtension;
    eventdata.NewData = filenameNoExtension;
    tblCycleCellEdit_Callback(gh.cycleGUI.tblCycle,eventdata,handles);
    
    % store the full config path
	[pname,fname,ext] = fileparts(fileName);
	if get(gh.cycleGUI.cbApplyToAll,'Value')
		[state.cycle.cycleConfigPaths{:}] = deal(pname);
	else
		state.cycle.cycleConfigPaths{eventdata.Indices(1)} = pname;
	end
	
 	% update the GUI (adding a file deselects all cells)
	selectedTableCells('cycle',[]);
	setAddButtonState('off');
	
	% cache the config file to memory
	try
		cacheConfiguration(fileName);
	catch ME
		error(['An error occurred while caching the config file: ' ME.message]);
	end

% --------------------------------------------------------------------
function updateTableCell(val,indices)
global state gh

if nargin < 2 || isempty(indices)
	indices = selectedTableCells('cycle');
	if numel(indices) ~= 2
		error('Multiple cells are selected; please call updateTableCell() specifying ''indices''');
	end
end

tableData = get(gh.cycleGUI.tblCycle,'Data');

% if get(gh.cycleGUI.cbApplyToAll,'Value')
%     [tableData{:,eventdata.Indices(2)}] = deal(val);
% else
    tableData{indices(1),indices(2)} = val;
% end
	
updateCycleTable(tableData);    

% --------------------------------------------------------------------
function etCycleName_Callback(hObject, eventdata, handles)
genericCallback(hObject);

% --- Executes on button press in cbCycleOn.
function cbCycleOn_Callback(h,eventdata, handles)
genericCallback(h);

% --- Executes on button press in cbGoHomeAtCycleEnd.
function cbGoHomeAtCycleEnd_Callback(hObject, eventdata, handles)
genericCallback(hObject);


function etNumCycleRepeats_Callback(hObject, eventdata, handles)
genericCallback(hObject);

% --- Executes on button press in cbRestoreOriginalCFG.
function cbRestoreOriginalCFG_Callback(hObject, eventdata, handles)
genericCallback(hObject);


% --- Executes on button press in pbAddRow.
function pbAddRow_Callback(hObject, eventdata, handles)
% global state gh;
% tableData = get(gh.cycleGUI.tblCycle,'Data');
% tableData(end+1,:) = state.cycle.cycleTableColumnDefaults;
% updateCycleTable(tableData);
% 
% state.cycle.cycleConfigPaths = [state.cycle.cycleConfigPaths {[]}];
handles.hModel.cycAddIteration();

% --- Executes on button press in pbDropRow.
function pbDropRow_Callback(hObject, eventdata, handles)
global state gh;

% determine the selected row(s)
indices = selectedTableCells('cycle');
rows = unique(indices(:,1));
for i = 1:size(rows)
    offset = length(rows(rows < rows(i))); % account for any already-removed rows
    state.hSI.cycRemoveIteration(rows(i) - offset);
end

% --- Executes on button press in pbClearTable.
function pbClearTable_Callback(hObject, eventdata, handles)
global state gh;

choice = questdlg('This will clear all table contents--would you like to proceed?', ...
	'Clear All', ...
	'Yes','No','No');

switch choice
    case 'Yes'
		updateCycleTable();
	case 'No'
		return;
end


function etCycleIteration_Callback(hObject, eventdata, handles)
global state;
cycleIterationVal = str2double(get(hObject,'String'));
if cycleIterationVal > state.cycle.cycleLength || cycleIterationVal < 1
	 cycleIterationVal = mod(cycleIterationVal-1,state.cycle.cycleLength) + 1;
end
state.cycle.iteration = cycleIterationVal;
updateGUIByGlobal('state.cycle.iteration');



% --- Executes on button press in pbCycleIterationDec.
function pbCycleIterationDec_Callback(hObject, eventdata, handles)
updateCycleIteration(-1);
% --- Executes on button press in pbCycleIterationInc.
function pbCycleIterationInc_Callback(hObject, eventdata, handles)
updateCycleIteration(1);

function updateCycleIteration(delta)
global state;
iteration = state.cycle.iteration + delta;
iteration = mod(iteration-1,state.cycle.cycleLength) + 1;
state.cycle.iteration = iteration;
updateGUIByGlobal('state.cycle.iteration');

function etCycleCount_Callback(hObject, eventdata, handles)
global state;
cyclesDoneVal = checkCycleCount(str2double(get(hObject,'String')));
state.cycle.cycleCount = cyclesDoneVal;
updateGUIByGlobal('state.cycle.cycleCount');

% --- Executes on button press in pbCycleCountDec.
function pbCycleCountDec_Callback(hObject, eventdata, handles)
updateCycleCount(-1);

% --- Executes on button press in pbCycleCountInc.
function pbCycleCountInc_Callback(hObject, eventdata, handles)
updateCycleCount(1);

function val = checkCycleCount(cyclesDoneVal)
global state;
if cyclesDoneVal >= state.cycle.numCycleRepeats
	val = state.cycle.numCycleRepeats - 1;
elseif cyclesDoneVal < 0
	val = 0;
else
	val = cyclesDoneVal;
end

function updateCycleCount(delta)
global state;
count = checkCycleCount(state.cycle.cycleCount + delta);
state.cycle.cycleCount = count;
updateGUIByGlobal('state.cycle.cycleCount');

% --- Executes on button press in tbShowAdvanced.
function tbShowAdvanced_Callback(hObject, eventdata, handles)
if ~toggleAdvancedPanel(hObject,113,'x');
	hideGUI('gh.metaStackGUI.figure1');
end

% --- Executes on button press in pbCycleReset.
function pbCycleReset_Callback(hObject, eventdata, handles)
global state;
state.cycle.iteration = 1;
updateGUIByGlobal('state.cycle.iteration');
state.cycle.cycleCount = 0;
updateGUIByGlobal('state.cycle.cycleCount');

% --- Executes on button press in cbCycleAutoReset.
function cbCycleAutoReset_Callback(hObject, eventdata, handles)
genericCallback(hObject);
global gh;
toggleableComponents = {'pbCycleCountInc' 'pbCycleCountDec' 'pbCycleIterationInc' 'pbCycleIterationDec' 'pbCycleReset' 'stIterationsPerLoop' 'etIterationsPerLoop'};

if get(hObject,'Value')
	state = 'Off';
else
	state = 'On';
end

for i = 1:length(toggleableComponents)
	set(gh.cycleGUI.(toggleableComponents{i}),'Enable',state);
end


% --- Executes on button press in pbMetaStack.
function pbMetaStack_Callback(hObject, eventdata, handles)
global gh;

if get(hObject,'Value')
	tetherGUIs('cycleGUI','metaStackGUI','righttop');
	seeGUI('gh.metaStackGUI.figure1');
else 
	hideGUI('gh.metaStackGUI.figure1');
end

% --- Executes on button press in pbMosaic.
function pbMosaic_Callback(hObject, eventdata, handles)
global state;
state.hSI.roiMacroMosaic();

function zlclClearCells(indices)
% Deletes the existing contents of a cell(s) and sets it to its default value.
global state gh;

tableData = get(gh.cycleGUI.tblCycle,'Data');

for row = indices'
	% the 'config' column requires some additional work:
	if row(2) == 1
		removeCachedConfiguration(fullfile(state.cycle.cycleConfigPaths{row(1)},[tableData{row(1),row(2)} '.cfg']));
		state.cycle.cycleConfigPaths{row(1)} = '';
	end
	
	tableData{row(1),row(2)} = state.cycle.cycleTableColumnDefaults{row(2)};
	
	set(gh.cycleGUI.pbAdd,'Enable','off');
	set(gh.cycleGUI.pbClear,'Enable','off');
end

set(gh.cycleGUI.pbClear,'Enable','off');
updateCycleTable(tableData);


% --- Executes on button press in pbAdd.
function pbAdd_Callback(~,~,handles)
	global state gh
	
	indices = selectedTableCells(handles.tblCycle);
	tableData = get(handles.tblCycle,'Data');
	
	try
		%Prompt user to select file
		startPath = state.hSI.getLastPath('cycleCFGLastPath');
		[fname, pname]=uigetfile({'*.cfg'},'Choose CFG File...',startPath);
		if isnumeric(fname)
			return
		else
			[~,filenameNoExtension,~] = fileparts(fname);

			if ~strcmp(pname,startPath)
				state.hSI.setLastPath('cycleCFGLastPath',pname);
			end
		end
	catch ME
		ME.throwAsCaller();
	end

	for row = indices'
		row = row';
		eventdata = struct();
		eventdata.Indices = row;
		
		if ~isempty(tableData{row(1),row(2)})
			zlclClearCells(row);
		end
		
		addConfigFile([],eventdata,handles,fullfile(pname,fname));
	end


% --- Executes on button press in pbClear.
function pbClear_Callback(hObject, eventdata, handles)
indices = selectedTableCells('cycle');
zlclClearCells(indices);


function etIterationsPerLoop_Callback(hObject, eventdata, handles)
genericCallback(hObject);
