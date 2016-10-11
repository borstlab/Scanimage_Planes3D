function varargout = metaStackGUI(varargin)
% METASTACKGUI MATLAB code for metaStackGUI.fig
if nargin == 0  % LAUNCH GUI

	fig = openfig(mfilename,'reuse');

	% Generate a structure of handles to pass to callbacks, and store it. 
	handles = guihandles(fig);
	guidata(fig, handles);

	if nargout > 0
		varargout{1} = fig;
    end
    
    %%%VI120108A%%%%%%%%%%%%%%%%%%%%%%
    set(fig,'KeyPressFcn',@genericKeyPressFunction);
    %Ensure all children respond to key presses, when they have the focus (for whatever reason)
    kidControls = findall(fig,'Type','uicontrol');
    for i=1:length(kidControls)
        if ~strcmpi(get(kidControls(i),'Style'),'edit')
            set(kidControls(i),'KeyPressFcn',@genericKeyPressFunction);
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif ischar(varargin{1}) % INVOKE NAMED SUBFUNCTION OR CALLBACK

	try
		[varargout{1:nargout}] = feval(varargin{:}); % FEVAL switchyard
    catch ME %VI101910A
        most.idioms.reportError(ME);
    end
    
end


function etNumSlices_Callback(hObject, eventdata, handles)
global state gh;

% constrain to only integer values
val = get(hObject,'String');
if isempty(val)
	return;
end
numSlices = str2double(val);
if isempty(regexp(val,'^\d+$'))
	numSlices = round(numSlices);
	set(hObject,'String',num2str(numSlices));
end

% handle stack start/stop constraints
if get(gh.metaStackGUI.cbStartEndConstrain,'Value')
	if ~areStackFieldsConstrained()
		stepSize = (state.motor.stackStop(3) - state.motor.stackStart(3))/numSlices;
		set(gh.metaStackGUI.etStepSize,'String',num2str(stepSize));
	end
end


function etStepSize_Callback(hObject, eventdata, handles)
global state gh;

stepSize = str2double(get(hObject,'String'));
if isempty(stepSize)
	return;
end

% handle stack start/stop constraints
if get(gh.metaStackGUI.cbStartEndConstrain,'Value')
	if ~areStackFieldsConstrained()
		numSlices = ceil((state.motor.stackStop(3) - state.motor.stackStart(3))/stepSize);
		set(gh.metaStackGUI.etNumSlices,'String',num2str(numSlices));
	end
end


function cbStackCentered_Callback(hObject, eventdata, handles)


% --- Executes on button press in cbStartEndConstrain.
function cbStartEndConstrain_Callback(hObject, eventdata, handles)
global state gh;

isEnabled = get(hObject,'Value');

if isEnabled
	% first, ensure that the start/end points have been defined and are valid.
	if isempty(state.motor.stackStart)
		disp('The ''Stack Start'' parameter is not defined.');
		set(gh.metaStackGUI.cbStartEndConstrain,'Value',false);
		return;
	end
	if isempty(state.motor.stackStop)
		disp('The ''Stack Stop'' parameter is not defined.');
		set(gh.metaStackGUI.cbStartEndConstrain,'Value',false);
		return;
	end
	if state.motor.stackStart == state.motor.stackStop
		disp('The ''Stack Start'' and ''Stack Stop'' parameters are invalid.');
		set(gh.metaStackGUI.cbStartEndConstrain,'Value',false);
		return;
	end
	
	if ~areStackFieldsConstrained()
		% update the fields to match
		numSlices = str2double(get(gh.metaStackGUI.etNumSlices,'String'));
		if isnan(numSlices)
			stepSize = str2double(get(gh.metaStackGUI.etStepSize,'String'));
			numSlices = ceil((state.motor.stackStop(3) - state.motor.stackStart(3))/stepSize);
			set(gh.metaStackGUI.etNumSlices,'String',num2str(numSlices));
		else
			stepSize = (state.motor.stackStop(3) - state.motor.stackStart(3))/numSlices;
			set(gh.metaStackGUI.etStepSize,'String',sprintf('%.2f',stepSize));
		end
	end
end


function pbApply_Callback(hObject, eventdata, handles)
global state gh;
numSlices = str2double(get(gh.metaStackGUI.etNumSlices,'String'));
stepSize = str2double(get(gh.metaStackGUI.etStepSize,'String'));
isStackCentered = get(gh.metaStackGUI.cbStackCentered,'Value');
doGoHome = get(gh.metaStackGUI.cbReturnHome,'Value');

if isempty(numSlices) || numSlices < 2 || isempty(stepSize)
	disp('Invalid parameters given.');
	return;
end

% get and replicate the current cycle table data
tableData = get(gh.cycleGUI.tblCycle,'Data');
dims = size(tableData);
initialCycleLength = dims(1);
tableData = repmat(tableData,numSlices,1);

% 
indicesToClear = find(ismember(state.cycle.cycleTableColumns,{'numberOfZSlices' 'zStepSize'}));
[tableData{:,indicesToClear}] = deal(1);

indicesToClear = find(ismember(state.cycle.cycleTableColumns,{'motorActionID'}));
[tableData{:,indicesToClear}] = deal([]);

% set the motor action column
motorActionColumnIndex = find(ismember(state.cycle.cycleTableColumns,{'motorAction'}));
[tableData{:,motorActionColumnIndex}] = deal('Z Step');

motorStepColumnIndex = find(ismember(state.cycle.cycleTableColumns,{'motorActionID'}));
if isStackCentered
	% set the initial move of half the stack size
	startOffset = floor((numSlices*stepSize)/2.0);
	tableData{1,motorStepColumnIndex} = -startOffset;
end

rowIndices = initialCycleLength+1:initialCycleLength:numSlices*initialCycleLength;

[tableData{rowIndices,motorStepColumnIndex}] = deal(stepSize);

% handle returning to the start position
if doGoHome
	tableData(end+1,:) = state.cycle.cycleTableColumnDefaults;
	tableData{end,motorActionColumnIndex} = 'Z Step';
	if isStackCentered
		tableData{end,motorStepColumnIndex} = ceil(-numSlices*stepSize/2);
	else	
		tableData{end,motorStepColumnIndex} = 1 - numSlices*stepSize;
	end
end

updateCycleTable(tableData);

% replicate the existing cycle config paths
state.cycle.cycleConfigPaths = repmat(state.cycle.cycleConfigPaths,1,numSlices);

% close the dialog
hideGUI('gh.metaStackGUI.figure1');

	
function val = areStackFieldsConstrained()
global state gh;

numSlices = str2double(get(gh.metaStackGUI.etNumSlices,'String'));
stepSize = str2double(get(gh.metaStackGUI.etStepSize,'String'));

if isnan(numSlices) || isnan(stepSize)
	if isnan(numSlices) && isnan(stepSize) 
		val = true;
		return;
	else
		val = false;
		return;
	end
end

if state.motor.stackStop - state.motor.stackStart == numSlices*stepSize
	val = true;
else
	val = false;
end
	
