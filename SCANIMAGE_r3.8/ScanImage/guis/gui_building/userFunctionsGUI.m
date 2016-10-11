function varargout = userFunctionsGUI(varargin)
	% USERFUNCTIONSGUI MATLAB code for userFunctionsGUI.fig
	%      USERFUNCTIONSGUI, by itself, creates a new USERFUNCTIONSGUI or raises the existing
	%      singleton*.
	%
	%      H = USERFUNCTIONSGUI returns the handle to a new USERFUNCTIONSGUI or the handle to
	%      the existing singleton*.
	%
	%      USERFUNCTIONSGUI('CALLBACK',hObject,eventData,handles,...) calls the local
	%      function named CALLBACK in USERFUNCTIONSGUI.M with the given input arguments.
	%
	%      USERFUNCTIONSGUI('Property','Value',...) creates a new USERFUNCTIONSGUI or raises the
	%      existing singleton*.  Starting from the left, property value pairs are
	%      applied to the GUI before userFunctionsGUI_OpeningFcn gets called.  An
	%      unrecognized property name or invalid value makes property application
	%      stop.  All inputs are passed to userFunctionsGUI_OpeningFcn via varargin.
	%
	%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
	%      instance to run (singleton)".
	%
	% See also: GUIDE, GUIDATA, GUIHANDLES

	% Edit the above text to modify the response to help userFunctionsGUI

	% Last Modified by GUIDE v2.5 30-Aug-2011 20:05:22

	% Begin initialization code - DO NOT EDIT
	gui_Singleton = 1;
	gui_State = struct('gui_Name',       mfilename, ...
		'gui_Singleton',  gui_Singleton, ...
		'gui_OpeningFcn', @userFunctionsGUI_OpeningFcn, ...
		'gui_OutputFcn',  @userFunctionsGUI_OutputFcn, ...
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
	% End initialization code - DO NOT EDIT


	end

	% --- Executes just before userFunctionsGUI is made visible.
	function userFunctionsGUI_OpeningFcn(hObject, eventdata, handles, varargin)
	% This function has no output args, see OutputFcn.
	% hObject    handle to figure
	% eventdata  reserved - to be defined in a future version of MATLAB
	% handles    structure with handles and user data (see GUIDATA)
	% varargin   command line arguments to userFunctionsGUI (see VARARGIN)

	% Choose default command line output for userFunctionsGUI
	handles.output = hObject;

	%File shared variables
	handles.selEvntIdx = [];
	handles.selEvnt = '';

	% Update handles structure
	guidata(hObject, handles);

	%Populate the list of Events from the EventManager class
	global state

	usrOnlyEvents = {'appOpen' 'appClose'}; %List of events to appear in the 'USR-only' table

	mc = metaclass(state.hSI);
	me = mc.Events;
	eventNames = cellfun(@(x)x.Name,me,'UniformOutput',false);
	eventLocal = cellfun(@(x)strcmp(x.DefiningClass.Name,class(state.hSI)),me);
	eventNames(~eventLocal) = [];
	eventNames = setdiff(eventNames,usrOnlyEvents);
	
	% remove 'dummyEvent' from eventNames
	eventNames = setxor(eventNames,'dummyEvent');

	zlclInitUsrFcnsTable(handles.tblUserFcns,eventNames);
	zlclInitUsrFcnsTable(handles.tblUSROnlyFcns,usrOnlyEvents);

	%Initialize the hEventMap and hEventMapUSRONLY
	state.userFcns.maxNumUserFcns = 10;
	state.userFcns.hEventMapUSR = containers.Map();
	state.userFcns.hEventMapCFG = containers.Map();
	state.userFcns.hEventMapUSRONLY = containers.Map();

	structInit = struct(...
		'userFcnName','',...        %Full name of user function, including path
		'userFcnListener',[],...    %Handle to listener object
		'userFcnOptArgs','',...     %String representation of cell array containing arguments passed in from the UserFunctions GUI
		'userFcnKernel',[]);		%Function handle used to construct listener 

	structInitUSRONLY = structInit;
	structInit = repmat(structInit,state.userFcns.maxNumUserFcns,1);

	for i=1:length(eventNames)               
		state.userFcns.hEventMapUSR(eventNames{i}) = structInit;
		state.userFcns.hEventMapCFG(eventNames{i}) = structInit;
	end

	for i=1:length(usrOnlyEvents)
		state.userFcns.hEventMapUSRONLY(usrOnlyEvents{i}) = structInitUSRONLY;
	end

	% Populate the list of Overridable Functions from the OverrideManager class
	overridableFcns = state.hSI.getOverridableFcns();
	initData = cell(length(overridableFcns),3);
	initData(:,1) = overridableFcns;
	initData(:,2:end) = repmat({'' false},length(overridableFcns),1);
	set(handles.tblOverrideFcns,'Data',initData);

	%Initialize the hOverrideMap
	state.userFcns.hOverrideMapUSR = containers.Map();
	state.userFcns.hOverrideMapCFG = containers.Map();

	structInit = struct(...
		'userFcnName','',...    %Full name of override function, including path
		'userFcnKernel',[]);		%Function handle to overriding function        

	structInit = repmat(structInit,1,1);

	for i=1:length(overridableFcns)               
		state.userFcns.hOverrideMapUSR(overridableFcns{i}) = structInit;       
		state.userFcns.hOverrideMapCFG(overridableFcns{i}) = structInit;       
	end
	
	% initialize the USR override state cache
	initCell = cell(1,length(overridableFcns));
	[initCell{:}] = deal(false);
	state.userFcns.overrideStateUSR = containers.Map(overridableFcns,initCell);
	state.userFcns.overrideStateCacheUSR = containers.Map(overridableFcns,initCell);
	state.userFcns.overrideStateCFG = containers.Map(overridableFcns,initCell);
end


% UIWAIT makes userFunctionsGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function zlclInitUsrFcnsTable(hTbl,eventNames)

initData = cell(length(eventNames),4);
initData(:,1) = eventNames;
initData(:,2:end) = repmat({'' '' false},length(eventNames),1);

set(hTbl,'Data',initData);
end


% --- Outputs from this function are returned to the command line.
function varargout = userFunctionsGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

end

function pbSave_Callback(hObject, eventdata, handles)
	global state;
	
	if strcmp(state.userFcns.saveTarget,'CFG')
		saveCurrentConfig();
	elseif strcmp(state.userFcns.saveTarget,'USR')
		saveCurrentUserSettings();
	end
end


% --------------------------------------------------------------------
function pbDec_Callback(hObject, eventdata, handles)
global state
state.userFcns.currentUserFcnIdx = state.userFcns.currentUserFcnIdx - 1;
updateGUIByGlobal('state.userFcns.currentUserFcnIdx','Callback',1);

updateUserFunctionsGUI('UserFcns');
end


% --------------------------------------------------------------------
function pbInc_Callback(hObject, eventdata, handles)
global state
state.userFcns.currentUserFcnIdx = state.userFcns.currentUserFcnIdx + 1;
updateGUIByGlobal('state.userFcns.currentUserFcnIdx','Callback',1);

updateUserFunctionsGUI('UserFcns');
end


% --------------------------------------------------------------------
function cbAffectAll_Callback(hObject, eventdata, handles)

global state

hEvntMap = state.userFcns.(['hEventMap' state.userFcns.saveTarget]);
if strcmp(state.userFcns.saveTarget,'CFG')
	userFcnIdx = state.userFcns.currentUserFcnIdx;
else
	userFcnIdx = 1;
end

enableAll = logical(get(hObject,'Value'));

%Directly update Enabled property of pertinent listeners
eventNames = hEvntMap.keys();
for i=1:length(eventNames)
    evntStruct = hEvntMap(eventNames{i});
    
    if ~isempty(evntStruct(userFcnIdx).userFcnListener)         
        evntStruct(userFcnIdx).userFcnListener.Enabled = enableAll;                
    end
end

%Update GUI (table) display & state vars
updateUserFunctionsGUI('UserFcns'); 
updateUserFunctionState('storeStateVars','UserFcns',[],userFcnIdx);
end

% --------------------------------------------------------------------
function pbAdd_Callback(hObject, eventdata, handles)

global state gh

if ~isempty(handles.selEvnt)       
    
    hEvntMap = state.userFcns.hEventMap;
    evntStruct = hEvntMap(handles.selEvnt);
    userFcnIdx = state.userFcns.currentUserFcnIdx;
    
    %Determine start path for selection 
    userFcnFullName = evntStruct(userFcnIdx).userFcnName;
    if ~isempty(userFcnFullName)
        userFcnPath = fileparts(userFcnFullName);
        if ~isempty(userFcnPath) && exist(userFcnPath,'dir') > 0
            startPath = userFcnPath;
        else
            startPath = getDefaultStartPath();
        end
    else
        startPath = getDefaultStartPath();
    end        
    
    if ~strcmp(startPath(end),filesep)
        startPath(end+1) = filesep;
    end
    
    %Prompt user to select file
    [fname, pname]=uigetfile({'*.m';'*.mexw32'},'Choose User Function...',startPath);
    if isnumeric(fname)
        return
    end
    
    %Add the user function!
    addUserFcn(fullfile(pname,fname),handles);    

end

    function startPath = getDefaultStartPath()        
        if ~isempty(state.userFcns.lastUserFcnPath) && exist(state.userFcns.lastUserFcnPath,'dir') > 0
            startPath = state.userFcns.lastUserFcnPath;
        else
            startPath = most.idioms.startPath(); %VI111110A
        end        
    end

end

% --------------------------------------------------------------------
function pbRemoveOverride_Callback(hObject, eventdata, handles)
removeOverrideFcn(handles);
end

% --------------------------------------------------------------------
function etUsrFcnIdx_Callback(hObject, eventdata, handles)
genericCallback(hObject);
end

function tbShowAdvanced_Callback(hObject,~,~)
toggleAdvancedPanel(hObject,15,'y');
end

function tbl_CellSelectionCallback(h, eventdata, handles)
global state gh;

	FCN_NAME_COL_IDX = 2; % The index for the 'UserFcnName' column

	if ~isempty(eventdata.Indices)
		% store the selected cell indices
		selectedTableCells(h,eventdata.Indices);
		
		% determine if any add/remove buttons need to be enabled/disabled
		indices = eventdata.Indices;
		tableData = get(h,'Data');
		switch h
			case gh.userFunctionsGUI.tblUserFcns
				tableName = 'userFcns';
			case gh.userFunctionsGUI.tblUSROnlyFcns
				tableName = 'usrOnlyFcns';
			case gh.userFunctionsGUI.tblOverrideFcns
				tableName = 'overrideFcns';
		end
		if numel(indices) == 2 % single cell selected
			if indices(2) == FCN_NAME_COL_IDX % fcn name column selected
				if isempty(tableData{indices(1),indices(2)})
					setAddButtonState(tableName,'on');
					setRemoveButtonState(tableName,'off');
				else
					setAddButtonState(tableName,'off');
					setRemoveButtonState(tableName,'on');
				end
			else
				setAddButtonState(tableName,'off');
				setRemoveButtonState(tableName,'off');
			end
		else % multiple cells selected
			if sum(indices(:,2)./FCN_NAME_COL_IDX - 1) == 0 % a cute way of determining that all selected cells are in the fcnName column
				% if there are any existing entries, enable the 'remove' button, otherwise enable the 'add' button
				emptyResults = cellfun(@isempty,tableData(indices(:,1),FCN_NAME_COL_IDX),'UniformOutput',false);
				if all([emptyResults{:}])
					setAddButtonState(tableName,'on');
					setRemoveButtonState(tableName,'off');
				else
					setAddButtonState(tableName,'off');
					setRemoveButtonState(tableName,'on');
				end
			else
				setAddButtonState(tableName,'off');
				setRemoveButtonState(tableName,'off');
			end
		end
	end

end

function setAddButtonState(tableName,mode)
	global gh;
	set(gh.userFunctionsGUI.(['pbAdd' upper(tableName(1)) tableName(2:end)]),'Enable',mode);
end

function setRemoveButtonState(tableName,mode)
	global gh;
	set(gh.userFunctionsGUI.(['pbRemove' upper(tableName(1)) tableName(2:end)]),'Enable',mode);
end


% --------------------------------------------------------------------
function tbl_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to tblUserFcns (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

global state gh;

rowIdx = eventdata.Indices(1);
colIdx = eventdata.Indices(2);

isUSROnly = false;

switch hObject
	case gh.userFunctionsGUI.tblUserFcns
		tableName = 'UserFcns';
		hEvntMap = state.userFcns.(['hEventMap' state.userFcns.saveTarget]);
		index = state.userFcns.currentUserFcnIdx;
		
	case gh.userFunctionsGUI.tblUSROnlyFcns
		tableName = 'USROnlyFcns';
		hEvntMap = state.userFcns.hEventMapUSRONLY;
		index = 1;
		isUSROnly = true;
		
	case gh.userFunctionsGUI.tblOverrideFcns
		tableName = 'OverrideFcns';
		hEvntMap = state.userFcns.(['hOverrideMap' state.userFcns.saveTarget]);
		index = 1;
end

tableData = get(hObject,'Data');

selEvnt = tableData{rowIdx,1}; 
evntStruct = hEvntMap(selEvnt);

columnNames = get(hObject,'ColumnName');
columnName = columnNames{colIdx};

switch columnName %the column specifier
    case 'UserFcn Name' %UserFcn Name column
        newValString = strtrim(eventdata.NewData);
        
        if isempty(newValString) && ~isempty(eventdata.PreviousData) %signals to clear the callback
            if isUSROnly
				removeUSROnlyFcn(handles);
			else
				removeUserFcn(handles);
			end
        else           
            
            s = which(newValString);
            
            if isempty(s) %Function not found anywhere on path
                fixTableValue();
                msgbox(sprintf('The specified name (''%s'') was not found on the Matlab search path.\nYou should either:\n  1) Correct the entry\n  2) Use Add... to browse for & select an M/MEX function (which need not be on the search path)',newValString),'Specified Function Not Found','warn');
                return; %No changes
            elseif exist(s,'file') %The function atop path is not a built-in --> store full path
                addUserFcn(s,handles);
            else %The function atop path is a built-in --> store just the function name
                addUserFcn(newValString,handles);
            end
        end
        
    case 'Arguments' %Arguments column (string cell array)
        
        try
            newValString = strtrim(eventdata.NewData);
            if isempty(newValString)
                newValString = '';
            else
                newVal = eval(newValString);
                
                if ~iscell(newVal)
                    newValString = sprintf('{%s}',newValString);
                else
                    assert(isvector(newVal));
                end
            end
            
            fixTableValue(newValString);
            
        catch ME %Unable to evaluate..or not a vectorial cell array
            msgbox('The (optional) arguments entry should be either a single variable, or a vectorial cell array.','Invalid Entry', 'warn');
            fixTableValue();
            return;
        end
        
        evntStruct(index).userFcnOptArgs = newValString;
        hEvntMap(selEvnt) = evntStruct;
        
        %Update Listener object
        updateUserFunctionState('updateListener',tableName,selEvnt,index);
        updateUserFunctionState('storeStateVars',tableName,[],index); %Force iteration through all events -- no support for per-event store at moment
        
    case 'Enable' %Enable column (logical)
		
		if strcmp(tableName,'OverrideFcns')
			if isempty(evntStruct(index).userFcnKernel)
				fixTableValue(false);
			else
				if eventdata.NewData == 1 && eventdata.PreviousData == 0 % enabling
					if strcmp(state.userFcns.saveTarget,'CFG')
						% if there is an existing (enabled) USR override, unregister it and cache its state.
						if state.userFcns.overrideStateUSR(selEvnt)
							state.hSI.unregisterOverrideFcn(selEvnt);
							state.userFcns.overrideStateCacheUSR(selEvnt) = true;
							state.userFcns.overrideStateUSR(selEvnt) = false;
						end
						state.userFcns.overrideStateCFG(selEvnt) = true;
					elseif strcmp(state.userFcns.saveTarget,'USR')
						% if there is an existing CFG override, don't register the USR override, just cache its state
						state.userFcns.overrideStateCacheUSR(selEvnt) = true;
						if state.userFcns.overrideStateCFG(selEvnt)
							state.userFcns.overrideStateUSR(selEvnt) = false;
							return;
						else
							state.userFcns.overrideStateUSR(selEvnt) = true;
						end
					end
					state.hSI.registerOverrideFcn(selEvnt,evntStruct(index).userFcnKernel);
				elseif eventdata.NewData == 0 && eventdata.PreviousData == 1 % disabling
					if strcmp(state.userFcns.saveTarget,'CFG')
						state.hSI.unregisterOverrideFcn(selEvnt);
						state.userFcns.overrideStateCFG(selEvnt) = false;
						
						% if there is an existing USR override, enable it
						if state.userFcns.overrideStateCacheUSR(selEvnt)
							hOverrideMapUSR = state.userFcns.hOverrideMapUSR;
							usrStruct = hOverrideMapUSR(selEvnt);
							state.hSI.registerOverrideFcn(selEvnt,usrStruct(index).userFcnKernel);
							state.userFcns.overrideStateUSR(selEvnt) = true;
							state.userFcns.overrideStateCacheUSR(selEvnt) = true;
						end
					elseif strcmp(state.userFcns.saveTarget,'USR')
						if state.userFcns.overrideStateUSR(selEvnt)
							state.hSI.unregisterOverrideFcn(selEvnt);
						end
						state.userFcns.overrideStateUSR(selEvnt) = false;
						state.userFcns.overrideStateCacheUSR(selEvnt) = false;
					end
				end
			end
		else
			if isempty(evntStruct(index).userFcnListener)
				%Force value false if no listener is defined
				fixTableValue(false);
			else
				%Update listener Enabled property
				evntStruct(index).userFcnListener.Enabled = eventdata.NewData;
			end
		end
        
        updateUserFunctionState('storeStateVars',tableName,[],index); %Force iteration through all events -- no support for per-event store at moment
		
    otherwise
        assert(false);
end
        
    function fixTableValue(fixedVal)
        if nargin
            tableData{rowIdx, colIdx} = fixedVal;
        else %Use previous value             
            tableData{rowIdx, colIdx} = eventdata.PreviousData;
        end
        set(hObject,'Data',tableData);			
    end


end


% --------------------------------------------------------------------
 function tbl_KeyPressCallback(hObject, eventdata, handles)	
	% handle a 'delete' or 'backspace' keystroke
	cellIndices = selectedTableCells(hObject);

	if cellIndices(2) == 2
		if (strcmp(eventdata.Key,'delete') || strcmp(eventdata.Key,'backspace'))
			zlclClearCells(hObject, cellIndices);
		end
	end
 end
  

function pbAddOverride_Callback(hObject, eventdata, handles)
global state gh

if ~isempty(handles.selEvnt)       
    
%     hEvntMap = state.userFcns.hEventMap;
%     evntStruct = hEvntMap(handles.selEvnt);
%     userFcnIdx = state.userFcns.currentUserFcnIdx;
%     
%     %Determine start path for selection 
%     userFcnFullName = evntStruct(userFcnIdx).userFcnName;
%     if ~isempty(userFcnFullName)
%         userFcnPath = fileparts(userFcnFullName);
%         if ~isempty(userFcnPath) && exist(userFcnPath,'dir') > 0
%             startPath = userFcnPath;
%         else
%             startPath = getDefaultStartPath();
%         end
%     else
        startPath = getDefaultStartPath();
%     end        
    
    if ~strcmp(startPath(end),filesep)
        startPath(end+1) = filesep;
    end
    
    %Prompt user to select file
    [fname, pname]=uigetfile({'*.m';'*.mexw32'},'Choose User Function...',startPath);
    if isnumeric(fname)
        return
    end
    
    %Add the user function!
    addOverrideFcn(fullfile(pname,fname),handles);    

end

    function startPath = getDefaultStartPath()        
        if ~isempty(state.userFcns.lastUserFcnPath) && exist(state.userFcns.lastUserFcnPath,'dir') > 0
            startPath = state.userFcns.lastUserFcnPath;
        else
            startPath = most.idioms.startPath(); %VI111110A
        end        
    end

end


% --- Executes on button press in tbCFG.
function tbCFG_Callback(h, eventdata, handles)
	toggleSaveTargetHelper('CFG');
end


% --- Executes on button press in tbUSR.
function tbUSR_Callback(h, eventdata, handles)
	toggleSaveTargetHelper('USR');
end


function toggleSaveTargetHelper(activeButton)
	global state gh;

	if strcmp(activeButton,'CFG')
		inactiveButton = 'USR';
	else
		inactiveButton = 'CFG';
	end

	hActiveButton = gh.userFunctionsGUI.(['tb' activeButton]);
	hInactiveButton = gh.userFunctionsGUI.(['tb' inactiveButton]);
	
	activeVal = get(hActiveButton,'Value');
	inactiveVal = get(hInactiveButton,'Value');
	
	% handle mutually exclusive toggle behavior
	if activeVal
		if inactiveVal
			set(hInactiveButton,'Value',false);
		end
		state.userFcns.saveTarget = activeButton;
		
		% take care of enabling/disabling dependent GUI components
		toggleUserFunctionsSaveTarget([],[],[],false);
	else
		if ~inactiveVal
			set(hActiveButton,'Value',true);
		end
	end
	
end


%% HELPER FUNCTIONS

function addUserFcn(fileName,handles)
global state gh;

hEvntMap = state.userFcns.(['hEventMap' state.userFcns.saveTarget]);
evntStruct = hEvntMap(handles.selEvnt);
userFcnIdx = state.userFcns.currentUserFcnIdx;

[pname,fnameNoExt] = fileparts(fileName);

%Construct function 'kernel'
if isempty(pname) %Built-in
    userFcnKernel = str2func(fnameNoExt);
else   
    prevPath = addpath(pname,'-begin','-frozen');
    userFcnKernel = str2func(fnameNoExt);
    path(prevPath);
end

%Update event structure record,
evntStruct(userFcnIdx).userFcnName = fileName;
evntStruct(userFcnIdx).userFcnKernel = userFcnKernel;

%Update Event Map
hEvntMap(handles.selEvnt) = evntStruct;

%Update listener & state vars
updateUserFunctionState('updateListener','UserFcns',handles.selEvnt,userFcnIdx);
updateUserFunctionState('storeStateVars','UserFcns',[],userFcnIdx);

%Update GUI (table data)
updateUserFunctionsGUI('UserFcns',handles.selEvnt);
set(gh.userFunctionsGUI.pbAddUserFcns,'Enable','off');

selectedTableCells('userFcns',[]);
end

function removeUserFcn(handles)

global state gh;

assert(~isempty(handles.selEvnt));

hEventMap = state.userFcns.(['hEventMap' state.userFcns.saveTarget]);
evntStruct = hEventMap(handles.selEvnt);
userFcnIdx = state.userFcns.currentUserFcnIdx;

evntStruct(userFcnIdx).userFcnName = '';
evntStruct(userFcnIdx).userFcnKernel = [];

%Update EventMap
hEventMap(handles.selEvnt) = evntStruct;

%Update listener & state vars
updateUserFunctionState('updateListener','UserFcns',handles.selEvnt,userFcnIdx);
updateUserFunctionState('storeStateVars','UserFcns',[],userFcnIdx);

%Update GUI (table)
updateUserFunctionsGUI('UserFcns',handles.selEvnt);
set(gh.userFunctionsGUI.pbRemoveUserFcns,'Enable','off');

selectedTableCells('userFcns',[]);
end

function removeOverrideFcn(handles)

global state gh;

assert(~isempty(handles.selEvnt));

hOverrideMap = state.userFcns.(['hOverrideMap' state.userFcns.saveTarget]);
evntStruct = hOverrideMap(handles.selEvnt);

evntStruct.userFcnName = '';
evntStruct.userFcnKernel = [];

%Update EventMap
hOverrideMap(handles.selEvnt) = evntStruct;

%Update state vars
updateUserFunctionState('storeStateVars','OverrideFcns',[]);

% Unregister the override with OverrideManager
state.hSI.unregisterOverrideFcn(handles.selEvnt);

if strcmp(state.userFcns.saveTarget,'USR')
	state.userFcns.overrideStateUSR(handles.selEvnt) = false;
	state.userFcns.overrideStateCacheUSR(handles.selEvnt) = false;
elseif strcmp(state.userFcns.saveTarget,'CFG')
	state.userFcns.overrideStateCFG(handles.selEvnt) = false;
	
	% check if there is a shadowed USR override; if so, enable it
	if state.userFcns.overrideStateCacheUSR(handles.selEvnt)
		hOverrideMapUSR = state.userFcns.hOverrideMapUSR;
		usrStruct = hOverrideMapUSR(handles.selEvnt);
		state.userFcns.overrideStateUSR(handles.selEvnt) = true;
		state.hSI.registerOverrideFcn(handles.selEvnt,usrStruct.userFcnKernel);
	end
end

%Update GUI (table)
updateUserFunctionsGUI('OverrideFcns',handles.selEvnt);
set(gh.userFunctionsGUI.pbRemoveOverrideFcns,'Enable','off');

selectedTableCells('overrideFcns',[]);
end

function addOverrideFcn(fileName,handles)
global state gh;

hOverrideMap = state.userFcns.(['hOverrideMap' state.userFcns.saveTarget]);
overrideStruct = hOverrideMap(handles.selEvnt);

[pname,fnameNoExt] = fileparts(fileName);

%Construct function handle
if isempty(pname) %Built-in
    hOverride = str2func(fnameNoExt);
else   
    prevPath = addpath(pname,'-begin','-frozen');
    hOverride = str2func(fnameNoExt);
    path(prevPath);
end

%Update event structure record,
overrideStruct.userFcnName = fileName;
overrideStruct.userFcnKernel = hOverride;
hOverrideMap(handles.selEvnt) = overrideStruct;

overriddenFcn = handles.selEvnt;
if strcmp(state.userFcns.saveTarget,'USR')
	% check if there is an existing (and enabled) CFG override in the same position
	if state.userFcns.overrideStateCFG(overriddenFcn)
		% there is an existing CFG override, defer to it (i.e., don't register the override, but cache its state)
		state.userFcns.overrideStateUSR(overriddenFcn) = false;
	else
		state.hSI.registerOverrideFcn(overriddenFcn,hOverride);
		state.userFcns.overrideStateUSR(overriddenFcn) = true;
	end
	state.userFcns.overrideStateCacheUSR(overriddenFcn) = true;
elseif strcmp(state.userFcns.saveTarget,'CFG')
	% check if there is an existing (and enabled) USR override in the same position
	if state.userFcns.overrideStateUSR(handles.selEvnt)
		% there is an existing USR override, so unregister it and cache its state
		hOverrideMapUSR = state.usrFcns.hOverrideMapUSR;
		usrStruct = hOverrideMapUSR(overriddenFcn);
		state.hSI.unregisterOverrideFcn(usrStruct.userFcnKernel);
		state.userFcns.overrideStateUSR(overriddenFcn) = false;
		state.userFcns.overrideStateCacheUSR(overriddenFcn) = true;
	end
	state.userFcns.overrideStateCFG(overriddenFcn) = true;
	state.hSI.registerOverrideFcn(overriddenFcn,hOverride);
end


%Update state vars
updateUserFunctionState('storeStateVars','OverrideFcns',[]);

%Update GUI (table data)
updateUserFunctionsGUI('OverrideFcns',handles.selEvnt);
set(gh.userFunctionsGUI.pbAddOverrideFcns,'Enable','off');

selectedTableCells('overrideFcns',[]);
end


function addUSROnlyFcn(filename,handles)
global state gh;
	
	hUSROnlyMap = state.userFcns.hEventMapUSRONLY;
	usrOnlyStruct = hUSROnlyMap(handles.selEvnt);

	[pname,filenameNoExtension] = fileparts(filename);
	
	%Construct function handle
	if isempty(pname) %Built-in
		hUSROnlyFcn = str2func(filenameNoExtension);
	else   
		prevPath = addpath(pname,'-begin','-frozen');
		hUSROnlyFcn = str2func(filenameNoExtension);
		path(prevPath);
	end

	%Update event structure record,
	usrOnlyStruct.userFcnName = fullfile(pname,[filenameNoExtension '.m']);
	usrOnlyStruct.userFcnKernel = hUSROnlyFcn;
	hUSROnlyMap(handles.selEvnt) = usrOnlyStruct;

	%Update listener & state vars
	updateUserFunctionState('updateListener','USROnlyFcns',handles.selEvnt);
	updateUserFunctionState('storeStateVars','USROnlyFcns',[]);
	
	% Update the GUI
	updateUserFunctionsGUI('USROnlyFcns',handles.selEvnt);
	set(gh.userFunctionsGUI.pbAddUsrOnlyFcns,'Enable','off');
	
	selectedTableCells('usrOnlyFcns',[]);
end


function removeUSROnlyFcn(handles)
	global state gh;

	assert(~isempty(handles.selEvnt));

	hEventMap = state.userFcns.hEventMapUSRONLY;
	evntStruct = hEventMap(handles.selEvnt);

	evntStruct.userFcnName = '';
	evntStruct.userFcnKernel = [];

	%Update EventMap
	hEventMap(handles.selEvnt) = evntStruct;

	%Update listener & state vars
	updateUserFunctionState('updateListener','USROnlyFcns',handles.selEvnt);
	updateUserFunctionState('storeStateVars','USROnlyFcns',[]);

	updateUserFunctionsGUI('USROnlyFcns',handles.selEvnt);
	set(gh.userFunctionsGUI.pbRemoveUsrOnlyFcns,'Enable','off');
	
	selectedTableCells('usrOnlyFcns',[]);
end


function zlclClearCells(table,indices)
% Deletes the existing contents of a cell(s) and sets it to its default value.

	global state gh;

	if table == gh.userFunctionsGUI.tblUSROnlyFcns
		isUSROnly = true;
	else
		isUSROnly = false;
	end

	for row = indices'
		tableData = get(table,'Data'); % this is inefficient, but guarantees that any updates to the table from outside this function are respected.
		columnTypes = get(table,'ColumnFormat');
		columnNames = get(table,'ColumnName');
		
		handles = struct();
		handles.selEvnt = tableData{row(1),1};
		
		% if the user has 'deleted' the userfcn, defer to removeXXXFcn().
		if strcmp(columnNames{row(2)},'UserFcn Name')
			switch table
				case gh.userFunctionsGUI.tblUserFcns
					removeUserFcn(handles);
				case gh.userFunctionsGUI.tblUSROnlyFcns
					removeUSROnlyFcn(handles);
				case gh.userFunctionsGUI.tblOverrideFcns
					removeOverrideFcn(handles);
			end
			continue;
		end
		
		eventdata = struct();
		eventdata.Indices = [row(1) row(2)];
		eventdata.PreviousData = tableData{row(1),row(2)};
		
		switch columnTypes{row(2)}
			case 'char'
				eventdata.EditData = '';
			case 'logical'
				eventdata.EditData = false;
			case 'numeric'
				eventdata.EditData = [];
		end
		tableData{row(1),row(2)} = eventdata.EditData;
		eventdata.NewData = eventdata.EditData;
		eventdata.Error = [];
		tblUserFcns_CellEditCallback(table,eventdata,handles,isUSROnly);
		
		updateTable(table,tableData);
	end
end

function updateTable(table,tableData)
	set(table,'Data',tableData);
end


% --- Executes on button press in pbAddUserFcns.
function pbAddUserFcns_Callback(hObject, eventdata, handles)
	addFcn('userFcns');
end

% --- Executes on button press in pbRemoveUserFcns.
function pbRemoveUserFcns_Callback(hObject, eventdata, handles)
	removeFcn('userFcns');
end


% --- Executes on button press in pbAddUsrOnlyFcns.
function pbAddUsrOnlyFcns_Callback(hObject, eventdata, handles)
	addFcn('usrOnlyFcns');
end

% --- Executes on button press in pbRemoveUsrOnlyFcns.
function pbRemoveUsrOnlyFcns_Callback(hObject, eventdata, handles)
	removeFcn('usrOnlyFcns');
end


% --- Executes on button press in pbAddOverrideFcns.
function pbAddOverrideFcns_Callback(hObject, eventdata, handles)
	addFcn('overrideFcns');
end


% --- Executes on button press in pbRemoveOverrideFcns.
function pbRemoveOverrideFcns_Callback(hObject, eventdata, handles)
	removeFcn('overrideFcns');
end

function addFcn(tableName)
	global state gh;

	indices = selectedTableCells(tableName);
	
	tableData = getTableData(tableName);

	if ~isempty([tableData{indices(:,1),2}])
		choice = questdlg('One or more of the selected cells are not empty--the existing data will be replaced...', ...
					'Existing User Functions', ...
					'Continue','Cancel','Continue');
		if strcmp(choice,'Cancel')
			return;
		end
	end
	
	try
		%Prompt user to select file
		startPath = state.hSI.getLastPath([tableName 'LastPath']);
		[fname, pname]=uigetfile({'*.m'},'Choose User Function File...',startPath);
		if isnumeric(fname)
			return
		else
			[~,filenameNoExtension,~] = fileparts(fname);

			if ~strcmp(pname,startPath)
				state.hSI.setLastPath([tableName 'LastPath'],pname);
			end
		end
	catch ME
		ME.throwAsCaller();
	end

	for row = indices'
		row = row';
		handles.selEvnt = tableData{row(1),1};
		switch tableName
			case 'userFcns'
				addUserFcn(fullfile(pname,fname),handles);
			case 'usrOnlyFcns'
				addUSROnlyFcn(fullfile(pname,fname),handles);
			case 'overrideFcns'
				addOverrideFcn(fullfile(pname,fname),handles);
		end
	end
end


function removeFcn(tableName)
	FCN_NAME_COL_IDX = 2; 
	global state gh;
	
	indices = selectedTableCells(tableName);
	tableData = getTableData(tableName);
	
	for row = indices'
		row = row';
		if isempty(tableData{row(1),FCN_NAME_COL_IDX})
			continue;
		end
		handles.selEvnt = tableData{row(1),1};
		switch tableName
			case 'userFcns'
				removeUserFcn(handles);
			case 'usrOnlyFcns'
				removeUSROnlyFcn(handles);
			case 'overrideFcns'
				removeOverrideFcn(handles);
		end
	end
end

function data = getTableData(table)
	global gh;
	
	if nargin < 1 || isempty(table)
		error('You must specify a table.');
	end
	
	if ischar(table)
		% user gave a table name--determine its graphics handle:
		switch table
			case 'userFcns'
				table = gh.userFunctionsGUI.tblUserFcns;
			case 'usrOnlyFcns'
				table = gh.userFunctionsGUI.tblUSROnlyFcns;
			case 'overrideFcns'
				table = gh.userFunctionsGUI.tblOverrideFcns;
		end
	end
	
	data = get(table,'Data');
end


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
hideGUI(hObject);
end
