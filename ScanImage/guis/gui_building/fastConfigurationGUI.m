function varargout = fastConfigurationGUI(varargin)
% FASTCONFIGURATIONGUI MATLAB code for fastConfigurationGUI.fig
%      FASTCONFIGURATIONGUI, by itself, creates a new FASTCONFIGURATIONGUI or raises the existing
%      singleton*.
%
%      H = FASTCONFIGURATIONGUI returns the handle to a new FASTCONFIGURATIONGUI or the handle to
%      the existing singleton*.
%
%      FASTCONFIGURATIONGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FASTCONFIGURATIONGUI.M with the given input arguments.
%
%      FASTCONFIGURATIONGUI('Property','Value',...) creates a new FASTCONFIGURATIONGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before fastConfigurationGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to fastConfigurationGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help fastConfigurationGUI

% Last Modified by GUIDE v2.5 30-Aug-2011 20:18:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @fastConfigurationGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @fastConfigurationGUI_OutputFcn, ...
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


% --- Executes just before fastConfigurationGUI is made visible.
function fastConfigurationGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to fastConfigurationGUI (see VARARGIN)

% Choose default command line output for fastConfigurationGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes fastConfigurationGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = fastConfigurationGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function pbSaveUSR_Callback(hObject, eventdata, handles)
saveCurrentUserSettings();

% --------------------------------------------------------------------
function tblFastConfig_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to tblFastConfig (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

    global state

    rowIdx = eventdata.Indices(1);
    colIdx = eventdata.Indices(2);
    
    if ~isempty(eventdata.NewData)
        switch colIdx %the column specifier
            
            case 3 %CFG Name column                
                %Disallow edit
                tableData = get(hObject,'Data');
                tableData{rowIdx, colIdx} = eventdata.PreviousData;
                set(hObject,'Data',tableData);
                
            case 4 %Autostart (logical)
                state.files.fastConfigAutoStartArray(rowIdx) = eventdata.NewData;
                updateFastConfigTable(rowIdx,'autoStart');
                
            case 5 %AutostartType (string)                
                state.files.fastConfigAutoStartTypeArray{rowIdx} = eventdata.NewData;
                
                %                 typeArray = state.files.fastConfigAutoStartTypeArray;
                %
                %                 typeStringEnds = strfind(typeArray,',');
                %                 typeStringEnd = typeStringEnds(rowIdx) - 1;
                %                 if rowIdx == 1
                %                     typeStringStart = 1;
                %                 else
                %                     typeStringStart = typeStringEnds(rowIdx-1) + 1;
                %                 end
                %
                %                 typeArray(typeStringStart:typeStringEnd) = '';
                %                 typeArray = [typeArray(1:(typeStringStart-1)) eventdata.NewData typeArray(typeStringStart:end)];
                %
                %                 state.files.fastConfigAutoStartTypeArray = typeArray;

            otherwise
                assert(false);
        end
    end

return;


% --------------------------------------------------------------------
function pbBrowse1_Callback(hObject, eventdata, handles)
setFastConfiguration(1,handles);

% --------------------------------------------------------------------
function pbBrowse2_Callback(hObject, eventdata, handles)
setFastConfiguration(2,handles);

% --------------------------------------------------------------------
function pbBrowse3_Callback(hObject, eventdata, handles)
setFastConfiguration(3,handles);

% --------------------------------------------------------------------
function pbBrowse4_Callback(hObject, eventdata, handles)
setFastConfiguration(4,handles);

% --------------------------------------------------------------------
function pbBrowse5_Callback(hObject, eventdata, handles)
setFastConfiguration(5,handles);

% --------------------------------------------------------------------
function pbBrowse6_Callback(hObject, eventdata, handles)
setFastConfiguration(6,handles);

% --------------------------------------------------------------------
function cbCtlFastConfigHotKeys_Callback(hObject, eventdata, handles)
genericCallback(hObject);
updateFastConfigTable(); %Call this to ensure that 'CTL+' is added to tooltips on toggle buttons; ideally, this would have been in updateFastConfigButtons() instead


% --------------------------------------------------------------------
function pbHelp_Callback(hObject, eventdata, handles)

helpString = [...
    'Fast Configurations allow loading a saved configuration (CFG) file with ' ...
    'one button -- either buttons on MAIN CONTROLS window or F1-F6 keys.' ...
    '\newline\newline' ...
    'To assign CFG file to Fast Config <1-6>, press corresponding ' ...
    'browse {\bf(...)} button, and then select file.' ...
    '\newline\newline' ...
    '{\bf  CFG Name:} Displays name of CFG file assigned to Fast Config <1-6>, if any. To recall path of file, select (...) and Cancel.' ...
    '\newline' ...
    '{\bf  AutoStart?:} If selected, pressing Fast Config button or F1-F6, will automaticaly start acquisition.\\' ...
    '\newline' ...
    '{\bf  AutoStart Type:} Specifies what type of acquisition is automatically started, when AutoStart is true.'...
    ];

msgbox(helpString,'FAST CONFIGURATIONS (Help)','help',struct('WindowStyle','modal','Interpreter','tex'));


% --------------------------------------------------------------------
function pbRemove1_Callback(hObject, eventdata, handles)
removeFastConfiguration(1,handles);



% --------------------------------------------------------------------
function pbRemove2_Callback(hObject, eventdata, handles)
removeFastConfiguration(2,handles);


% --------------------------------------------------------------------
function pbRemove3_Callback(hObject, eventdata, handles)
removeFastConfiguration(3,handles);


% --------------------------------------------------------------------
function pbRemove4_Callback(hObject, eventdata, handles)
removeFastConfiguration(4,handles);


% --------------------------------------------------------------------
function pbRemove5_Callback(hObject, eventdata, handles)
removeFastConfiguration(5,handles);


% --------------------------------------------------------------------
function pbRemove6_Callback(hObject, eventdata, handles)
removeFastConfiguration(6,handles);


%% HELPER FUNCTIONS

function removeFastConfiguration(fastConfigNum,handles)

global state

currentFastConfig = state.files.(['fastConfig' num2str(fastConfigNum)]);

if ~isempty(currentFastConfig) 
    state.files.(['fastConfig' num2str(fastConfigNum)]) = '';
    updateFastConfigTable(fastConfigNum,'configName'); %Update the table display
end
updateFastConfigButtons(); %VI102810A: Update display of toggle buttons

% remove cached config data
removeCachedConfiguration(currentFastConfig);

return;

function setFastConfiguration(fastConfigNum,handles)

global state

currentFastConfig = state.files.(['fastConfig' num2str(fastConfigNum)]);
if ~isempty(currentFastConfig) && exist(currentFastConfig,'file')
    startPath = fileparts(currentFastConfig);
else    
    if ~isempty(state.configPath) && isdir(state.configPath)
        startPath = state.configPath;
    elseif ~isempty(state.files.lastFastConfigPath) && isdir(state.files.lastFastConfigPath)
        startPath = state.files.lastFastConfigPath;
    else
        startPath = most.idioms.startPath(); %VI111110A
    end
end

%Ensure that path has slash at end
if ~strcmpi(startPath(end),filesep)
    startPath(end+1) = filesep;
end

%Prompt user to select file
[fname, pname]=uigetfile([startPath '*.cfg'], 'Choose Configuration name...');
if isnumeric(fname)
    return
end

%Update state vars & display of configuration
state.files.lastFastConfigPath = pname;
state.files.(['fastConfig' num2str(fastConfigNum)]) = fullfile(pname,fname);
cacheConfiguration(fullfile(pname,fname));
updateFastConfigTable(fastConfigNum,'configName');
updateFastConfigButtons(); %VI102810A: Update display of toggle buttons


return;


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
hideGUI(hObject);
