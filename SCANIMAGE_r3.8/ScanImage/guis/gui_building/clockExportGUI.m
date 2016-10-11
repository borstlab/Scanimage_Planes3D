function varargout = clockExportGUI(varargin)
% CLOCKEXPORTGUI MATLAB code for clockExportGUI.fig
%      CLOCKEXPORTGUI, by itself, creates a new CLOCKEXPORTGUI or raises the existing
%      singleton*.
%
%      H = CLOCKEXPORTGUI returns the handle to a new CLOCKEXPORTGUI or the handle to
%      the existing singleton*.
%
%      CLOCKEXPORTGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CLOCKEXPORTGUI.M with the given input arguments.
%
%      CLOCKEXPORTGUI('Property','Value',...) creates a new CLOCKEXPORTGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before clockExportGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to clockExportGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help clockExportGUI

% Last Modified by GUIDE v2.5 30-Aug-2011 20:17:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @clockExportGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @clockExportGUI_OutputFcn, ...
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


% --- Executes just before clockExportGUI is made visible.
function clockExportGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to clockExportGUI (see VARARGIN)

% Choose default command line output for clockExportGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes clockExportGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);
return;

%--------------------------------------------------------------------------
% --- Outputs from this function are returned to the command line.
function varargout = clockExportGUI_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;
return;

%--------------------------------------------------------------------------
% --- Executes on button press in frameClockPolarityLow.
function frameClockPolarityLow_Callback(hObject, eventdata, handles)
global state

state.acq.clockExport.frameClockPolarityHigh = 0;
updateGUIByGlobal('state.acq.clockExport.frameClockPolarityHigh');
genericCallback(hObject);

handleConfigChange(true); %VI111110A

return;

%--------------------------------------------------------------------------
% --- Executes on button press in frameClockPolarityHigh.
function frameClockPolarityHigh_Callback(hObject, eventdata, handles)
global state

state.acq.clockExport.frameClockPolarityLow = 0;
updateGUIByGlobal('state.acq.clockExport.frameClockPolarityLow');
genericCallback(hObject);

handleConfigChange(true); %VI111110A

return;

%--------------------------------------------------------------------------
% --- Executes on button press in pixelClockGated.
function pixelClockGated_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;

%--------------------------------------------------------------------------
% --- Executes on button press in pixelClockGated.
function lineClockGated_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;

%--------------------------------------------------------------------------
function lineClockGateSource_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;


%--------------------------------------------------------------------------
% --- Executes on button press in lineClockPolarityLow.
function lineClockPolarityLow_Callback(hObject, eventdata, handles)
global state

state.acq.clockExport.lineClockPolarityHigh = 0;
updateGUIByGlobal('state.acq.clockExport.lineClockPolarityHigh');
genericCallback(hObject);

handleConfigChange(true); %VI111110A
return;

%--------------------------------------------------------------------------
% --- Executes on button press in lineClockPolarityHigh.
function lineClockPolarityHigh_Callback(hObject, eventdata, handles)
global state

state.acq.clockExport.lineClockPolarityLow = 0;
updateGUIByGlobal('state.acq.clockExport.lineClockPolarityLow');
genericCallback(hObject);

handleConfigChange(true); %VI111110A

return;

%--------------------------------------------------------------------------
% --- Executes on button press in pixelClockPolarityLow.
function pixelClockPolarityLow_Callback(hObject, eventdata, handles)
global state

state.acq.clockExport.pixelClockPolarityHigh = 0;
updateGUIByGlobal('state.acq.clockExport.pixelClockPolarityHigh');
genericCallback(hObject);

handleConfigChange(true); %VI111110A 

return;

%--------------------------------------------------------------------------
% --- Executes on button press in pixelClockPolarityHigh.
function pixelClockPolarityHigh_Callback(hObject, eventdata, handles)
global state

state.acq.clockExport.pixelClockPolarityLow = 0;
updateGUIByGlobal('state.acq.clockExport.pixelClockPolarityLow');
genericCallback(hObject);
handleConfigChange(true); %VI111110A


return;

%--------------------------------------------------------------------------
function pixelClockGateSource_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;


%--------------------------------------------------------------------------
function frameClockGateSource_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 

return;


%--------------------------------------------------------------------------
% --- Executes on button press in frameClockGated.
function frameClockGated_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;

%--------------------------------------------------------------------------
function pixelClockPulseWidthFraction_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;


%--------------------------------------------------------------------------
% --- Executes on button press in lineClockAutoSource.
function lineClockAutoSource_Callback(hObject, eventdata, handles)
global state gh

genericCallback(hObject);
if state.acq.clockExport.lineClockAutoSource
    set(gh.clockExportGUI.lineClockGateSource, 'Enable', 'Off');
else
    set(gh.clockExportGUI.lineClockGateSource, 'Enable', 'On');
end
drawnow update;

handleConfigChange(); 

return;

%--------------------------------------------------------------------------
% --- Executes on button press in pixelClockAutoSource.
function pixelClockAutoSource_Callback(hObject, eventdata, handles)
global state gh

genericCallback(hObject);

if state.acq.clockExport.pixelClockAutoSource
    set(gh.clockExportGUI.pixelClockGateSource, 'Enable', 'Off');
else
    set(gh.clockExportGUI.pixelClockGateSource, 'Enable', 'On');
end
drawnow update;

handleConfigChange(); 

return;

%--------------------------------------------------------------------------
function cbExportClockOnFocus_Callback(hObject, eventdata, handles)
genericCallback(hObject);
return;

%--------------------------------------------------------------------------
function cbPixelClockEnable_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;

%--------------------------------------------------------------------------
function cbFrameClockEnable_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;

%--------------------------------------------------------------------------
function cbLineClockEnable_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;

function etFrameClockDelay_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;

%--------------------------------------------------------------------------
function etLineClockDelay_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;

%--------------------------------------------------------------------------
function etPixelClockDelay_Callback(hObject, eventdata, handles)
genericCallback(hObject);
handleConfigChange(); 
return;

%--------------------------------------------------------------------------
function pbSaveUSR_Callback(hObject, eventdata, handles)
saveCurrentUserSettings();
return;

%--------------------------------------------------------------------------
function pbSaveCFG_Callback(hObject, eventdata, handles)
saveCurrentConfig();
return;

%% HELPER FUNCTIONS
function handleConfigChange(polarityChange)

global state 

exportClocks(); %Actually apply configuration changes right away

%%%VI111110A: Ensure idle state change (i.e. polarity change) takes effect immediately
if nargin && polarityChange
    state.acq.hClockTasks.start();
    state.acq.hClockTasks.abort();
end

setConfigurationNeedsSaving(); %Flag that congfiguration is not yet saved


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
hideGUI(hObject);