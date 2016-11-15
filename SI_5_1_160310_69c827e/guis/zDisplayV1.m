function varargout = zDisplayV1(varargin)
% ZDISPLAYV1 MATLAB code for zDisplayV1.fig
%      ZDISPLAYV1, by itself, creates a new ZDISPLAYV1 or raises the existing
%      singleton*.
%
%      H = ZDISPLAYV1 returns the handle to a new ZDISPLAYV1 or the handle to
%      the existing singleton*.
%
%      ZDISPLAYV1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ZDISPLAYV1.M with the given input arguments.
%
%      ZDISPLAYV1('Property','Value',...) creates a new ZDISPLAYV1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before zDisplayV1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to zDisplayV1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help zDisplayV1

% Last Modified by GUIDE v2.5 11-Nov-2016 12:46:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @zDisplayV1_OpeningFcn, ...
                   'gui_OutputFcn',  @zDisplayV1_OutputFcn, ...
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

% --- Executes just before zDisplayV1 is made visible.
function zDisplayV1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to zDisplayV1 (see VARARGIN)

% Choose default command line output for zDisplayV1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
% This sets up the initial plot - only do when we are invisible
% so window can get raised using zDisplayV1.
if strcmp(get(hObject,'Visible'),'off')
    %plot initial plane
    
end
% UIWAIT makes zDisplayV1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = zDisplayV1_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



% --------------------------------------------------------------------
function uiupdateXDisplay_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to uiupdateXDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
state = sum(sign(get(handles.setColorMap,'State')));
handles.hController.updateZDisplay(hObject,handles,state);

% --------------------------------------------------------------------
function setColorMap_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to setColorMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function setColorMap_OffCallback(hObject, eventdata, handles)
% hObject    handle to setColorMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
state = sum(sign(get(hObject,'State')));
handles.hController.updateZDisplay(hObject,handles,state);


% --------------------------------------------------------------------
function setColorMap_OnCallback(hObject, eventdata, handles)
% hObject    handle to setColorMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
state = sum(sign(get(hObject,'State')));
handles.hController.updateZDisplay(hObject,handles,state);


% --- Executes on selection change in changeUnits.
function changeUnits_Callback(hObject, eventdata, handles)
% hObject    handle to changeUnits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns changeUnits contents as cell array
%        contents{get(hObject,'Value')} returns selected item from changeUnits
% Determine the selected data set.
str = get(hObject, 'String');
val = get(hObject,'Value');
% Set current data to the selected data set.
% switch str{val};
% case 'Microns' % User selects peaks.
%    handles.current_data = handles.peaks;
% case 'FOV' % User selects membrane.
%    handles.current_data = handles.membrane;
% case 'Volts AO' % User selects sinc.
%    handles.current_data = handles.sinc;
% end
% % Save the handles structure.
% guidata(hObject,handles)
state = sum(sign(get(handles.hController.hGUIData.zDisplayV1.setColorMap,'State')));
handles.hController.updateZDisplay(hObject,handles,state);


% --- Executes during object creation, after setting all properties.
function changeUnits_CreateFcn(hObject, eventdata, handles)
% hObject    handle to changeUnits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
