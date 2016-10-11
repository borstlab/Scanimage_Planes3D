function varargout = roiDisplayGUI(varargin)
% ROIDISPLAYGUI MATLAB code for roiDisplayGUI.fig
%      ROIDISPLAYGUI, by itself, creates a new ROIDISPLAYGUI or raises the existing
%      singleton*.
%
%      H = ROIDISPLAYGUI returns the handle to a new ROIDISPLAYGUI or the handle to
%      the existing singleton*.
%
%      ROIDISPLAYGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROIDISPLAYGUI.M with the given input arguments.
%
%      ROIDISPLAYGUI('Property','Value',...) creates a new ROIDISPLAYGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before roiDisplayGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to roiDisplayGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help roiDisplayGUI

% Last Modified by GUIDE v2.5 06-Jun-2012 10:24:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @roiDisplayGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @roiDisplayGUI_OutputFcn, ...
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


% --- Executes just before roiDisplayGUI is made visible.
function roiDisplayGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to roiDisplayGUI (see VARARGIN)

% Choose default command line output for roiDisplayGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes roiDisplayGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = roiDisplayGUI_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;


% --- Executes on button press in pbZoomIn.
function pbZoomIn_Callback(hObject, eventdata, handles)
handles.hController.roiDisplayZoom('in');

% --- Executes on button press in pbZoomOut.
function pbZoomOut_Callback(hObject, eventdata, handles)
handles.hController.roiDisplayZoom('out');

% --- Executes on button press in pbPan.
function pbPan_Callback(hObject, eventdata, handles)
pan;

function etPosnID_Callback(hObject, eventdata, handles)
% hObject    handle to etPosnID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etPosnID as text
%        str2double(get(hObject,'String')) returns contents of etPosnID as a double


% --- Executes during object creation, after setting all properties.
function etPosnID_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPosnID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function etPositionString_Callback(hObject, eventdata, handles)
% hObject    handle to etPositionString (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etPositionString as text
%        str2double(get(hObject,'String')) returns contents of etPositionString as a double


% --- Executes during object creation, after setting all properties.
function etPositionString_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPositionString (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function etDisplayDepth_Callback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);

% --- Executes during object creation, after setting all properties.
function etDisplayDepth_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function pbShowParent_ClickedCallback(hObject, eventdata, handles)
handles.hController.roiUpOneLevel_Callback();


% --------------------------------------------------------------------
function pbShowActive_ClickedCallback(hObject, eventdata, handles)
handles.hModel.roiShowActive();

% --------------------------------------------------------------------
function pbGotoShown_ClickedCallback(hObject, eventdata, handles)
handles.hModel.roiGotoROI(handles.hModel.shownROI);

% --------------------------------------------------------------------
function tbDepthOne_ClickedCallback(hObject, eventdata, handles)
handles.hModel.roiDisplayDepth = 1;

% --------------------------------------------------------------------
function tbDepthTwo_ClickedCallback(hObject, eventdata, handles)
handles.hModel.roiDisplayDepth = 2;

% --------------------------------------------------------------------
function tbDepthInf_ClickedCallback(hObject, eventdata, handles)
handles.hModel.roiDisplayDepth = inf;

% --------------------------------------------------------------------
function pbClearShown_ClickedCallback(hObject, eventdata, handles)
handles.hController.removeROI_Callback(num2cell(handles.hModel.roiGetDisplayedChildren()));

% --------------------------------------------------------------------
function tbDisplayNumbers_ClickedCallback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);

% --------------------------------------------------------------------
function tbOne_ClickedCallback(hObject, eventdata, handles)
handles.hModel.roiDisplayedChannel = '1';

% --------------------------------------------------------------------
function tbTwo_ClickedCallback(hObject, eventdata, handles)
handles.hModel.roiDisplayedChannel = '2';

% --------------------------------------------------------------------
function tbThree_ClickedCallback(hObject, eventdata, handles)
handles.hModel.roiDisplayedChannel = '3';

% --------------------------------------------------------------------
function tbFour_ClickedCallback(hObject, eventdata, handles)
handles.hModel.roiDisplayedChannel = '4';

% --------------------------------------------------------------------
function tbMerge_ClickedCallback(hObject, eventdata, handles)
handles.hModel.roiDisplayedChannel = 'merge';

function etShownROI_Callback(hObject, eventdata, handles)
% hObject    handle to etShownROI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etShownROI as text
%        str2double(get(hObject,'String')) returns contents of etShownROI as a double
handles.hController.updateModel(hObject,eventdata,handles);


% --- Executes on button press in pbShowRootROI.
function pbShowRootROI_Callback(hObject, eventdata, handles)
% hObject    handle to pbShowRootROI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hModel.shownROI = handles.hModel.ROI_ROOT_ID;


function etShownRotation_Callback(hObject, eventdata, handles)
% hObject    handle to etShownRotation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etShownRotation as text
%        str2double(get(hObject,'String')) returns contents of etShownRotation as a double


% --- Executes during object creation, after setting all properties.
function etShownRotation_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etShownRotation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in lbPositionIDs.
function lbPositionIDs_Callback(hObject, eventdata, handles)
% hObject    handle to lbPositionIDs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns lbPositionIDs contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lbPositionIDs
handles.hController.changeROISelectedPositionID(hObject);

% --- Executes during object creation, after setting all properties.
function lbPositionIDs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lbPositionIDs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in lbScanRotations.
function lbScanRotations_Callback(hObject, eventdata, handles)
% hObject    handle to lbScanRotations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns lbScanRotations contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lbScanRotations

% --- Executes during object creation, after setting all properties.
function lbScanRotations_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lbScanRotations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pbSetShownROI.
function pbSetShownROI_Callback(hObject, eventdata, handles)
% hObject    handle to pbSetShownROI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.changeShownROI();
