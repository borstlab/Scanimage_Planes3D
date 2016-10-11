function varargout = bScope2ControlsV5(varargin)
% BSCOPE2CONTROLSV5 MATLAB code for BSCOPE2CONTROLSV5.fig
%      BSCOPE2CONTROLSV5, by itself, creates a new BSCOPE2CONTROLSV5 or raises the existing
%      singleton*.
%
%      H = BSCOPE2CONTROLSV5 returns the handle to a new BSCOPE2CONTROLSV5 or the handle to
%      the existing singleton*.
%
%      BSCOPE2CONTROLSV5('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BSCOPE2CONTROLSV5.M with the given input arguments.
%
%      BSCOPE2CONTROLSV5('Property','Value',...) creates a new BSCOPE2CONTROLSV5 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before bScope2ControlsV5_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to bScope2ControlsV5_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help bScope2ControlsV5

% Last Modified by GUIDE v2.5 12-Nov-2014 17:32:12

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @bScope2ControlsV5_OpeningFcn, ...
                   'gui_OutputFcn',  @bScope2ControlsV5_OutputFcn, ...
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


% --- Executes just before bScope2ControlsV5 is made visible.
function bScope2ControlsV5_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to bScope2ControlsV5 (see VARARGIN)

% Choose default command line output for bScope2ControlsV5
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes bScope2ControlsV5 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = bScope2ControlsV5_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pbPmt.
function pbPmt_Callback(hObject, eventdata, handles)
% hObject    handle to pbPmt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.changeFlipperMirrorPosition('pmt');


% --- Executes on button press in pbCamera.
function pbCamera_Callback(hObject, eventdata, handles)
% hObject    handle to pbCamera (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.changeFlipperMirrorPosition('camera');


% --- Executes on button press in pbGG_In.
function pbGG_In_Callback(hObject, eventdata, handles)
% hObject    handle to pbGG_In (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.changeGalvoGalvoMirrorInPath(1);


% --- Executes on button press in pbGG_Out.
function pbGG_Out_Callback(hObject, eventdata, handles)
% hObject    handle to pbGG_Out (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.changeGalvoGalvoMirrorInPath(0);


% --- Executes on button press in pbGR_In.
function pbGR_In_Callback(hObject, eventdata, handles)
% hObject    handle to pbGR_In (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.changeGalvoResonantMirrorInPath(1);


% --- Executes on button press in pbGR_Out.
function pbGR_Out_Callback(hObject, eventdata, handles)
% hObject    handle to pbGR_Out (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.changeGalvoResonantMirrorInPath(0);



function etRotationAngle_Callback(hObject, eventdata, handles)
% hObject    handle to etRotationAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.setBScope2RotationAngle(str2num(get(hObject,'String')));
% Hints: get(hObject,'String') returns contents of etRotationAngle as text
%        str2double(get(hObject,'String')) returns contents of etRotationAngle as a double


% --- Executes during object creation, after setting all properties.
function etRotationAngle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etRotationAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pbRotationAngle_Dec.
function pbRotationAngle_Dec_Callback(hObject, eventdata, handles)
% hObject    handle to pbRotationAngle_Dec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.stepBScope2RotationAngle(-1);


% --- Executes on button press in pbRotationAngle_Inc.
function pbRotationAngle_Inc_Callback(hObject, eventdata, handles)
% hObject    handle to pbRotationAngle_Inc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.stepBScope2RotationAngle(1);



function etRotationAngleStepSize_Callback(hObject, eventdata, handles)
% hObject    handle to etRotationAngleStepSize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.bScope2RotationAngleStepSize = str2num(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of etRotationAngleStepSize as text
%        str2double(get(hObject,'String')) returns contents of etRotationAngleStepSize as a double


% --- Executes during object creation, after setting all properties.
function etRotationAngleStepSize_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etRotationAngleStepSize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pbUpdateRotationAngle.
function pbUpdateRotationAngle_Callback(hObject, eventdata, handles)
% hObject    handle to pbUpdateRotationAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.changedBScope2RotationAngle();


% --- Executes on button press in pbResetLSC.
function pbResetLSC_Callback(hObject, eventdata, handles)
% hObject    handle to pbResetLSC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.hController.hThorBScope2LSC)
    handles.hController.hThorBScope2LSC.reset();
end



function etScanAlign_Callback(hObject, eventdata, handles)
% hObject    handle to etScanAlign (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
updateModel(handles.hController,hObject,eventdata,handles);
% Hints: get(hObject,'String') returns contents of etScanAlign as text
%        str2double(get(hObject,'String')) returns contents of etScanAlign as a double


% --- Executes during object creation, after setting all properties.
function etScanAlign_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etScanAlign (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slScanAlign_Callback(hObject, eventdata, handles)
% hObject    handle to slScanAlign (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject,'Value',floor(get(hObject,'Value')));
updateModel(handles.hController,hObject,eventdata,handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slScanAlign_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slScanAlign (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


%--------------------------------------------------------------------------%
% bScope2ControlsV5.m                                                      %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
