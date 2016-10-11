function varargout = powerControlsV4(varargin)
%POWERCONTROLSV4 M-file for powerControlsV4.fig

% Last Modified by GUIDE v2.5 01-Jul-2015 14:11:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @powerControlsV4_OpeningFcn, ...
                   'gui_OutputFcn',  @powerControlsV4_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
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

function powerControlsV4_OpeningFcn(hObject, eventdata, handles, varargin) %#ok<*INUSL>
handles.output = hObject;
handles.powerTbl = most.gui.control.ColumnArrayTable(findobj(hObject,'Tag','tblBeams'));
guidata(hObject, handles);

function varargout = powerControlsV4_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;

%% Beam Index Control

function pumBeamIdx_Callback(hObject,eventdata,handles)
handles.hController.beamDisplayIdx = get(hObject,'Value');

function sldBeamIdx_Callback(hObject, eventdata, handles)
handles.hController.beamDisplayIdx = get(hObject,'Value');

%% Beam-Indexed Controls
function etBeamPower_Callback(hObject,eventdata,handles) %#ok<*DEFNU>
handles.hController.changeBeamPowersDisplay(hObject,eventdata,handles);

function sldBeamPower_Callback(hObject, eventdata, handles)
handles.hController.changeBeamPowersDisplay(hObject,eventdata,handles);

function etMaxLimit_Callback(hObject, eventdata, handles)
handles.hController.changeBeamParams(hObject,eventdata,handles);

function sldMaxLimit_Callback(hObject, eventdata, handles)
handles.hController.changeBeamParams(hObject,eventdata,handles);

function etZLengthConstant_Callback(hObject, eventdata, handles)
handles.hController.changeBeamParams(hObject,eventdata,handles);

function cbDirectMode_Callback(hObject, eventdata, handles)
handles.hController.changeBeamParams(hObject,eventdata,handles);

function cbPzAdjust_Callback(hObject, eventdata, handles)
handles.hController.changeBeamParams(hObject,eventdata,handles);

function etInterlaceDecimation_Callback(hObject, eventdata, handles)
handles.hController.changeBeamParams(hObject,eventdata,handles);

function etInterlaceOffset_Callback(hObject, eventdata, handles)
handles.hController.changeBeamParams(hObject,eventdata,handles);

%% Non-Beam-Indexed Controls

function rbPercentBeamPower_Callback(hObject, eventdata, handles)
handles.hController.hModel.powerUnits = 'percent';

function rbMilliwattBeamPower_Callback(hObject, eventdata, handles)
handles.hController.hModel.powerUnits = 'milliwatts';

%% Power Box

% % --- Executes on button press in tbShowPowerBox.
% function tbShowPowerBox_Callback(hObject, eventdata, handles)
% % hObject    handle to tbShowPowerBox (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hint: get(hObject,'Value') returns toggle state of tbShowPowerBox

%% Create Fcns

% --- Executes during object creation, after setting all properties.
function sldBeamIdx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sldBeamIdx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function pumBeamIdx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pumBeamIdx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function sldBeamPower_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sldBeamPower (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function sldMaxLimit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sldMaxLimit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function etZLengthConstant_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etZLengthConstant (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function etInterlaceDecimation_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etInterlaceDecimation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function etInterlaceOffset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etInterlaceOffset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function etBeamLead_Callback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);
% hObject    handle to etBeamLead (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etBeamLead as text
%        str2double(get(hObject,'String')) returns contents of etBeamLead as a double


% --- Executes during object creation, after setting all properties.
function etBeamLead_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etBeamLead (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in cbBlankFlyback.
function cbBlankFlyback_Callback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);
% hObject    handle to cbBlankFlyback (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbBlankFlyback


% --- Executes on button press in pbCalibrate.
function pbCalibrate_Callback(hObject, eventdata, handles)
% hObject    handle to pbCalibrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.calibrateBeam();


% --- Executes on button press in pbShowCurve.
function pbShowCurve_Callback(hObject, eventdata, handles)
% hObject    handle to pbShowCurve (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.showCalibrationCurve();


% --- Executes on button press in pbMeasureOffsets.
function pbMeasureOffsets_Callback(hObject, eventdata, handles)
% hObject    handle to pbMeasureOffsets (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.measureCalibrationOffset();



% --- Executes when entered data in editable cell(s) in tblBeams.
function tblBeams_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to tblBeams (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
colIdx = eventdata.Indices(2);
switch colIdx
    case 1
        handles.hController.beamDisplayIdx = eventdata.Indices(1);
        
    otherwise
        handles.hController.updateModel(hObject,eventdata,handles);
end


% --- Executes on button press in cbEnablePowerBox.
function cbEnablePowerBox_Callback(hObject, eventdata, handles)
% hObject    handle to cbEnablePowerBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.updateModel(hObject,eventdata,handles);


% --- Executes on button press in pbPowerBoxSettings.
function pbPowerBoxSettings_Callback(hObject, eventdata, handles)
% hObject    handle to pbPowerBoxSettings (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hController.showGUI('powerBoxControlsV4');
handles.hController.raiseGUI('powerBoxControlsV4');


%--------------------------------------------------------------------------%
% powerControlsV4.m                                                        %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
