function varargout = motorControls(varargin)
%MOTORCONTROLS M-file for motorControls.fig
%      MOTORCONTROLS, by itself, creates a new MOTORCONTROLS or raises the existing
%      singleton*.
%
%      H = MOTORCONTROLS returns the handle to a new MOTORCONTROLS or the handle to
%      the existing singleton*.
%
%      MOTORCONTROLS('Property','Value',...) creates a new MOTORCONTROLS using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to motorControls_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      MOTORCONTROLS('CALLBACK') and MOTORCONTROLS('CALLBACK',hObject,...) call the
%      local function named CALLBACK in MOTORCONTROLS.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help motorControls

% Last Modified by GUIDE v2.5 05-Jun-2012 15:41:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @motorControls_OpeningFcn, ...
                   'gui_OutputFcn',  @motorControls_OutputFcn, ...
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


% --- Executes just before motorControls is made visible.
function motorControls_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for motorControls
handles.output = hObject;

%Ensure all controls/panels respond to key presses, when they have the focus (for whatever reason)
fig = openfig(mfilename, 'reuse');
set(fig,'KeyPressFcn',@genericKeyPressFunction);
kidControls = findall(fig,'Type','uicontrol');
for i=1:length(kidControls)
    if ~strcmpi(get(kidControls(i),'Style'),'edit')
        set(kidControls(i),'KeyPressFcn',@genericKeyPressFunction);
    end
end
    

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes motorControls wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = motorControls_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% % --------------------------------------------------------------------
% function varargout = generic_Callback(h, eventdata, handles, varargin)
% % Stub for Callback of the uicontrol handle
% global gh
% figure(gh.motorControls.figure1)
% genericCallback(h);

%% CALLBACKS

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
hideGUI(hObject);


function etPosX_Callback(hObject, eventdata, handles)
% hObject    handle to etPosX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etPosX as text
%        str2double(get(hObject,'String')) returns contents of etPosX as a double
genericCallback(hObject);
motorSetPositionRelative([],'verify');  %VI060610A %VI032010A 


function etPosY_Callback(hObject, eventdata, handles)
% hObject    handle to etPosY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etPosY as text
%        str2double(get(hObject,'String')) returns contents of etPosY as a double
genericCallback(hObject);
motorSetPositionRelative([],'verify'); %VI060610A %VI032010A 


function etPosZ_Callback(hObject, eventdata, handles)
% hObject    handle to etPosZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etPosZ as text
%        str2double(get(hObject,'String')) returns contents of etPosZ as a double
genericCallback(hObject);
motorSetPositionRelative([],'verify'); %VI060610A %VI032010A 

function etPosZZ_Callback(hObject, eventdata, handles)
% hObject    handle to etPosZZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etPosZZ as text
%        str2double(get(hObject,'String')) returns contents of etPosZZ as a double
genericCallback(hObject);
motorSetPositionRelative([],'verify'); %VI060610A %VI032010A 

function etNumberOfZSlices_Callback(hObject, eventdata, handles)
% hObject    handle to etNumberOfZSlices (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etNumberOfZSlices as text
%        str2double(get(hObject,'String')) returns contents of etNumberOfZSlices as a double
genericCallback(hObject);
updateNumberOfZSlices(hObject);

function etZStepPerSlice_Callback(hObject, eventdata, handles)
% hObject    handle to etZStepPerSlice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etZStepPerSlice as text
%        str2double(get(hObject,'String')) returns contents of etZStepPerSlice as a double

genericCallback(hObject);

global state
if ~isempty(state.motor.stackStart) && ~isempty(state.motor.stackStop)        
    if state.motor.stackEndpointsDominate %VI111108A
        calculateStackParameters(); %VI051111A
    end
end

% --- Executes on button press in cbReturnHome.
function cbReturnHome_Callback(hObject, eventdata, handles)
% hObject    handle to cbReturnHome (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbReturnHome
genericCallback(hObject);

% --- Executes on button press in cbCenteredStack.
function cbCenteredStack_Callback(hObject, eventdata, handles)
% hObject    handle to cbCenteredStack (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbCenteredStack
genericCallback(hObject);

% --- Executes on button press in pbAltZeroXY.
function pbAltZeroXY_Callback(hObject, eventdata, handles)
% hObject    handle to pbAltZeroXY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
motorSetRelativeOrigin([1 1 0]);

% --- Executes on button press in pbAltZeroZ.
function pbAltZeroZ_Callback(hObject, eventdata, handles)
% hObject    handle to pbAltZeroZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
if state.motor.dimensionsXYZZ && state.motor.motorZEnable
    motorSetRelativeOrigin([0 0 0 1]);
else
    motorSetRelativeOrigin([0 0 1]);
end    

% --- Executes on button press in cbSecZ.
function cbSecZ_Callback(hObject, eventdata, handles)
% hObject    handle to cbSecZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbSecZ
genericCallback(hObject);

% --- Executes on button press in pbReadPos.
function pbReadPos_Callback(hObject, eventdata, handles)
% hObject    handle to pbReadPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
turnOffMotorButtons;
motorGetPosition();
turnOnMotorButtons;

% % --------------------------------------------------------------------
% function varargout = relPos_Callback(h, eventdata, handles, varargin)
% % Stub for Callback of the uicontrol handle
% genericCallback(h);
% motorAction(@()motorSetPositionRelative([],'verify'),'Moving motor to new specified coordinates.') %VI060610A %VI032010A 


% --- Executes on button press in pbZeroXY.
function pbZeroXY_Callback(hObject, eventdata, handles)
% hObject    handle to pbZeroXY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
motorSetRelativeOrigin([1 1 0]);


% --- Executes on button press in pbZeroZ.
function pbZeroZ_Callback(hObject, eventdata, handles)
% hObject    handle to pbZeroZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
motorSetRelativeOrigin([0 0 1]);


% --- Executes on button press in pbZeroXYZ.
function pbZeroXYZ_Callback(hObject, eventdata, handles)
% hObject    handle to pbZeroXYZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
motorSetRelativeOrigin([1 1 1 1]);


% --- Executes on button press in pbSetStart.
function pbSetStart_Callback(hObject, eventdata, handles)
% hObject    handle to pbSetStart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state gh
figure(gh.motorControls.figure1)
turnOffMotorButtons;
%%%VI100908A
% flag=updateMotorPosition;
% if isempty(flag)
%     disp('setStackStart_Callback : Unable to set stack start.  MP285 Error');
%     beep;
% else

%%%VI032310A:Removed
% if ~isempty(updateMotorPosition) %ensure there's no error
%     state.motor.stackStart=state.motor.lastPositionRead;  

state.motor.stackStart = motorGetPosition(); %VI032310A
    
setStatusString('Stack start set');
if ~isempty(state.motor.stackStop) %VI100908A
    calculateStackParameters;
end

turnOnMotorButtons;
updateStackEndpoints; %VI113008A

%%%VI010610A: Removed%%%
%     %%VI052809A%%%%%%%%%
%     if any(state.init.eom.powerVsZEnableArray)
%         state.init.eom.powerVsZStartArray = state.init.eom.maxPower;
%     end
%     %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI010610A%%%%%%%%%%%%%%%
for i=1:state.init.eom.numberOfBeams
    state.init.eom.powerVsZStartArray(i) = state.init.eom.maxPower(i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on button press in pbSetEnd.
function pbSetEnd_Callback(hObject, eventdata, handles)
% hObject    handle to pbSetEnd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global state gh
figure(gh.motorControls.figure1);
turnOffMotorButtons;
%%%VI100908A
% flag=updateMotorPosition;
% if isempty(flag)
%     disp('setStackStop_Callback : Unable to set stack end.  MP285 Error');
%     beep;
% else

%%%VI032310A:Removed
%if ~isempty(updateMotorPosition)
%    state.motor.stackStop=state.motor.lastPositionRead;

state.motor.stackStop= motorGetPosition(); %VI032310A

setStatusString('Stack end set');
if ~isempty(state.motor.stackStart) %VI100908B
    calculateStackParameters;
end

turnOnMotorButtons;
updateStackEndpoints; %VI113008A

%%%VI010610A: Removed%%%
%     %%VI052809A%%%%%%%%%
%     if any(state.init.eom.powerVsZEnableArray) && state.init.eom.powerLzOverride %Only record power at end stack position if the 'Lz Override' feature is specified
%         state.init.eom.powerVsZEndArray = state.init.eom.maxPower;
%     end
%     %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI010610A%%%%%%%%%%%%%%%
for i=1:state.init.eom.numberOfBeams
    state.init.eom.powerVsZEndArray(i) = state.init.eom.maxPower(i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on button press in cbLockSliceVals.
function cbLockSliceVals_Callback(hObject, eventdata, handles)
% hObject    handle to cbLockSliceVals (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbLockSliceVals
% --------------------------------------------------------------------

genericCallback(hObject);

%%%VI112309A%%%%
global state
if ~state.motor.stackEndpointsDominate
    state.motor.stackStop = [];
end    
updateStackEndpoints();
%%%%%%%%%%%%%%%%

% --- Executes on button press in pbGrabOneStack.
function pbGrabOneStack_Callback(hObject, eventdata, handles)
% hObject    handle to pbGrabOneStack (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global gh
figure(gh.motorControls.figure1)
executeGrabOneStackCallback(hObject);

% MARKED FOR DELETION
% % --------------------------------------------------------------------
% function varargout = definePosition_Callback(h, eventdata, handles, varargin)
% % Stub for Callback of the uicontrol handles.definePosition.
% global gh
% figure(gh.motorControls.figure1)
% turnOffMotorButtons;
% motorPositionDefine();
% turnOnMotorButtons;

% MARKED FOR DELETION
% % --------------------------------------------------------------------
% function varargout = gotoPosition_Callback(h, eventdata, handles, varargin)
% % Stub for Callback of the uicontrol handles.gotoPosition.
% global gh state
% figure(gh.motorControls.figure1)
% turnOffMotorButtons;
% motorPositionGoto(state.motor.position,true); %VI060610A: This reads position and updates display on copmletion -- no separate motorGetPosition() call needed anymore
% turnOnMotorButtons;

% MARKED FOR DELETION
% % --------------------------------------------------------------------
% function varargout = shiftXY_Callback(h, eventdata, handles, varargin)
% % Stub for Callback of the uicontrol handles.gotoPosition.
% global gh state
% figure(gh.motorControls.figure1)
% turnOffMotorButtons;
% motorPositionShift(state.motor.position, [1 1 0],true);
% turnOnMotorButtons;

% MARKED FOR DELETION
% % --------------------------------------------------------------------
% function varargout = shiftXYZ_Callback(h, eventdata, handles, varargin)
% % Stub for Callback of the uicontrol handles.gotoPosition.
% global state gh
% figure(gh.motorControls.figure1)
% turnOffMotorButtons;
% motorPositionShift(state.motor.position, [1 1 1],true);
% turnOnMotorButtons;


% MARKED FOR DELETION
% % --------------------------------------------------------------------
% function varargout = savePositionListButton_Callback(h, eventdata, handles, varargin)
% % Stub for Callback of the uicontrol handles.savePositionListButton.
% global gh
% figure(gh.motorControls.figure1)
% savePositionListAs;

% MARKED FOR DELETION
% % --------------------------------------------------------------------
% function varargout = loadPositionListButton_Callback(h, eventdata, handles, varargin)
% % Stub for Callback of the uicontrol handles.loadPositionListButton.
% global gh
% figure(gh.motorControls.figure1)
% loadPositionList;


% --------------------------------------------------------------------
function pbRecover_Callback(hObject, eventdata, handles)
%MP285Recover; %VI040210A
motorRecover(); %VI040210A


% --------------------------------------------------------------------
function cbOverrideLz_Callback(hObject, eventdata, handles)
genericCallback(hObject);


% --- Executes on button press in pbStepXDec.
function pbStepXDec_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.motorStep_Callback('x','dec');

% --- Executes on button press in pbStepXInc.
function pbStepXInc_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.motorStep_Callback('x','inc');

% --- Executes on button press in pbStepYDec.
function pbStepYDec_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.motorStep_Callback('y','dec');

% --- Executes on button press in pbStepYInc.
function pbStepYInc_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.motorStep_Callback('y','inc');

% --- Executes on button press in pbStepZDec.
function pbStepZDec_Callback(hObject, eventdata, handles)
global state;
if state.motor.dimensionsXYZZ && state.motor.motorZEnable
    axis = 'zz';
    if state.motor.zDepthPositiveZ
        direction = 'inc';
    else
        direction = 'dec';
    end
else
    axis = 'z';
    if state.motor.zDepthPositive
        direction = 'inc';
    else
        direction = 'dec';
    end
end
state.hSICtl.motorStep_Callback(axis,direction);

% --- Executes on button press in pbStepZInc.
function pbStepZInc_Callback(hObject, eventdata, handles)
global state;

if state.motor.dimensionsXYZZ && state.motor.motorZEnable
    axis = 'zz';
    if state.motor.zDepthPositiveZ
        direction = 'dec';
    else
        direction = 'inc';
    end
else
    axis = 'z';
    if state.motor.zDepthPositive
        direction = 'dec';
    else
        direction = 'inc';
    end
end
state.hSICtl.motorStep_Callback(axis,direction);

function etStepSizeY_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.updateModel(hObject,eventdata,handles);

function etStepSizeZ_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.updateModel(hObject,eventdata,handles);

function etStepSizeZZ_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.updateModel(hObject,eventdata,handles);

function etStepSizeX_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.updateModel(hObject,eventdata,handles);


% % --- Executes on button press in pbZeroStep.
% function pbZeroStep_Callback(hObject, eventdata, handles)
% global state;
% state.hSICtl.motorZero_Callback();

% --- Executes on button press in tbTogglePosn.
function tbTogglePosn_Callback(hObject, eventdata, handles)
% hObject    handle to tbTogglePosn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of tbTogglePosn
global state
if get(hObject,'Value')   
    if isempty(state.internal.positionGUIBottom)
        tetherGUIs('motorControls', 'positionGUI', 'righttop');
    end
    seeGUI('gh.positionGUI.figure1');
else
    hideGUI('gh.positionGUI.figure1');
end

% --- Executes on button press in pbAddCurrent.
function pbAddCurrent_Callback(hObject, eventdata, handles)
% hObject    handle to pbAddCurrent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hModel.roiAddPosition();



%% CREATE FCNS

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

% --- Executes during object creation, after setting all properties.
function etStepSizeX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etStepSizeX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etStepSizeY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etStepSizeY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etStepSizeZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etStepSizeZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etNumberOfZSlices_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etNumberOfZSlices (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etZStepPerSlice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etZStepPerSlice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etStepSizeZZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etStepSizeZZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etPosY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPosY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function etPosZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPosZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function etPosX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPosX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function etPosR_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPosR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function etStackEnd_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etStackEnd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function etEndPower_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etEndPower (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function etStackStart_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etStackStart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function etStartPower_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etStartPower (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etPosZZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPosZZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pbReset.
function pbReset_Callback(hObject, eventdata, handles)
% hObject    handle to pbReset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function pbStepXDec_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pbStepXDec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,180,[0 0 1]));
end

% --- Executes during object creation, after setting all properties.
function pbStepXInc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pbStepXInc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,[],[0 0 1]));
end

% --- Executes during object creation, after setting all properties.
function pbStepYDec_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pbStepYDec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,90,[0 0 1]));
end

% --- Executes during object creation, after setting all properties.
function pbStepYInc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pbStepYInc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,270,[0 0 1]));
end

% --- Executes during object creation, after setting all properties.
function pbStepZDec_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pbStepZDec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,90,[0 0 1]));
end

% --- Executes during object creation, after setting all properties.
function pbStepZInc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pbStepZInc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,270,[0 0 1]));
end


% --- Executes on selection change in pmPosnID.
function pmPosnID_Callback(hObject, eventdata, handles)
% hObject    handle to pmPosnID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns pmPosnID contents as cell array
%        contents{get(hObject,'Value')} returns selected item from pmPosnID
handles.hController.changeActivePositionID(hObject);

% --- Executes during object creation, after setting all properties.
function pmPosnID_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmPosnID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
