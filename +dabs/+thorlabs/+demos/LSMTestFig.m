function varargout = LSMTestFig(varargin)
% LSMTESTFIG M-file for LSMTestFig.fig
%      LSMTESTFIG, by itself, creates a new LSMTESTFIG or raises the existing
%      singleton*.
%
%      H = LSMTESTFIG returns the handle to a new LSMTESTFIG or the handle to
%      the existing singleton*.
%
%      LSMTESTFIG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LSMTESTFIG.M with the given input arguments.
%
%      LSMTESTFIG('Property','Value',...) creates a new LSMTESTFIG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LSMTestFig_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LSMTestFig_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LSMTestFig

% Last Modified by GUIDE v2.5 24-Aug-2010 12:14:49

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LSMTestFig_OpeningFcn, ...
                   'gui_OutputFcn',  @LSMTestFig_OutputFcn, ...
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


% --- Executes just before LSMTestFig is made visible.
function LSMTestFig_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LSMTestFig (see VARARGIN)

global LSMTest;

LSMTest.LSMInitialized = 0;
LSMTest.framesToAcquire = 200;

pixelsPerDim = 512;

figH = figure(1);
LSMTest.axes1Hnd = axes;
set(figH, 'menubar', 'none');
set(figH, 'Position', [100 100 pixelsPerDim pixelsPerDim])
set(figH, 'Name', 'Channel 1');

figH = figure(2);
LSMTest.axes2Hnd = axes;
set(figH, 'Position', [700 100 pixelsPerDim pixelsPerDim])
set(figH, 'menubar', 'none');
set(figH, 'Name', 'Channel 2');

LSMTest.image1Hnd = image(zeros(pixelsPerDim,pixelsPerDim,'uint16'),'Parent',LSMTest.axes1Hnd);
LSMTest.image2Hnd = image(zeros(pixelsPerDim,pixelsPerDim,'uint16'),'Parent',LSMTest.axes2Hnd);

set(LSMTest.image1Hnd, 'CDataMapping', 'scaled');
set(LSMTest.image2Hnd, 'CDataMapping', 'scaled');

set(LSMTest.axes1Hnd, 'CLim', [0 2^14-1])
set(LSMTest.axes2Hnd, 'CLim', [0 2^14-1])

%set([LSMTest.axes1Hnd LSMTest.axes2Hnd],'XLim',[1 pixelsPerDim],'YLim',[1 pixelsPerDim])


set(handles.pixelsPerDim_edit, 'String', num2str(pixelsPerDim));
% Choose default command line output for LSMTestFig
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LSMTestFig wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LSMTestFig_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in initLSM_pushbutton.
function initLSM_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to initLSM_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global LSMTest;

LSMTest.hLSM = dabs.thorlabs.LSM();
LSMTest.hLSM.configureFrameAcquiredEvent('debugmessages',1);
LSMTest.LSMInitialized = 1;


names = fieldnames(handles);
for n=1:length(names);
    try
        eval(['set(handles.' names{n} ', ''Enable'', ''on'')']);
    catch e
        
    end
end

% set(handles.deleteLSM_pushbutton, 'Enable', 'on');
% set(handles.freeRunTest_pushbutton, 'Enable', 'on');
% set(handles.multiFrameTest_pushbutton, 'Enable', 'on');
% set(handles.singleFrameTest_pushbutton, 'Enable', 'on');

set(hObject, 'Enable', 'off');

% --- Executes on button press in deleteLSM_pushbutton.
function deleteLSM_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to deleteLSM_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global LSMTest;


LSMTest.LSMInitialized = 0;

names = fieldnames(handles);
for n=1:length(names);
    try
        eval(['set(handles.' names{n} ', ''Enable'', ''off'')']);
    catch e
        
    end
end

delete(LSMTest.hLSM);

%unloadlibrary('ThorPMT');
%unloadlibrary('ThorConfocal');

set(handles.initLSM_pushbutton, 'Enable', 'on');
%clear classes;
%LSMTestFig;

% set(handles.freeRunTest_pushbutton, 'Enable', 'off');
% set(handles.multiFrameTest_pushbutton, 'Enable', 'off');
% set(handles.singleFrameTest_pushbutton, 'Enable', 'off');

% --- Executes on button press in grabTest_pushbutton.
function grabTest_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to grabTest_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
LSMGrabTest;


% --- Executes on button press in diskLogEnable_checkbox.
function diskLogEnable_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to diskLogEnable_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of diskLogEnable_checkbox
global LSMTest;

LSMTest.hLSM.loggingEnable = logical(get(hObject,'Value'));


function circBufferSize_edit_Callback(hObject, eventdata, handles)
% hObject    handle to circBufferSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of circBufferSize_edit as text
%        str2double(get(hObject,'String')) returns contents of circBufferSize_edit as a double
global LSMTest;

LSMTest.hLSM.circBufferSize = str2double(get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function circBufferSize_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to circBufferSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in channel1Range_popupmenu.
function channel1Range_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to channel1Range_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns channel1Range_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from channel1Range_popupmenu
global LSMTest;

LSMTest.hLSM.inputChannelRange1 = get(hObject, 'Value');
fprintf(1, 'ch1= %d ch2=%d\n', LSMTest.hLSM.inputChannelRange1, LSMTest.hLSM.inputChannelRange2);

% --- Executes during object creation, after setting all properties.
function channel1Range_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to channel1Range_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in channel1Active_checkbox.
function channel1Active_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to channel1Active_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

setChannelsActive(hObject,1);

% % Hint: get(hObject,'Value') returns toggle state of channel1Active_checkbox
% global LSMTest;
% 
% LSMTest.hLSM.channelsActive = unique([1 LSMTest.hLSM.channelsActive]);
% %LSMTest.hLSM.setChannelActive(1, get(hObject, 'Value')); %Array identifying which channels are active, e.g. 1, [1 2], etc.

% --- Executes on button press in channel2Active_checkbox.
function channel2Active_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to channel2Active_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

setChannelsActive(hObject,2);

% Hint: get(hObject,'Value') returns toggle state of channel2Active_checkbox
% global LSMTest;
% 
% if get(hObject,'Value')
% LSMTest.hLSM.channelsActive = unique([2 LSMTest.hLSM.channelsActive]);
% %LSMTest.hLSM.setChannelActive(2, get(hObject, 'Value'));


function setChannelsActive(hObject,chanNumber)

global LSMTest;

if get(hObject,'Value')
    LSMTest.hLSM.channelsActive = unique([chanNumber LSMTest.hLSM.channelsActive]);
else
    LSMTest.hLSM.channelsActive = setdiff(LSMTest.hLSM.channelsActive, chanNumber);
end


function framesToAcquire_edit_Callback(hObject, eventdata, handles)
% hObject    handle to framesToAcquire_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of framesToAcquire_edit as text
%        str2double(get(hObject,'String')) returns contents of framesToAcquire_edit as a double
global LSMTest;

LSMTest.framesToAcquire = round(str2double(get(hObject,'String')));
LSMTest.hLSM.multiFrameCount = LSMTest.framesToAcquire;


% --- Executes during object creation, after setting all properties.
function framesToAcquire_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to framesToAcquire_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in channel2Range_popupmenu.
function channel2Range_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to channel2Range_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns channel2Range_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from channel2Range_popupmenu
global LSMTest;

LSMTest.hLSM.inputChannelRange2 = get(hObject, 'Value');
fprintf(1, 'ch1= %d ch2=%d\n', LSMTest.hLSM.inputChannelRange1, LSMTest.hLSM.inputChannelRange2);


% --- Executes during object creation, after setting all properties.
function channel2Range_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to channel2Range_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in averaging_popupmenu.
function averaging_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to averaging_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns averaging_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from averaging_popupmenu
global LSMTest;

contents = cellstr(get(hObject,'String'));
LSMTest.hLSM.averagingMode =  contents{get(hObject,'Value')}; %One of {'AVG_NONE', 'AVG_CUMULATIVE'};

% --- Executes during object creation, after setting all properties.
function averaging_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to averaging_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function framesToAverage_edit_Callback(hObject, eventdata, handles)
% hObject    handle to framesToAverage_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of framesToAverage_edit as text
%        str2double(get(hObject,'String')) returns contents of framesToAverage_edit as a double
global LSMTest;

LSMTest.hLSM.averagingNumFrames = str2double(get(hObject,'String')); %Number of frames to average, when averagingMode = 'AVG_CUMULATIVE'

% --- Executes during object creation, after setting all properties.
function framesToAverage_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to framesToAverage_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pixelsPerDim_edit_Callback(hObject, eventdata, handles)
% hObject    handle to pixelsPerDim_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pixelsPerDim_edit as text
%        str2double(get(hObject,'String')) returns contents of pixelsPerDim_edit as a double
global LSMTest;

pixelsPerDim = str2double(get(hObject,'String'));

set([LSMTest.axes1Hnd LSMTest.axes2Hnd],'XLim',[1 pixelsPerDim],'YLim',[1 pixelsPerDim]);
LSMTest.hLSM.pixelsPerDim = pixelsPerDim;

% --- Executes during object creation, after setting all properties.
function pixelsPerDim_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pixelsPerDim_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function fieldSize_edit_Callback(hObject, eventdata, handles)
% hObject    handle to fieldSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fieldSize_edit as text
%        str2double(get(hObject,'String')) returns contents of fieldSize_edit as a double
global LSMTest;

LSMTest.hLSM.fieldSize = str2double(get(hObject,'String')); %Value from 1-255 setting the field-size

% --- Executes during object creation, after setting all properties.
function fieldSize_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fieldSize_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in scanMode_popupmenu.
function scanMode_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to scanMode_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns scanMode_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from scanMode_popupmenu
global LSMTest;

contents = cellstr(get(hObject,'String'));
LSMTest.hLSM.scanMode = contents{get(hObject,'Value')}; %One of {'TWO_WAY_SCAN', 'FORWARD_SCAN', 'BACKWARD_SCAN'}

% --- Executes during object creation, after setting all properties.
function scanMode_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to scanMode_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bidiPhaseAlignment_edit_Callback(hObject, eventdata, handles)
% hObject    handle to bidiPhaseAlignment_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bidiPhaseAlignment_edit as text
%        str2double(get(hObject,'String')) returns contents of bidiPhaseAlignment_edit as a double
global LSMTest;

LSMTest.hLSM.bidiPhaseAlignment = str2double(get(hObject,'String')); %Value from -127-128 allowing bidi scan adjustment ('TWO_WAY_SCAN' mode)

% --- Executes during object creation, after setting all properties.
function bidiPhaseAlignment_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bidiPhaseAlignment_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function logFilePath_edit_Callback(hObject, eventdata, handles)
% hObject    handle to logFilePath_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of logFilePath_edit as text
%        str2double(get(hObject,'String')) returns contents of logFilePath_edit as a double
global LSMTest;

LSMTest.hLSM.loggingFilePath = get(hObject,'String');

% --- Executes during object creation, after setting all properties.
function logFilePath_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to logFilePath_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function logFileName_edit_Callback(hObject, eventdata, handles)
% hObject    handle to logFileName_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of logFileName_edit as text
%        str2double(get(hObject,'String')) returns contents of logFileName_edit as a double
global LSMTest;

LSMTest.hLSM.loggingFileName = get(hObject,'String');

% --- Executes during object creation, after setting all properties.
function logFileName_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to logFileName_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in freeRunTest_pushbutton.
function freeRunTest_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to freeRunTest_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
dabs.thorlabs.Demos.LSMFreeRunTest();


% --- Executes on button press in singleFrameTest_pushbutton.
function singleFrameTest_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to singleFrameTest_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
dabs.thorlabs.Demos.LSMSingleFrameTest();

% --- Executes on button press in multiFrameTest_pushbutton.
function multiFrameTest_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to multiFrameTest_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
dabs.thorlabs.Demos.LSMMultiFrameTest(); %TODO!!


% --- Executes on button press in stop_pushbutton.
function stop_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to stop_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global LSMTest;

LSMTest.hLSM.stop();


% --- Executes on button press in pmt1enable_checkbox.
function pmt1enable_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to pmt1enable_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of pmt1enable_checkbox
global LSMTest;

LSMTest.hLSM.hPMTModule.pmtEnable1 = get(hObject,'Value');

% --- Executes on button press in pmt2enable_checkbox.
function pmt2enable_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to pmt2enable_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of pmt2enable_checkbox
global LSMTest;

LSMTest.hLSM.hPMTModule.pmtEnable2 = get(hObject,'Value');


function pmt1gain_edit_Callback(hObject, eventdata, handles)
% hObject    handle to pmt1gain_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pmt1gain_edit as text
%        str2double(get(hObject,'String')) returns contents of pmt1gain_edit as a double
global LSMTest;

LSMTest.hLSM.hPMTModule.pmtGain1 = get(hObject,'Value')

% --- Executes during object creation, after setting all properties.
function pmt1gain_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmt1gain_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pmt2gain_edit_Callback(hObject, eventdata, handles)
% hObject    handle to pmt2gain_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pmt2gain_edit as text
%        str2double(get(hObject,'String')) returns contents of pmt2gain_edit as a double
global LSMTest;

LSMTest.hLSM.hPMTModule.pmtGain2 = get(hObject,'Value');

% --- Executes during object creation, after setting all properties.
function pmt2gain_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmt2gain_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure

figH = figure(1);
delete(figH);
figH = figure(2);
delete(figH);

delete(hObject);
