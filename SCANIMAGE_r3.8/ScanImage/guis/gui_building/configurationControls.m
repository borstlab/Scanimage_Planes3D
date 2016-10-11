function varargout = configurationControls(varargin)
%CONFIGURATIONCONTROLS M-file for configurationControls.fig
%      CONFIGURATIONCONTROLS, by itself, creates a new CONFIGURATIONCONTROLS or raises the existing
%      singleton*.
%
%      H = CONFIGURATIONCONTROLS returns the handle to a new CONFIGURATIONCONTROLS or the handle to
%      the existing singleton*.
%
%      CONFIGURATIONCONTROLS('Property','Value',...) creates a new CONFIGURATIONCONTROLS using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to configurationControls_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      CONFIGURATIONCONTROLS('CALLBACK') and CONFIGURATIONCONTROLS('CALLBACK',hObject,...) call the
%      local function named CALLBACK in CONFIGURATIONCONTROLS.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help configurationControls

% Last Modified by GUIDE v2.5 29-May-2012 15:36:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @configurationControls_OpeningFcn, ...
                   'gui_OutputFcn',  @configurationControls_OutputFcn, ...
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


% --- Executes just before configurationControls is made visible.
function configurationControls_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for configurationControls
handles.output = hObject;

%Handle KeyPressFcn bindings here (for now)
fig = openfig(mfilename, 'reuse');
set(fig, 'KeyPressFcn', @genericKeyPressFunction); %VI043008A
%%%%VI070308 -- Ensure all children respond to key presses, when they have the focus (for whatever reason)
kidControls = findall(fig, 'Type', 'uicontrol');
for i=1:length(kidControls)
    if ~strcmpi(get(kidControls(i), 'Style'), 'edit')
        set(kidControls(i), 'KeyPressFcn', @genericKeyPressFunction);
    end
end
%%%%%%

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes configurationControls wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = configurationControls_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
state.internal.showCfgGUI = 0;
updateGUIByGlobal('state.internal.showCfgGUI','Callback',1);


% function varargout = generic_Callback(h, eventdata, handles, varargin)
% % Stub for Callback of most uicontrol handles
% global state
% % state.internal.configurationChanged=1;
% % state.internal.configurationNeedsSaving=1;
% flagConfigChange; %VI092508A
% genericCallback(h);
% %%%VI121608A%%%%%
% % if any(strcmp(get(h,'tag'),{'etScanAngleMultiplierFast','etScanAngleMultiplierSlow'}))
% %     state.internal.aspectRatioChanged=1;
% % end
%%%%%%%%%%%%%%%%%

% function scanAngleMultiplier_Callback(h) 
% flagConfigChange; 
% genericCallback(h);
% %state.internal.aspectRatioChanged=1; %VI102609A
% 
% % --------------------------------------------------------------------
% function etScanAngleMultiplierFast_Callback(hObject, eventdata, handles)
% scanAngleMultiplier_Callback(hObject);
% 
% % --------------------------------------------------------------------
% function etScanAngleMultiplierSlow_Callback(hObject, eventdata, handles)
% global state;
% scanAngleMultiplier_Callback(hObject);
% 
% % disable LS mode, if necessary
% if state.hSI.lineScanEnable
%     state.hSI.lineScanEnable = false;
% end


function etFramesPerFile_Callback(hObject, eventdata, handles)
% hObject    handle to etFramesPerFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etFramesPerFile as text
%        str2double(get(hObject,'String')) returns contents of etFramesPerFile as a double
genericCallback(hObject);

global state gh
if state.acq.framesPerFileLock
	framesPerFile = get(hObject,'String');
	numFrames = get(gh.mainControls.etNumFrames,'String');
	if ~strcmp(framesPerFile,numFrames)
		state.acq.framesPerFile = str2double(numFrames);
		state.acq.framesPerFileGUI = state.acq.framesPerFile;
		updateGUIByGlobal('state.acq.framesPerFileGUI');
	end
end

% --- Executes on button press in cbFramesPerFileLock.
function cbFramesPerFileLock_Callback(hObject, eventdata, handles)
% hObject    handle to cbFramesPerFileLock (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbFramesPerFileLock
genericCallback(hObject);

global state
if state.acq.framesPerFileLock
	state.acq.framesPerFile = state.acq.numberOfFrames;
	state.acq.framesPerFileGUI = state.acq.numberOfFrames;
	updateGUIByGlobal('state.acq.framesPerFileGUI');
end


% --- Executes on button press in cbAutoSave.
function cbAutoSave_Callback(hObject, eventdata, handles)
% hObject    handle to cbAutoSave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbAutoSave
genericCallback(hObject);
preallocateMemory(true); %VI092210A


% --------------------------------------------------------------------
function cbBlankFlyback_Callback(hObject, eventdata, handles)
pockelsParam_Callback(hObject);

% --- Executes during object creation, after setting all properties.
function configurationName_Callback(hObject, eventdata, handles)

% --------------------------------------------------------------------
function etFillFracAdjust_Callback(hObject, eventdata, handles)
pockelsParam_Callback(hObject);

% --------------------------------------------------------------------
function varargout = pockelsParam_Callback(hObject, eventdata, handles, varargin)
global state 
flagConfigChange;
genericCallback(hObject);
state.init.eom.changed(1:state.init.eom.numberOfBeams) = 1; %VI102008A

% --------------------------------------------------------------------
function varargout = pixelsPerLine_Callback(h, eventdata, handles, varargin)
global state
flagConfigChange;
state.acq.pixelsPerLineGUI = get(h,'Value');
state.acq.pixelsPerLine = str2num(getMenuEntry(h, state.acq.pixelsPerLineGUI));
genericCallback(h);

% --------------------------------------------------------------------
function varargout = fastScanRadioX_Callback(h, eventdata, handles, varargin)
global gh state
flagConfigChange; 
genericCallback(h);
state.acq.fastScanningY = 0;
updateGUIByGlobal('state.acq.fastScanningY');

% --------------------------------------------------------------------
function varargout = fastScanRadioY_Callback(h, eventdata, handles, varargin)
global gh state
flagConfigChange; 
genericCallback(h);
state.acq.fastScanningX = 0;
updateGUIByGlobal('state.acq.fastScanningX');

% --------------------------------------------------------------------
function varargout = linesPerFrame_Callback(h, eventdata, handles, varargin)
global state
flagConfigChange;
genericCallback(h);

% --------------------------------------------------------------------
function pbApplyConfig_Callback(hObject, eventdata, handles)
global state

%Do the following to ensure that any queued callbacks are executed (MW Service Request  1-6D7KR)
hideGUI('gh.configurationControls.figure1');
drawnow; 
seeGUI('gh.configurationControls.figure1'); 

if state.internal.configurationChanged==1
    applyConfigurationSettings;
end

% --------------------------------------------------------------------
function tbShowAdvanced_Callback(hObject, eventdata, handles)
extraChars = 17; %DE112210A %how many extra characters to extend height of Fig
toggleAdvancedPanel(hObject,extraChars,'y');

% --------------------------------------------------------------------
function pbOptimizeScanAmp_Callback(hObject, eventdata, handles)
return;

% --------------------------------------------------------------------
function etScanDelay_Callback(hObject, eventdata, handles)
genericCallback(hObject);

%Update scan delay var (subject to all constraints) and its various internal representations
updateScanDelay();

%Update config array display
updateConfigZoomFactor();

%Flag configuration change, and trigger applyConfiguration if change occurred during FOCUS
flagConfigChange(true,hObject); %VI011910A

% --------------------------------------------------------------------
function pbDecScanDelay_Callback(hObject, eventdata, handles)
stepScanDelay('dec');

% --------------------------------------------------------------------
function pbIncScanDelay_Callback(hObject, eventdata, handles)
stepScanDelay('inc');

% --------------------------------------------------------------------
function etAcqDelay_Callback(hObject, eventdata, handles)
genericCallback(hObject);

global state gh

%Contrain/store current acq delay value
state.internal.acqDelayGUI = constrainAcqDelay(state.internal.acqDelayGUI,true);
updateGUIByGlobal('state.internal.acqDelayGUI');
state.acq.acqDelay = state.internal.acqDelayGUI * 1e-6;

%Update array (and its display)
state.internal.acqDelayArray(getConfigZoomFactor()) = state.internal.acqDelayGUI;
updateConfigZoomFactor();

%Flag configuration change, and allow it to occur during FOCUS
acqDelayCmdEffect = (state.init.eom.pockelsOn && state.acq.pockelsClosedOnFlyback) || state.acq.staircaseSlowDim; %VI020509A: Determine if acq delay change requires change in command waveform(s)
if acqDelayCmdEffect %VI011910A 
    flagConfigChange(true,hObject); %VI011910A %VI011910A %VI013109A, VI020509A
else
    flagConfigChange(true); %VI011910A: Don't pass object handle if no need to recompute focus
end

% --------------------------------------------------------------------
function pbDecAcqDelay_Callback(hObject, eventdata, handles)
stepAcqDelay('dec');

% --------------------------------------------------------------------
function pbIncAcqDelay_Callback(hObject, eventdata, handles)
stepAcqDelay('inc');

% --------------------------------------------------------------------
function cbFineAcqAdjust_Callback(hObject, eventdata, handles)
genericCallback(hObject);

% --------------------------------------------------------------------
function pmFillFrac_Callback(hObject, eventdata, handles)
genericCallback(hObject);

global state gh


%set(hObject,'Enable','off'); %VI011910B
if strcmp(get(gh.mainControls.focusButton, 'String'), 'ABORT') %focusing now
    stopFocus(true);
end


%Update fill frac var, and its various internal/GUI representations
[state.acq.fillFraction state.acq.msPerLine] = decodeFillFractionGUI(state.internal.fillFractionGUI);
state.internal.fillFractionGUIArray(getConfigZoomFactor()) = state.internal.fillFractionGUI;
updateGUIByGlobal('state.acq.msPerLine'); %VI022009A

%Update scan delay, as it may have to change based on new FF var
updateScanDelay();

%Update config array display
updateConfigZoomFactor();

%Update frame rate var/display
updateFrameRate(); %Vi043009A

%Flag configuration change, and allow it to occur during FOCUS
flagConfigChange(true,hObject); %VI011910A

% --------------------------------------------------------------------
function varargout = pmFillFracConfig_Callback(hObject, eventdata, handles) %VI092408A: Renamed to pmFillFractOpts
flagConfigChange; 
genericCallback(hObject);

global state gh

state.internal.fillFractionGUIArray(state.internal.configZoomFactor) = state.internal.fillFractionGUIConfig;

%Refresh scanDelayConfig given new fillFractionConfig value
updateGUIByGlobal('state.internal.scanDelayConfig','Callback',1);

%Update array/current display
updateConfigZoomFactor();
updateZoom();

% --------------------------------------------------------------------
function etAcqDelayConfig_Callback(hObject, eventdata, handles)
flagConfigChange; 
genericCallback(hObject);

global state
state.internal.acqDelayConfig = constrainAcqDelay(state.internal.acqDelayConfig,true);
updateGUIByGlobal('state.internal.acqDelayConfig');

state.internal.acqDelayArray(state.internal.configZoomFactor) = state.internal.acqDelayConfig;
updateConfigZoomFactor(); 
updateZoom();

% --------------------------------------------------------------------
function etScanDelayConfig_Callback(hObject, eventdata, handles)
flagConfigChange; 
genericCallback(hObject);

global state

%Ensure value satisfies discretization & FF constraints
[fillFraction, msPerLine] = decodeFillFractionGUI(state.internal.fillFractionGUIConfig);
state.internal.scanDelayConfig = constrainScanDelay(state.internal.scanDelayConfig, fillFraction, msPerLine);
updateGUIByGlobal('state.internal.scanDelayConfig');

state.internal.scanDelayArray(state.internal.configZoomFactor) = state.internal.scanDelayConfig;

%Update array/current display
updateConfigZoomFactor(); 
updateZoom();

% --------------------------------------------------------------------
function varargout = pmMsPerLine_Callback(h, eventdata, handles, varargin) %VI092408A: formerly etMsPerLine2_Callback in advancedconfigurationControls
global state
flagConfigChange; 
genericCallback(h);

%Update the Config Scan Delay values, which applies constraint if needed
lastConfigZoomFactor = state.internal.configZoomFactor;
for i=1:state.acq.baseZoomFactor
    state.internal.configZoomFactor = i;
    updateConfigZoomFactor();
    updateGUIByGlobal('state.internal.scanDelayConfig','Callback',1);
end
state.internal.configZoomFactor = lastConfigZoomFactor;
updateConfigZoomFactor();

% --------------------------------------------------------------------
function tbConfigChanged_Callback(hObject, eventdata, handles)

global gh 

if get(hObject,'Value')
    set(hObject,'BackgroundColor',[1 0 0]);
    set(gh.configurationControls.pbApplyConfig,'Enable','on');    
    set(gh.configurationControls.pbSaveConfig,'Enable' ,'on');
else
    set(hObject,'BackgroundColor',[0 1 0]);
    set(gh.configurationControls.pbApplyConfig,'Enable','off');
end


% --------------------------------------------------------------------
function etMaxFlybackRate_Callback(hObject, eventdata, handles)

global state

choice = questdlg(['The max flyback rate should be changed with caution. Are you sure of your change?' sprintf('\n') ...
    'NOTE: Changes to this parameter are not saved with configuration. To save change, edit your INI file (typ. standard.ini)'], 'WARNING', 'Yes', 'No', 'No');

switch choice
    case 'Yes'
        genericCallback(hObject);
    case 'No'
        updateGUIByGlobal('state.internal.maxFlybackRate'); %Restores past value
end


% --------------------------------------------------------------------
function etMinZoom_Callback(hObject, eventdata, handles)
global state;

flagConfigChange;
val = str2double(get(hObject,'String'));
if val < 1 && ~state.hSI.isSubUnityZoomAllowed
	set(hObject,'String','1');
	disp('Zoom factors less than 1 are currently disabled.');
    val = 1;
end

if state.acq.zoomFactor < val
	setZoomValue(val);
end
genericCallback(hObject);

% --------------------------------------------------------------------
function etBaseZoom_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);

global state 
%Reset config zoom factor to valid value, if needed
if state.internal.configZoomFactor > state.acq.baseZoomFactor
    state.internal.configZoomFactor = state.acq.baseZoomFactor;
end 
updateGUIByGlobal('state.internal.configZoomFactor','Callback',1);


% --------------------------------------------------------------------
function etConfigZoomFactor_Callback(hObject, eventdata, handles)
genericCallback(hObject);


% --------------------------------------------------------------------
function pbIncZoom_Callback(hObject, eventdata, handles)
incrementConfigZoomFactor(1);

% --------------------------------------------------------------------
function pbDecZoom_Callback(hObject, eventdata, handles)
incrementConfigZoomFactor(-1);

% --------------------------------------------------------------------
function pbAutoCompute_Callback(hObject, eventdata, handles)

% --------------------------------------------------------------------
function etShutterDelay_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);

% --------------------------------------------------------------------
function cbStaircaseSlowDim_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);

% --------------------------------------------------------------------
function cbDisableStriping_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);

%%%VI040509A: Removed %%%%%%%%%%
% % --------------------------------------------------------------------
% function cbIncreaseAORates_Callback(hObject, eventdata, handles)
% flagConfigChange();
% genericCallback(hObject);
% 
% global state
% if state.internal.increaseAORates 
%     state.acq.outputRate = state.internal.baseOutputRate * state.internal.featureAORateMultiplier;
% else
%     state.acq.outputRate = state.internal.baseOutputRate; 
% end
% updateGUIByGlobal('state.acq.outputRate');
% 
% %Actually handle update of channel AO rates in this callback
% %Do it here, rather than in general configuration handler, since in most cases, it isn't needed
% 
% %Update sample rate of Pockels channels
% if state.init.eom.pockelsOn %VI031009A
%     for i = 1:state.init.eom.numberOfBeams
%         setAOProperty(state.acq.dm, state.init.eom.pockelsCellNames{i}, 'SampleRate', state.acq.outputRate);
%     end
% end
% 
% %Update sample rate of mirror channels
% warning('off','daq:set:propertyChangeFlushedData');
% set([state.init.ao2 state.init.ao2F],'SampleRate',state.acq.outputRate)
% warning('on','daq:set:propertyChangeFlushedData');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --------------------------------------------------------------------
function pbSaveConfig_Callback(hObject, eventdata, handles)
pbApplyConfig_Callback(); %This will apply configuration settings, if they remain unapplied
saveCurrentConfig();

% --------------------------------------------------------------------
function pbSaveConfigAs_Callback(hObject, eventdata, handles)
pbApplyConfig_Callback(); %This will apply configuration settings, if they remain unapplied
saveCurrentConfigAs();

% --------------------------------------------------------------------
function pbLoadConfig_Callback(hObject, eventdata, handles)
loadConfigurationFile();


%%%VI040509A%%%%%%%%%%%%%%%%
% --------------------------------------------------------------------
function pmAIRate_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);

% --------------------------------------------------------------------
function pmAORate_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI041009A%%%%%%%%%%%%%%%
% --------------------------------------------------------------------
function cbAutoAIRate_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);

% --------------------------------------------------------------------
function cbAutoAORate_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);
%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%VI042709A%%%%%%%%%%%%%%%
% --------------------------------------------------------------------
function cbBidirectionalScan_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI102209A%%%%%%%%%%%%%%%
% --------------------------------------------------------------------
function cbFlybackFinalLine_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);

% --------------------------------------------------------------------
function cbDiscardFlybackLine_Callback(hObject, eventdata, handles)
flagConfigChange();
genericCallback(hObject);
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CREATE FCNS

% --- Executes during object creation, after setting all properties.
function linesPerFrame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linesPerFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function xScanOffset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xScanOffset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function yScanOffset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yScanOffset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function pixelsPerLine_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pixelsPerLine (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function configurationName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to configurationName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etLineDelay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etLineDelay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function pmMsPerLine_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmMsPerLine (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function outputRate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outputRate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function inputRate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to inputRate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function scanRotation_CreateFcn(hObject, eventdata, handles)
% hObject    handle to scanRotation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function msPerLine_CreateFcn(hObject, eventdata, handles)
% hObject    handle to msPerLine (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etBaseZoomFactorOld_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etBaseZoomFactorOld (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function pockelsCellLineDelay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pockelsCellLineDelay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etSawtoothBaseZoomFactor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etSawtoothBaseZoomFactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etNominalFillFrac_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etNominalFillFrac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function pockelsCellFillFraction_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pockelsCellFillFraction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function pockelsCellFillFractionSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pockelsCellFillFractionSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function etShutterDelay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etShutterDelay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etSamplesPerLine_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etSamplesPerLine (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etPixelTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPixelTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etBinFactor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etBinFactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etFrameRate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etFrameRate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etFramesPerFile_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etFramesPerFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etScanDelay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etScanDelay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etAcqDelay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etAcqDelay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function pmFillFrac_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmFillFrac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etMsPerLine_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etMsPerLine (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function pmAIRate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmAIRate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function pmAORate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmAORate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etMsPerLineConfig_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etMsPerLineConfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etMinZoom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etMinZoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etConfigZoomFactor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etConfigZoomFactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etScanDelayConfig_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etScanDelayConfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function pmFillFracConfig_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmFillFracConfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etBaseZoom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etBaseZoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etAcqDelayConfig_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etAcqDelayConfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etMaxFlybackRate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etMaxFlybackRate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etMsPerLine2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etMsPerLine2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etFillFracAdjust_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etFillFracAdjust (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

%% HELPERS
function incrementConfigZoomFactor(increment)

global state

newval = state.internal.configZoomFactor + increment;
if newval >= 1 && newval <= state.acq.baseZoomFactor
    state.internal.configZoomFactor = newval;
    updateGUIByGlobal('state.internal.configZoomFactor','Callback',1);
end


% --------------------------------------------------------------------
function stepAcqDelay(incOrDec)
global state

incVal = computeAcqDelayIncrement();

switch(lower(incOrDec))
    case 'inc'
        state.internal.acqDelayGUI = constrainAcqDelay(state.internal.acqDelayGUI + incVal,state.internal.fineAcqDelayAdjust,@floor);
    case 'dec'
        state.internal.acqDelayGUI = constrainAcqDelay(state.internal.acqDelayGUI - incVal,state.internal.fineAcqDelayAdjust,@ceil);
    otherwise
        error('Argument should be either ''inc'' or ''dec''');               
end      

%state.acq.servoDelay = state.internal.servoDelay * state.acq.msPerLine * 1e3; %Store/display value in microseconds %VI012109A
updateGUIByGlobal('state.internal.acqDelayGUI','Callback',1);


% --------------------------------------------------------------------
function stepScanDelay(incOrDec)
global state

incVal = state.internal.minAOPeriodIncrement * 1e6; %VI041209A: Allow scan delay increments in single AO sample increments

switch (lower(incOrDec))
    case 'inc'
        state.internal.scanDelayGUI = state.internal.scanDelayGUI + incVal;
    case 'dec'
        state.internal.scanDelayGUI = state.internal.scanDelayGUI - incVal;
end
updateGUIByGlobal('state.internal.scanDelayGUI','Callback',1);


% --------------------------------------------------------------------
function updateScanDelay()
global state

if ~state.acq.bidirectionalScan
    %Constrain scan delay value to proper increments/values, accounting for current Fill Fraction & Ms/Line
    state.internal.scanDelayGUI = constrainScanDelay(state.internal.scanDelayGUI, state.acq.fillFraction, state.acq.msPerLine);
    
    %Update GUI and non GUI scanDelay representations
    updateGUIByGlobal('state.internal.scanDelayGUI');
    state.acq.scanDelay = state.internal.scanDelayGUI * 1e-6;
    state.internal.scanDelayArray(getConfigZoomFactor()) = state.internal.scanDelayGUI;
end

% --------------------------------------------------------------------
function scanDelayOut = constrainScanDelay(scanDelayIn, fillFraction, msPerLine)
global state 

%Constrain scan delay value to proper increments/values, accounting for current Fill Fraction & Ms/Line
maxScanDelay = (1 - fillFraction) * msPerLine * 1e3;
scanDelayIncrement = state.internal.minAOPeriodIncrement * 1e6; %VI032409B: Allow steps of one AO sampling period (using minimum supported AO rate)

discretizedScanDelayGUI = round(scanDelayIn  / scanDelayIncrement) * scanDelayIncrement;
maxScanDelay = floor(maxScanDelay  / scanDelayIncrement) * scanDelayIncrement;

scanDelayOut = min(discretizedScanDelayGUI, maxScanDelay);


return;

% --------------------------------------------------------------------
function configZoomFactor = getConfigZoomFactor()
global state
configZoomFactor = min(state.acq.zoomFactor,state.acq.baseZoomFactor); %VI021809A

% --------------------------------------------------------------------
function flagConfigChange(allowDuringFocus, recomputeFocusControlHandle)  
global state gh

if nargin < 1
    allowDuringFocus = false;
end

recomputeDuringFocus = (nargin > 1) && ishandle(recomputeFocusControlHandle);  %VI011910A

%setConfigurationNeedsSaving(); %VI020209A

focusingNow = strcmp(get(gh.mainControls.focusButton, 'String'), 'ABORT');
if focusingNow
    if allowDuringFocus 
        if recomputeDuringFocus
            setScanProps(recomputeFocusControlHandle); %VI011910A %VI013109A: this will stop and restart focus
            %stopAndRestartFocus(); %VI013109A
        end
        return;
    else
        abortFocus(); 
    end
end

setConfigurationChanged(); %VI100710A

%%%VI100710A: Removed%%%%%%
% %Set Configuration Changed flag
% state.internal.configurationChanged=1;
% set(gh.configurationControls.pbApplyConfig,'Enable','on','ForegroundColor',[0 .5 0]);
% turnOffExecuteButtons('state.internal.configurationChanged');
% 
% setConfigurationNeedsSaving(); %VI020209A
%%%%%%%%%%%%%%%%%%%%%%%%%%%
