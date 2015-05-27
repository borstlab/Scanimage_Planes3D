function varargout = mainControls(varargin)
%MAINCONTROLS M-file for mainControls.fig
%      MAINCONTROLS, by itself, creates a new MAINCONTROLS or raises the existing
%      singleton*.
%
%      H = MAINCONTROLS returns the handle to a new MAINCONTROLS or the handle to
%      the existing singleton*.
%
%      MAINCONTROLS('Property','Value',...) creates a new MAINCONTROLS using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to mainControls_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      MAINCONTROLS('CALLBACK') and MAINCONTROLS('CALLBACK',hObject,...) call the
%      local function named CALLBACK in MAINCONTROLS.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mainControls

% Last Modified by GUIDE v2.5 13-Dec-2011 15:37:44

%% CHANGES
% VI041308A: Disallow external triggering for multi-slice acquisitions -- Vijay Iyer 4/13/2008
% VI043008A: Specify key /release/ callback as a function handle in order to take advantage of eventdata feature-- Vijay Iyer 4/30/2008
% VI091508A: Employ absoute value with scanAmplitudeX/Y to handle case where scan direction is reversed by using negative value -- Vijay Iyer 9/15/2008
% VI091508B: Restored etServoDelay and phaseSlider controls (tied to cusp delay) and added FinePhaseControl checkbox -- Vijay Iyer 9/15/08
% VI091608A: Handle newly added autoSave checkbox -- Vijay Iyer 9/16/2008
% VI091808A: Handle newly added minimum-zoom property -- Vijay Iyer 9/18/2008
% VI092808A: All zoom processing now runs through setZoom, which now calls updateZoom() -- Vijay Iyer 9/24/2008
% VI110608A: New implementation allowing scanOffset update to be optionally written to current INI file
% VI120108A: Abort current scan (if any) quietly when toggling line scan -- Vijay Iyer 12/01/08
% VI121908A: Handle removal of maxOffsetX/Y and maxAmplitudeX/Y parameters (and updateScanFOV function) -- Vijay Iyer 12/19/08
% VI121908B: Warn user before parking at scan center -- Vijay Iyer 12/19/08
% VI010609A: Eliminate updateZoomStrings(); this is now in updateZoom() -- Vijay Iyer 1/06/09
% VI010809A: Cache old zoom value and flag changes that may cause fill fraction (line period) to change -- Vijay Iyer 1/08/09
% VI010909A: Route all zoom factor changes through setZoom, which can handle either the zoom 'dial' controls or other zoom changes (FULL/pbAddCurrent/etc) -- Vijay Iyer 1/09/09
% VI011509A: (Refactoring) Remove explicit calls to setupAOData()/flushAOData(), as these are now called as part of setupDAQDevices_ConfigSpecific() -- Vijay Iyer 1/15/09
% VI011509B: Add increment/decrement buttons for servo delay, replacing slider. Allows update of servo delay in units of AI samples -- Vijay Iyer 1/15/09
% VI011609B: Handle conversion of state.acq.cuspDelay to state.internal/acq.servoDelay; servo delay now displayed/stored in time units -- Vijay Iyer 1/16/09
% VI012109A: msPerLine is now actually in milliseconds -- Vijay Iyer 1/21/09
% VI012809A: Don't change configurationChanged flag for change to line-scan checkbox
% VI021309A: Moved Set/Park Offset logic to alignGUI -- Vijay Iyer 2/13/09
% VI021809A: Use si_getrect() instead of getrect() -- Vijay Iyer 2/18/09
% VI021909B: User si_selectImageFigure() to select the image figure before user-interactive work -- Vijay Iyer 2/19/09
% VI030309A: Bypass si_selectImageFigure() if the callback is invoked from the dropdown menu of one of the figures -- Vijay Iyer 3/3/09
% VI030409A: Update current pbAddCurrent in popup menu following the addition of an pbAddCurrent -- Vijay Iyer 3/4/09
% VI030409B: Correctly pbBase the zoom value when the pbBase button is pressed -- Vijay Iyer 3/4/09
% VI042709A: Ignore rotation when selecting line scan with bidi scanning enabled -- Vijay Iyer 4/27/09
% VI043009A: For now, tether GUIs in all cases...don't allow (saved) repositinioning of tethered GUIs -- Vijay Iyer 4/30/09
% VI050509A: Correct pbAddCurrent zoom factor determination for case of non-square aspect ratios -- Vijay iyer 5/21/09
% VI052009A: (REFACTOR) All calls to setupDaqDevices_ConfigSpecifig() also call preallocateMemory() -- Vijay Iyer 5/21/09
% VI102609A: Use state.internal.scanAmplitudeX/Y in lieu of state.acq.scanAmplitudeX/Y, as the internal value is now used to represent the actual command voltage -- Vijay Iyer 10/26/09
% VI111609A: BUGFIX - genericCallback() call was missing from zoomhundreds slider -- Vijay Iyer 11/16/09
% VI111609B: BUGFIX - CenterOnSelection was not working without Image Processing toolbox. It now uses newly created si_getpt(), instead of getpts(). Also it uses si_selectImageFigure() now first to identify target figure, as with other graphical interaction tools. -- Vijay Iyer 11/16/09
% VI071310A: Use getPointsFromAxes()/getRectFromAxes() in lieu of getpts/getline/getrect -- Vijay Iyer 7/13/10
% VI092210A: Call through to preallocateMemory() when auto-save checkbox is toggled from this GUI -- Vijay Iyer 9/22/10
% VI092210A: The state.acq.acquiredData var is now a frame-indexed, reverse-chronological running buffer; changes made also have effect of displaying the most-recent, rather than first, frame acquired (the better behavior) -- Vijay Iyer 9/23/10
% VI092710A: Add fastConfigurationGUI -- Vijay Iyer 9/27/10
% VI100110A: Add userFunctionsGUI -- Vijay Iyer 10/1/10
% VI110310A: Remove dead code; state.acq.scanAmplitudeSlow no longer exists -- Vijay Iyer 11/3/10
% DEQ111910A: BUGFIX - Errors in zoom slider logic involving zoom factors > 100 -- David Earl 11/19/10
% VI041311A: BUGFIX - pbAddCurrent selection shift Fast/Slow parameters were calculated in voltage rather than angle, as now required -- Vijay Iyer 4/13/11
% VI042311A: BUGFIX - cblinescanenable/Center selection shift Fast/Slow parameters were calculated in voltage rather than angle, similar to ROI (VI041311A) -- Vijay Iyer 4/23/11
%
%% ***************************************************


% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @mainControls_OpeningFcn, ...
    'gui_OutputFcn',  @mainControls_OutputFcn, ...
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

% --- Executes just before mainControls is made visible.
function mainControls_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for mainControls
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

% UIWAIT makes mainControls wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = mainControls_OutputFcn(hObject, eventdata, handles)
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

% Hint: delete(hObject) closes the figure
scim_exit('prompt');


%| ABOUT CALLBACKS:
%| GUIDE automatically appends subfunction prototypes to this file, and
%| sets objects' callback properties to call them through the FEVAL
%| switchyard above. This comment describes that mechanism.
%|
%| Each callback subfunction declaration has the following form:
%| <SUBFUNCTION_NAME>(H, EVENTDATA, HANDLES, VARARGIN)
%|
%| The subfunction name is composed using the object's Tag and the
%| callback type separated by '_', e.g. 'slider2_Callback',
%| 'figure1_CloseRequestFcn', 'axis1_ButtondownFcn'.
%|
%| H is the callback object's handle (obtained using GCBO).
%|
%| EVENTDATA is empty, but reserved for future use.
%|
%| HANDLES is a structure containing handles of components in GUI using
%| tags as fieldnames, e.g. handles.figure1, handles.slider2. This
%| structure is created at GUI startup using GUIHANDLES and stored in
%| the figure's application data using GUIDATA. A copy of the structure
%| is passed to each callback.  You can store additional information in
%| this structure at GUI startup, and you can change the structure
%| during callbacks.  Call guidata(h, handles) after changing your
%| copy to replace the stored original so that subsequent callbacks see
%| the updates. Type "help guihandles" and "help guidata" for more
%| information.
%|
%| VARARGIN contains any extra arguments you have passed to the
%| callback. Specify the extra arguments by editing the callback
%| property in the inspector. By default, GUIDE sets the property to:
%| <MFILENAME>('<SUBFUNCTION_NAME>', gcbo, [], guidata(gcbo))
%| Add any extra arguments after the last argument, before the final
%| closing parenthesis.


function repeatsTotal_Callback(hObject, eventdata, handles)
% hObject    handle to repeatsTotal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of repeatsTotal as text
%        str2double(get(hObject,'String')) returns contents of repeatsTotal as a double
genericCallback(hObject);

function slicesTotal_Callback(hObject, eventdata, handles)
% hObject    handle to slicesTotal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of slicesTotal as text
%        str2double(get(hObject,'String')) returns contents of slicesTotal as a double
genericCallback(hObject);
updateNumberOfZSlices(hObject);


function framesTotal_Callback(hObject, eventdata, handles)
% hObject    handle to framesTotal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of framesTotal as text
%        str2double(get(hObject,'String')) returns contents of framesTotal as a double
genericCallback(hObject);
preallocateMemory();


function etIterationsTotal_Callback(hObject, eventdata, handles)
% hObject    handle to etIterationsTotal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etIterationsTotal as text
%        str2double(get(hObject,'String')) returns contents of etIterationsTotal as a double
genericCallback(hObject);


function baseName_Callback(hObject, eventdata, handles)
% hObject    handle to baseName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of baseName as text
%        str2double(get(hObject,'String')) returns contents of baseName as a double
genericCallback(hObject);

function fileCounter_Callback(hObject, eventdata, handles)
% hObject    handle to fileCounter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fileCounter as text
%        str2double(get(hObject,'String')) returns contents of fileCounter as a double
genericCallback(hObject);


function scanRotation_Callback(hObject, eventdata, handles)
% hObject    handle to scanRotation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of scanRotation as text
%        str2double(get(hObject,'String')) returns contents of scanRotation as a double
updateScanParameter(hObject);

% --- Executes on slider movement.
function scanRotationSlider_Callback(hObject, eventdata, handles)
% hObject    handle to scanRotationSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
updateScanParameter(hObject);

% --- Executes on button press in pbRoot.
function pbRoot_Callback(hObject, eventdata, handles)
% hObject    handle to pbRoot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hModel.roiGotoROI(handles.hModel.ROI_ROOT_ID);

function etNumAvgFramesSave_Callback(hObject, eventdata, handles)
% hObject    handle to etNumAvgFramesSave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etNumAvgFramesSave as text
%        str2double(get(hObject,'String')) returns contents of etNumAvgFramesSave as a double

genericCallback(hObject);


% --------------------------------------------------------------------
function varargout = focusButton_Callback(h, eventdata, handles, varargin)
% Stub for Callback of the uicontrol handles.focusButton.
global state gh
figure(gh.mainControls.figure1);
state.internal.whatToDo = 1;
executeFocusCallback(h);

% --------------------------------------------------------------------
function varargout = grabOneButton_Callback(h, eventdata, handles, varargin)
% Stub for Callback of the uicontrol handles.grabOneButton.
global state gh
figure(gh.mainControls.figure1);
state.internal.whatToDo = 2;
executeGrabOneCallback(h);

% --------------------------------------------------------------------
function varargout = startLoopButton_Callback(h, eventdata, handles, varargin)
% Stub for Callback of the uicontrol handles.startLoopButton.
global gh
figure(gh.mainControls.figure1);
executeStartLoopCallback(h);

% --------------------------------------------------------------------
function varargout = mainZoom_Callback(h, eventdata, handles, varargin)
genericCallback(h);
setScanProps(h);

% --------------------------------------------------------------------
function varargout = fullfield_Callback(h, eventdata, handles, varargin)
global gh state
setZoomValue(1); %VI010909A
%updateZoomStrings; %VI010609A
setScanProps(h);



%%%%%From 3.0%%%%
% updateGUIByGlobal('state.acq.zoomFactor');
% state.acq.zoomhundreds=0;
% state.acq.zoomtens=0;
% state.acq.zoomones=1;
% updateGUIByGlobal('state.acq.zoomhundreds');
% updateGUIByGlobal('state.acq.zoomtens');
% updateGUIByGlobal('state.acq.zoomones');
%%%%%%%%%%%%%%%%%%


% --------------------------------------------------------------------
function varargout = scanShiftSlow_Callback(h, eventdata, handles, varargin)
updateScanParameter(h);

% --------------------------------------------------------------------
function varargout = scanShiftFast_Callback(h, eventdata, handles, varargin)
updateScanParameter(h);


% --------------------------------------------------------------------
function varargout = right_Callback(h, eventdata, handles, varargin)
global state
state.acq.scanShiftFast = state.acq.scanShiftFast + 1 / state.acq.zoomFactor * state.acq.xstep;
if abs(state.acq.scanShiftFast) < .0001
    state.acq.scanShiftFast = 0;
end
%updateScanFOV; %VI062508A, VI121908A
updateGUIByGlobal('state.acq.scanShiftFast');
setScanProps(h);
updateRSPs();

% --------------------------------------------------------------------
function varargout = left_Callback(h, eventdata, handles, varargin)
global state
state.acq.scanShiftFast = state.acq.scanShiftFast - 1 / state.acq.zoomFactor * state.acq.xstep;
if abs(state.acq.scanShiftFast) < .0001
    state.acq.scanShiftFast = 0;
end
%updateScanFOV; %VI062508A, VI121908A
updateGUIByGlobal('state.acq.scanShiftFast');
setScanProps(h);
updateRSPs();

% --------------------------------------------------------------------
function varargout = down_Callback(h, eventdata, handles, varargin)
global state
state.acq.scanShiftSlow = state.acq.scanShiftSlow + 1 / state.acq.zoomFactor * state.acq.ystep;
if abs(state.acq.scanShiftSlow) < 0.0001
    state.acq.scanShiftSlow = 0;
end
%updateScanFOV; %VI062508A, VI121908A
updateGUIByGlobal('state.acq.scanShiftSlow');
setScanProps(h);
updateRSPs();

% --------------------------------------------------------------------
function varargout = up_Callback(h, eventdata, handles, varargin)
global state
state.acq.scanShiftSlow = state.acq.scanShiftSlow - 1 / state.acq.zoomFactor * state.acq.ystep;
if abs(state.acq.scanShiftSlow) < 0.0001
    state.acq.scanShiftSlow = 0;
end
%updateScanFOV; %VI062508A, VI121908A
updateGUIByGlobal('state.acq.scanShiftSlow');
setScanProps(h);
updateRSPs();

% --------------------------------------------------------------------
function varargout = zero_Callback(h, eventdata, handles, varargin)
global state
state.acq.scanShiftSlow = 0;
updateGUIByGlobal('state.acq.scanShiftSlow');
state.acq.scanShiftFast = 0;
updateGUIByGlobal('state.acq.scanShiftFast');
setScanProps(h);
updateRSPs();

% % --------------------------------------------------------------------
% function done = drawROISI(handle)
% global state
% done = 0;
%
% %%%VI021909A%%%%
% if ismember(handle,state.internal.GraphFigure) %VI030309A
%     hax = get(handle,'CurrentAxes'); %VI030309A
% else
%     hax = si_selectImageFigure();
%     if isempty(hax)
%         return;
%     end
% end
% %%%%%%%%%%%%%%%
%
% %%%VI041311A%%%
% %[hax, volts_per_pixelFast, volts_per_pixelSlow, sizeImage] = genericFigSelectionFcn(hax); %VI041311A: Removed %VI021909A
% sizeImage = [state.acq.pixelsPerLine  state.internal.storedLinesPerFrame]; %VI102209A
% %%%%%%%%%%%%%%%%
%
% pos=getRectFromAxes(hax, 'Cursor', 'crosshair', 'nomovegui', 1); %VI071310A %VI021809B
% if pos(3) == 0 || pos(4) == 0
%     return
% end
%
% %%%VI050509A%%%%%%%%%%%%%
% originalZoomFactor = state.acq.zoomFactor; %VI041311A
% roiZoomFactor = min(sizeImage(1:2) ./ fliplr(pos(3:4)));
% setZoomValue(ceil(state.acq.zoomFactor * roiZoomFactor));
% %setZoomValue(ceil(state.acq.zoomFactor*round(sizeImage(1)./pos(3)))); %VI010909A %VI050509A: Removed
% %%%%%%%%%%%%%%%%%%%%%%%%%e
%
% %%%VI010909A: Removed %%%%%%%%%%%%
% %state.acq.zoomFactor=ceil(state.acq.zoomFactor*round(sizeImage(1)./pos(3)));
% %updateGUIByGlobal('state.acq.zoomFactor');
% %updateZoomStrings; %VI010609A
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% centerX = (pos(1) + 0.5 * pos(3));
% centerY = (pos(2) + 0.5 * pos(4));
% state.acq.scanShiftFast = state.acq.scanShiftFast + ((state.init.scanAngularRangeReferenceFast * state.acq.scanAngleMultiplierFast)/originalZoomFactor) * (centerX-sizeImage(2)/2) / sizeImage(2); %VI041311A
% state.acq.scanShiftSlow = state.acq.scanShiftSlow + ((state.init.scanAngularRangeReferenceSlow * state.acq.scanAngleMultiplierSlow)/originalZoomFactor) * (centerY-sizeImage(1)/2) / sizeImage(1); %VI041311A
% updateGUIByGlobal('state.acq.scanShiftFast');
% updateGUIByGlobal('state.acq.scanShiftSlow');
% done = 1;

% --------------------------------------------------------------------
function pbBase_Callback(h, eventdata, handles, varargin)
global state;
state.hSI.roiLoadBaseConfig();

% -------------------------------------------------------------------
function varargout = ystep_Callback(h, eventdata, handles, varargin)
genericCallback(h);

% --------------------------------------------------------------------
function varargout = xstep_Callback(h, eventdata, handles, varargin)
genericCallback(h);

% % --------------------------------------------------------------------
% function varargout = showrotbox_Callback(h, eventdata, handles, varargin)
% global state gh
%
% %currentString=get(h,'String');
% pos=get(ancestor(h,'figure'), 'position'); %VI070308A -- replace get(h,'Parent') with ancestor(h,'figure')
% if get(h,'Value') %DEQ20110331: changing to togglebutton  strcmp(currentString, '>>')
%     set(h,'String', '<<');
%     pos(3) = 94;
%     set(ancestor(h, 'figure'), 'position', pos); %VI070308A -- replace get(h,'Parent') with gcbf
% else
%     set(h,'String', '>>');
%     pos(3) = 49;
%     set(ancestor(h, 'figure'), 'position', pos); %VI070308A -- replace get(h,'Parent') with gcbf
% end
% state.internal.showRotBox=get(h,'Value'); %get(h,'String');

% % --------------------------------------------------------------------
% function varargout = cbLineScanEnable_Callback(h, eventdata, handles, varargin)
% global state;
% state.hSICtl.updateModel(h,eventdata,handles);

% % --------------------------------------------------------------------
% function varargout = phaseSlider_Callback(h, eventdata, handles, varargin)
% genericCallback(h);

% %%%VI020709A: Removed %%%%%%%%%%%%%%%%%%%%%
% % --------------------------------------------------------------------
% function done=setLS(handle)
% global state gh
% % done=0;
% % setImagesToWhole;
% % if nargin<1
% %     axis=state.internal.axis(logical(state.acq.imagingChannel));
% %     image=state.internal.imagehandle(logical(state.acq.imagingChannel));
% %     axis=axis(1);
% %     image=image(1);
% % elseif ishandle(handle)
% %     ind=find(handle==state.internal.axis);
% %     if isempty(ind)
% %         return
% %     end
% %     axis=handle;
% %     image=state.internal.imagehandle(ind);
% % else
% %     ~ishandle(handle)
% %     return
% % end
% % fractionUsedXDirection=state.acq.fillFraction;
% % x=get(axis,'XLim');
% % y=get(axis,'YLim');
% % sizeImage=[y(2) round(state.acq.roiCalibrationFactor*x(2))];
% % volts_per_pixelX=((1/state.acq.zoomFactor)*2*fractionUsedXDirection*abs(state.acq.scanAmplitudeX))/sizeImage(2); %VI091508A
% % volts_per_pixelY=((1/state.acq.zoomFactor)*2*abs(state.acq.scanAmplitudeY))/sizeImage(1); %VI091508A
% % [xpt,ypt]=getline(axis);
% % slope=(ypt(2)-ypt(1))/(xpt(2)-xpt(1));
% % state.acq.scanRotation=state.acq.scanRotation-(180/pi*atan(slope));
% % updateGUIByGlobal('state.acq.scanRotation');
% %
% % centerX=.5*(xpt(1)+xpt(2));
% % centerY=.5*(ypt(1)+ypt(2));
% %
% % state.acq.scaleXShift=state.acq.scaleXShift+volts_per_pixelX*(centerX-x(2)/2);
% % state.acq.scaleYShift=state.acq.scaleYShift+volts_per_pixelY*(centerY-sizeImage(1)/2);
% % updateGUIByGlobal('state.acq.scaleXShift');
% % updateGUIByGlobal('state.acq.scaleYShift');
% %
% % done=1;


% --------------------------------------------------------------------
function varargout = abortCurrentAcq_Callback(h, eventdata, handles, varargin)
abortCurrent;

% --------------------------------------------------------------------
function varargout = zoomhundredsslider_Callback(h, eventdata, handles, varargin)
genericCallback(h); %VI111609A
setZoom(h);

% --------------------------------------------------------------------
function varargout = zoomtensslider_Callback(h, eventdata, handles, varargin)

global state
genericCallback(h);
if state.acq.zoomtens == 10 && state.acq.zoomhundreds < 9
    state.acq.zoomtens = 0;
    state.acq.zoomhundreds = state.acq.zoomhundreds + 1;
elseif state.acq.zoomtens == 10 && state.acq.zoomhundreds >= 9
    state.acq.zoomtens = 9;
elseif state.acq.zoomtens < 0 && state.acq.zoomhundreds >= 1 %DEQ111910A
    %elseif state.acq.zoomtens == -1 && state.acq.zoomhundreds == 1 %DEQ111910A: Replaced
    state.acq.zoomtens = 9;
    state.acq.zoomhundreds=state.acq.zoomhundreds-1;
    % DEQ111910A: REMOVED%%%
    % elseif state.acq.zoomtens == 0 && state.acq.zoomhundreds >= 1
    %     %state.acq.zoomones = 9; % DEQ111910A - a subtraction of 10 should never affect the ones digit
    %     %state.acq.zoomhundreds = 0;
    %     state.acq.zoomhundreds = state.acq.zoomhundreds - 1;
    %%%%%%%%%%%%%%%%%%%%%%%%
elseif state.acq.zoomtens == -1 && state.acq.zoomhundreds < 1
    state.acq.zoomtens = 0;
end
%%%VI010909A%%%%%%%%%%%%%%%%%%%
% updateGUIByGlobal('state.acq.zoomones');
% updateGUIByGlobal('state.acq.zoomtens');
% updateGUIByGlobal('state.acq.zoomhundreds');
setZoom(h);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --------------------------------------------------------------------
function varargout = zoomonesslider_Callback(h, eventdata, handles, varargin)
genericCallback(h);

global state
if state.acq.zoomones == 10 && state.acq.zoomtens < 9
    state.acq.zoomones = 0;
    state.acq.zoomtens = state.acq.zoomtens + 1;
elseif state.acq.zoomones == 10 && state.acq.zoomtens >= 9
    state.acq.zoomones = 0;
    state.acq.zoomtens = 0;
    state.acq.zoomhundreds = state.acq.zoomhundreds + 1; %DEQ111910A - this was just setting it to 1
elseif state.acq.zoomones < 0 && state.acq.zoomtens >= 1
    state.acq.zoomones=9;
    state.acq.zoomtens=state.acq.zoomtens-1;
elseif state.acq.zoomones < 0 && state.acq.zoomtens < 1 && state.acq.zoomhundreds >= 1
    state.acq.zoomones = 9;
    state.acq.zoomtens = 9;
    state.acq.zoomhundreds = state.acq.zoomhundreds - 1;
end
%%%VI010909A%%%%%%%%%%%
% updateGUIByGlobal('state.acq.zoomones');
% updateGUIByGlobal('state.acq.zoomtens');
% updateGUIByGlobal('state.acq.zoomhundreds');
setZoom(h);
%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on slider movement.
function zoomfracslider_Callback(h, eventdata, handles)
genericCallback(h);

global state
if state.acq.zoomfrac == 10 && state.acq.zoomones < 9
    state.acq.zoomfrac = 0;
    state.acq.zoomones = state.acq.zoomones + 1;
elseif state.acq.zoomfrac == 10 && state.acq.zoomones >= 9 && state.acq.zoomtens < 9
    state.acq.zoomfrac = 0;
    state.acq.zoomones = 0;
    state.acq.zoomtens = state.acq.zoomtens + 1;
elseif state.acq.zoomfrac == 10 && state.acq.zoomones >= 9 && state.acq.zoomtens >= 9
    state.acq.zoomfrac = 0;
    state.acq.zoomones = 0;
    state.acq.zoomtens = 0;
    if state.acq.zoomhundreds < 9
        state.acq.zoomhundreds = state.acq.zoomhundreds + 1;
    end
elseif state.acq.zoomfrac == -1
    if state.acq.zoomones > 0
        state.acq.zoomones = state.acq.zoomones - 1;
        state.acq.zoomfrac = 9;
    else
        if state.acq.zoomtens > 0
            state.acq.zoomfrac = 9;
            state.acq.zoomones = 9;
            state.acq.zoomtens = state.acq.zoomtens - 1;
        else
            if state.acq.zoomhundreds > 0
                state.acq.zoomfrac = 9;
                state.acq.zoomones = 9;
                state.acq.zoomtens = 9;
                state.acq.zoomhundreds = state.acq.zoomhundreds - 1;
            else
                state.acq.zoomfrac = 0;
            end
        end
    end
    
    
end

setZoom(h);

% --------------------------------------------------------------------
%Generic handler for zoom 'dial' controls
function setZoom(h)
global state gh

%VI010909A: Defer processing to setZoomValue()
setZoomValue(str2num([num2str(round(state.acq.zoomhundreds))...
    num2str(round(state.acq.zoomtens)) num2str(round(state.acq.zoomones)) '.' num2str(state.acq.zoomfrac)]));

%Effect the change on the scan parameters
setScanProps(h);

% % --------------------------------------------------------------------
% function varargout = shutterDelay_Callback(h, eventdata, handles, varargin)
% genericCallback(h);
% updateShutterDelay;
%
% % --------------------------------------------------------------------
% function varargout = syncToPhysiology_Callback(h, eventdata, handles, varargin)
% genericCallback(h);

% --------------------------------------------------------------------
function varargout = pbSetBase_Callback(h, eventdata, handles, varargin)
global state;
state.hSI.roiSetBaseConfig();

% % --------------------------------------------------------------------
% function varargout = addROI_Callback(h, eventdata, handles, varargin)
% addROI;
%
% % --------------------------------------------------------------------
% function varargout = roiSaver_Callback(h, eventdata, handles, varargin)
% gotoROI(h);

% %---------------------------------------------------------------------
% function addROI
% global state gh
% %updateMotorPosition;
% motorAction(@motorGetPosition);
% state.acq.roiList=[state.acq.roiList; [state.acq.scanShiftFast state.acq.scanShiftSlow state.acq.scanRotation...
%     state.acq.zoomFactor state.acq.zoomones state.acq.zoomtens state.acq.zoomhundreds ...
%     state.motor.absXPosition state.motor.absYPosition state.motor.absZPosition]];
% set(gh.mainControls.roiSaver, 'String', cellstr(num2str((1 : size(state.acq.roiList, 1))')));
% set(gh.mainControls.roiSaver, 'Value', size(state.acq.roiList, 1)); %VI030409A
% drawROIsOnFigure;
%
% % --------------------------------------------------------------------
% function varargout = dropROI_Callback(h, eventdata, handles, varargin)
% global gh state
% if isempty(state.acq.roiList)
%     return
% end
% str=get(gh.mainControls.roiSaver,'String');
% val=get(gh.mainControls.roiSaver,'Value');
% if ~isempty(str)
%     state.acq.roiList(val, :) = [];
%     set(gh.mainControls.roiSaver, 'Value', max(val - 1, 1));
%     set(gh.mainControls.roiSaver, 'String', cellstr(num2str((1 : size(state.acq.roiList, 1))')));
% end
% drawROIsOnFigure;
%
%
% % --------------------------------------------------------------------
% function varargout = backROI_Callback(h, eventdata, handles, varargin)
% global gh state
% str = get(gh.mainControls.roiSaver, 'String');
% if ~iscellstr(str)
%     return
% end
% val=get(gh.mainControls.roiSaver, 'Value');
% if val == 1
%     val = length(str);
% else
%     val = val - 1;
% end
% set(gh.mainControls.roiSaver, 'Value', val);
% gotoROI(h);
%
% % --------------------------------------------------------------------
% function varargout = nextROI_Callback(h, eventdata, handles, varargin)
% global gh state
% str = get(gh.mainControls.roiSaver, 'String');
% if ~iscellstr(str)
%     return
% end
% val = get(gh.mainControls.roiSaver, 'Value');
% if val == length(str)
%     val = 1;
% else
%     val = val + 1;
% end
% set(gh.mainControls.roiSaver, 'Value', val);
% gotoROI(h);

% --------------------------------------------------------------------
function varargout = snapShot_Callback(h, eventdata, handles, varargin)
global state
snapShot(state.acq.numberOfFramesSnap);

% --------------------------------------------------------------------
function varargout = numberOfFramesSnap_Callback(h, eventdata, handles, varargin)
genericCallback(h);

% --------------------------------------------------------------------
function varargout = centerOnSelection_Callback(h, eventdata, handles, varargin)
global state gh
buttonHandles = [gh.mainControls.grabOneButton gh.mainControls.focusButton gh.mainControls.startLoopButton];
if all(strcmpi(get(buttonHandles, 'Visible'), 'on'))
    setUndo;
    %%%VI111609B%%%%%%%%%%
    axis = si_selectImageFigure();
    if isempty(axis)
        return;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%
    done = centerOnSelection(axis); %VI111609B
    if done
        setScanProps(h);
        snapShot(1);
    end
else
    beep;
    disp('ROI selection is disabled while acquiring or focusing.');
end

% --------------------------------------------------------------------
function done=centerOnSelection(handle)
global state gh
done = 0;
setImagesToWhole;
if nargin < 1
    axis = state.internal.axis(logical(state.acq.imagingChannel));
    image = state.internal.imagehandle(logical(state.acq.imagingChannel));
    axis = axis(1);
    image = image(1);
elseif ishandle(handle)
    ind = find(handle == state.internal.axis);
    if isempty(ind)
        return
    end
    axis = handle;
    image = state.internal.imagehandle(ind);
else
    ~ishandle(handle)
    return
end
fractionUsedXDirection = state.acq.fillFraction;
x = get(axis,'XLim');
y = get(axis,'YLim');
sizeImage = [y(2) round(state.acq.roiCalibrationFactor * x(2))];
%volts_per_pixelFast = ((1 / state.acq.zoomFactor) * 2 * fractionUsedXDirection * abs(state.internal.scanAmplitudeFast)) / sizeImage(2); %VI042311A: Removed %VI102609A %VI091508A
%volts_per_pixelSlow = ((1 / state.acq.zoomFactor) * 2 * abs(state.internal.scanAmplitudeSlow)) / sizeImage(1); %VI042311A: Removed %VI102609A %VI091508A
[xpt, ypt] = getPointsFromAxes(axis, 'numberOfPoints', 1, 'Cursor', 'crosshair', 'nomovegui', 1); %VI071310A
if isempty(xpt)
    return
elseif length(xpt) > 1
    xpt = xpt(end);
    ypt = ypt(end);
end
centerX = (xpt);
centerY = (ypt);
% state.acq.scanShiftFast = state.acq.scanShiftFast + volts_per_pixelFast * (centerX - x(2) / 2); %VI042311A: Removed
% state.acq.scanShiftSlow = state.acq.scanShiftSlow + volts_per_pixelSlow * (centerY - sizeImage(1) / 2); %VI042311A: Removed
state.acq.scanShiftFast = state.acq.scanShiftFast + ((state.init.scanAngularRangeReferenceFast * state.acq.scanAngleMultiplierFast)/state.acq.zoomFactor) * (centerX-sizeImage(2)/2) / sizeImage(2); %VI042311A
state.acq.scanShiftSlow = state.acq.scanShiftSlow + ((state.init.scanAngularRangeReferenceSlow * state.acq.scanAngleMultiplierSlow)/state.acq.zoomFactor) * (centerY-sizeImage(1)/2) / sizeImage(1); %VI042311A
updateGUIByGlobal('state.acq.scanShiftFast');
updateGUIByGlobal('state.acq.scanShiftSlow');
done = 1;

return;

function configName_Callback(hObject, eventdata, handles)
% hObject    handle to configName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of configName as text
%        str2double(get(hObject,'String')) returns contents of configName as a double


% --------------------------------------------------------------------
function varargout = zeroRotate_Callback(h, eventdata, handles, varargin)
global state

state.acq.scanRotation=0;
updateGUIByGlobal('state.acq.scanRotation');
setScanProps(h);

updateRSPs();

return;

% % --------------------------------------------------------------------
% function varargout = undo_Callback(h, eventdata, handles, varargin)
% global state gh
%
% if ~isempty(state.acq.lastROIForUndo)
%     state.acq.scanShiftFast=state.acq.lastROIForUndo(1);
%     state.acq.scanShiftSlow=state.acq.lastROIForUndo(2);
%     state.acq.scanRotation=state.acq.lastROIForUndo(3);
%     %state.acq.zoomFactor=state.acq.lastROIForUndo(4); %VI010909A
%     setZoomValue(state.acq.lastROIForUndo(4)); %VI010909A
%     updateGUIByGlobal('state.acq.scanShiftSlow');
%     updateGUIByGlobal('state.acq.scanShiftFast');
%     updateGUIByGlobal('state.acq.zoomFactor'); %VI010909A
%     updateGUIByGlobal('state.acq.scanRotation');
%     setScanProps(h);
%     buttonHandles=[gh.mainControls.grabOneButton gh.mainControls.focusButton gh.mainControls.startLoopButton];
%     if all(strcmpi(get(buttonHandles,'Visible'),'on'))
%         snapShot(1);
%     end
% end
%
% return;

% --------------------------------------------------------------------
function tbExternalTrig_Callback(h, eventdata, handles)
global state

if isempty(state.acq.startTrigInputTerminal)
    disp('To enable external triggering, you must specify an input terminal in the ''Triggers'' dialog.');
    set(h,'Value',false);
else
    genericCallback(h); %VI123109A
end

%%%VI123109A: Removed%%%%%%%%
% %Disallow external trigger for multi-slice acqusitions (VI041308A)
% if state.acq.numberOfZSlices > 1
%     state.acq.externallyTriggered = 0;
%     updateGUIByGlobal('state.acq.externallyTriggered');
%     setStatusString('Ext trig not possible');
%     disp('External triggering not possible for multi-slice acquisitions');
% else
%     genericCallback(h);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
return;

% --------------------------------------------------------------------
function cbInfiniteFocus_Callback(h, eventdata, handles)
genericCallback(h);

% ------------------------(VI091508B)------------------------------------
function cbFinePhaseAdjust_Callback(hObject, eventdata, handles)

genericCallback(hObject); %VI011509A

%%%VI011509A: Removed%%%%%%%%%%
% global gh
%
% sliderStep = get(gh.mainControls.phaseSlider,'SliderStep');
%
% if get(hObject,'Value')
%     sliderStep(1) = .005;
% else
%     sliderStep(1) = .025;
% end
%
% set(gh.mainControls.phaseSlider,'SliderStep',sliderStep);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
return;

% --------------------------------------------------------------------
function tbShowAlignGUI_Callback(hObject, eventdata, handles)
genericCallback(hObject);

global state
if state.internal.showAlignGUI
    tetherGUIs('mainControls','alignGUI','rightcenter'); %VI043009A
    seeGUI('gh.alignGUI.figure1');
    set(hObject,'String', 'ALIGN <<');
else
    hideGUI('gh.alignGUI.figure1');
    set(hObject,'String', 'ALIGN >>');
end

return;

% --------------------------------------------------------------------
function tbShowConfigGUI_Callback(hObject, eventdata, handles)
genericCallback(hObject);

global state
if state.internal.showCfgGUI
    tetherGUIs('mainControls','configurationControls','righttop'); %VI043009A
    seeGUI('gh.configurationControls.figure1');
    set(hObject,'String', 'CFG <<');
else
    hideGUI('gh.configurationControls.figure1');
    set(hObject,'String', 'CFG >>');
end

return;

% --------------------------------------------------------------------
function tbFastConfig1_Callback(hObject, eventdata, handles)
loadFastConfig(hObject);
return;

% --------------------------------------------------------------------
function tbFastConfig2_Callback(hObject, eventdata, handles)
loadFastConfig(hObject);
return;

% --------------------------------------------------------------------
function tbFastConfig3_Callback(hObject, eventdata, handles)
loadFastConfig(hObject);
return;

% --------------------------------------------------------------------
function tbFastConfig4_Callback(hObject, eventdata, handles)
loadFastConfig(hObject);
return;

% --------------------------------------------------------------------
function tbFastConfig5_Callback(hObject, eventdata, handles)
loadFastConfig(hObject);
return;

% --------------------------------------------------------------------
function tbFastConfig6_Callback(hObject, eventdata, handles)
loadFastConfig(hObject);
return;

% --------------------------------------------------------------------
function exportedClocks_Callback(hObject, eventdata, handles)
global gh

set(gh.clockExportGUI.figure1, 'Visible', 'On');%TO091210B

return;


% --------------------------------------------------------------------
function fastConfigs_Callback(hObject, eventdata, handles)
global gh
set(gh.fastConfigurationGUI.figure1, 'Visible', 'On'); %VI092710A


% VI100110A-------------------------------------------------------------
function userFunctions_Callback(hObject, eventdata, handles)
global gh
%updateUserFunctionsGUI([],true);
set(gh.userFunctionsGUI.figure1, 'Visible', 'On');


% --------------------------------------------------------------------
function tbFastConfig1_ButtonDownFcn(hObject, eventdata, handles)
fastConfigButtonDownFcn(hObject);

% --------------------------------------------------------------------
function tbFastConfig2_ButtonDownFcn(hObject, eventdata, handles)
fastConfigButtonDownFcn(hObject);

% --------------------------------------------------------------------
function tbFastConfig3_ButtonDownFcn(hObject, eventdata, handles)
fastConfigButtonDownFcn(hObject);

% --------------------------------------------------------------------
function tbFastConfig4_ButtonDownFcn(hObject, eventdata, handles)
fastConfigButtonDownFcn(hObject);

% --------------------------------------------------------------------
function tbFastConfig5_ButtonDownFcn(hObject, eventdata, handles)
fastConfigButtonDownFcn(hObject);

% --------------------------------------------------------------------
function tbFastConfig6_ButtonDownFcn(hObject, eventdata, handles)
fastConfigButtonDownFcn(hObject);



function zoomfrac_Callback(hObject, eventdata, handles)


function userSettingsName_Callback(hObject, eventdata, handles)

% --- Executes on button press in tbCycleControls.
function tbCycleControls_Callback(h, eventdata, handles)
global gh state
if get(h,'Value')
    set(h,'String','CYC<<');    
    if isempty(state.internal.cycleGUIBottom)
        if strcmp(get(gh.configurationControls.figure1,'Visible'),'on')
            tetherGUIs('configurationControls', 'cycleGUI', 'righttop');
        else
            tetherGUIs('mainControls','cycleGUI','righttop');
        end
    end
    seeGUI('gh.cycleGUI.figure1');
else
    set(h,'String','CYC>>');
    hideGUI('gh.cycleGUI.figure1');
end


% --------------------------------------------------------------------
function pbAddCurrent_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.roiAdd('current');

% --- Executes on button press in pbAddSquare.
function pbAddSquare_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.roiAdd('square');

% --- Executes on button press in pbAddRectangle.
function pbAddRectangle_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.roiAdd('rect');

% --- Executes on button press in pbAddLine.
function pbAddLine_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.roiAdd('line');

% --- Executes on button press in pbAddPoint.
function pbAddPoint_Callback(hObject, eventdata, handles)
global state;
state.hSICtl.roiAdd('point');

% --- Executes on button press in tbToggleROI.
function tbToggleROI_Callback(hObject, eventdata, handles)
global gh state
if get(hObject,'Value')
    
    if isempty(state.internal.roiGUIBottom)
        % the configGUI is open, dock to it; otherwise, dock to the mainGUI
        if strcmp(get(gh.configurationControls.figure1, 'Visible'), 'on')
            tetherGUIs('configurationControls', 'roiGUI', 'righttop');
        else
            tetherGUIs('mainControls', 'roiGUI', 'righttop');
        end
    end
        
    seeGUI('gh.roiGUI.figure1');
else
    hideGUI('gh.roiGUI.figure1');
end

% --- Executes on button press in cbSnapOnAdd.
function cbSnapOnAdd_Callback(hObject, eventdata, handles)
% hObject    handle to cbSnapOnAdd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbSnapOnAdd
handles.hModel.roiSnapOnAdd = get(hObject,'Value');


% --- Executes on button press in cbGotoOnAdd.
function cbGotoOnAdd_Callback(hObject, eventdata, handles)
global gh;
handles.hModel.roiGotoOnAdd = get(hObject,'Value');
handles.hController.roiGotoOnAdd_Helper(get(hObject,'Value'),struct('mnuSnapOnAdd',gh.roiGUI.mnuSnapOnAdd)); % fabricate 'handles'

function edit47_Callback(hObject, eventdata, handles)
% hObject    handle to edit47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit47 as text
%        str2double(get(hObject,'String')) returns contents of edit47 as a double


% --- Executes during object creation, after setting all properties.
function edit47_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function etScanAngleMultiplierFast_Callback(hObject, eventdata, handles)
% hObject    handle to etScanAngleMultiplierFast (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etScanAngleMultiplierFast as text
%        str2double(get(hObject,'String')) returns contents of etScanAngleMultiplierFast as a double
updateScanParameter(hObject);
%updateScanAmplitude();
%applyConfigurationSettings();

function etScanAngleMultiplierSlow_Callback(hObject, eventdata, handles)
% hObject    handle to etScanAngleMultiplierSlow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etScanAngleMultiplierSlow as text
%        str2double(get(hObject,'String')) returns contents of etScanAngleMultiplierSlow as a double
updateScanParameter(hObject);
%updateScanAmplitude();
%applyConfigurationSettings();

% % disable LS mode, if necessary
% global state
% if state.hSI.lineScanEnable
%     state.hSI.lineScanEnable = false;
% end


function etRepeatPeriod_Callback(hObject, eventdata, handles)
% hObject    handle to etRepeatPeriod (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etRepeatPeriod as text
%        str2double(get(hObject,'String')) returns contents of etRepeatPeriod as a double
% Stub for Callback of most uicontrol handles
genericCallback(hObject);
global state
state.internal.secondsCounter=state.acq.repeatPeriod; %This should likely be in logic of an INI file callback...
updateGUIByGlobal('state.internal.secondsCounter');


function etCurrentROIID_Callback(hObject, eventdata, handles)
% hObject    handle to etCurrentROIID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etCurrentROIID as text
%        str2double(get(hObject,'String')) returns contents of etCurrentROIID as a double


% --- Executes on button press in pbAddPoints.
function pbAddPoints_Callback(hObject, eventdata, handles)
% hObject    handle to pbAddPoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state;
state.hSICtl.roiAdd('points');

% --- Executes on button press in pbAddCenterPoint.
function pbAddCenterPoint_Callback(hObject, eventdata, handles)
% hObject    handle to pbAddCenterPoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state;
state.hSICtl.roiAdd('centerpoint');

% --- Executes on button press in pbLoadUsr.
function pbLoadUsr_Callback(hObject, eventdata, handles)
% hObject    handle to pbLoadUsr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
openAndLoadUserSettings

% --- Executes on button press in pbSaveUsr.
function pbSaveUsr_Callback(hObject, eventdata, handles)
% hObject    handle to pbSaveUsr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveCurrentUserSettings;

% --- Executes on button press in pbLoadCfg.
function pbLoadCfg_Callback(hObject, eventdata, handles)
% hObject    handle to pbLoadCfg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
loadConfigurationFile();

% --- Executes on button press in pbSaveCfg.
function pbSaveCfg_Callback(hObject, eventdata, handles)
% hObject    handle to pbSaveCfg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
if state.internal.configurationChanged==1
    applyConfigurationSettings;
end
saveCurrentConfig();


% --- Executes on button press in pbSetSaveDir.
function pbSetSaveDir_Callback(hObject, eventdata, handles)
% hObject    handle to pbSetSaveDir (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setSavePath();

%% FILE MENU

% --------------------------------------------------------------------
function mnu_File_LoadUserSettings_Callback(hObject, eventdata, handles)
% hObject    handle to mnuLoadUsr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
openAndLoadUserSettings();

% --------------------------------------------------------------------
function mnu_File_SaveUserSettings_Callback(hObject, eventdata, handles)
% hObject    handle to mnuSaveUsr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveCurrentUserSettings();
% --------------------------------------------------------------------
function mnu_File_SaveUserSettingsAs_Callback(hObject, eventdata, handles)
% hObject    handle to mnuSaveUsrAs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveCurrentUserSettingsAs();
% --------------------------------------------------------------------
function mnu_File_LoadConfiguration_Callback(hObject, eventdata, handles)
% hObject    handle to mnuLoadCfg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
loadConfigurationFile;
% --------------------------------------------------------------------
function mnu_File_SaveConfiguration_Callback(hObject, eventdata, handles)
% hObject    handle to mnuSaveCfg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveCurrentConfig;
% --------------------------------------------------------------------
function mnu_File_SaveConfigurationAs_Callback(hObject, eventdata, handles)
% hObject    handle to mnuSaveCfgAs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveCurrentConfigAs;

% --------------------------------------------------------------------
function mnu_File_FastConfigurations_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_File_FastConfigurations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.fastConfigurationGUI.figure1');

% --------------------------------------------------------------------
function mnu_File_LoadCycle_Callback(hObject, eventdata, handles)
% hObject    handle to mnuLoadCycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
openAndLoadCycle;

% --------------------------------------------------------------------
function mnu_File_SaveCycle_Callback(hObject, eventdata, handles)
% hObject    handle to mnuSaveCycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveCurrentCycle;

% --------------------------------------------------------------------
function mnu_File_SaveCycleAs_Callback(hObject, eventdata, handles)
% hObject    handle to mnuSaveCycleAs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveCurrentCycleAs;

% % --------------------------------------------------------------------
% function mnu_File_DisplayCycle_Callback(hObject, eventdata, handles)
% % hObject    handle to mnuDisplayCycle (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% displayCycle;

% --------------------------------------------------------------------
function mnu_File_SetSavePath_Callback(hObject, eventdata, handles)
% hObject    handle to mnuSetSavePath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setSavePath;

% --------------------------------------------------------------------
function mnu_File_SaveLastAcqAs_Callback(hObject, eventdata, handles)
% hObject    handle to mnuSaveLastAcqAs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveLastAcquisitionAs;

% --------------------------------------------------------------------
function mnu_File_ExitScanImage_Callback(hObject, eventdata, handles)
% hObject    handle to mnuExitSI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
scim_exit();

% --------------------------------------------------------------------
function mnu_File_ExitMatlab_Callback(hObject, eventdata, handles)
% hObject    handle to mnuExitMatlab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
evalin('base','exit,exit')

%% SETTINGS MENU

% --------------------------------------------------------------------
function mnu_Settings_ExportedClocks_Callback(hObject, eventdata, handles)
% hObject    handle to mnuExportedClocks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.clockExportGUI.figure1');


% --------------------------------------------------------------------
function mnu_Settings_FastConfigurations_Callback(hObject, eventdata, handles)
% hObject    handle to mnuFastConfigs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.fastConfigurationGUI.figure1');

% --------------------------------------------------------------------
function mnu_Settings_UserFunctions_Callback(hObject, eventdata, handles)
% hObject    handle to mnuUserFcns (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.userFunctionsGUI.figure1');

% --------------------------------------------------------------------
function mnu_Settings_Channels_Callback(hObject, eventdata, handles)
% hObject    handle to mnuChannels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
openChannelGUI;
% --------------------------------------------------------------------
function mnu_Settings_Beams_Callback(hObject, eventdata, handles)
% hObject    handle to mnuBeams (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.laserFunctionPanel.figure1');

% --------------------------------------------------------------------
function mnu_Settings_Triggers_Callback(hObject, eventdata, handles)
% hObject    handle to mnuTriggers (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.triggerGUI.figure1');

% --------------------------------------------------------------------
function mnu_Settings_UserSettings_Callback(hObject, eventdata, handles)
% hObject    handle to mnuUserSettings (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.userPreferenceGUI.figure1');

%% VIEW MENU

% --------------------------------------------------------------------
function mnu_View_RaiseAllWindows_Callback(hObject, eventdata, handles)
scim_show();


% --------------------------------------------------------------------
function mnu_View_ShowAllWindows_Callback(hObject, eventdata, handles)
scim_show(true);

% --------------------------------------------------------------------
function mnuShowAllControlsForce_Callback(hObject, eventdata, handles)
scim_show('all',true);

% --------------------------------------------------------------------
function mnu_View_MainControls_Callback(hObject, eventdata, handles)
% hObject    handle to mnuMainControls (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.mainControls.figure1');


% --------------------------------------------------------------------
function mnu_View_ImageControls_Callback(hObject, eventdata, handles)
% hObject    handle to mnuImageControls (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.imageControls.figure1');


% --------------------------------------------------------------------
function mnu_View_PowerControls_Callback(hObject, eventdata, handles)
% hObject    handle to mnuPowerControls (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.powerControl.figure1');


% --------------------------------------------------------------------
function mnu_View_MotorControls_Callback(hObject, eventdata, handles)
% hObject    handle to mnuMotorControls (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.motorControls.figure1');


% --------------------------------------------------------------------
function mnu_View_ROIControls_Callback(hObject, eventdata, handles)
% hObject    handle to mnuROIControls (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.roiGUI.figure1');


% --------------------------------------------------------------------
function mnu_View_CycleModeControls_Callback(hObject, eventdata, handles)
% hObject    handle to mnuCycleControls (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.cycleGUI.figure1');


% --------------------------------------------------------------------
function mnu_View_PosnControls_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_View_PosnControls (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.positionGUI.figure1');


% --------------------------------------------------------------------
function mnu_View_Channel1Display_Callback(hObject, eventdata, handles)
% hObject    handle to mnuChannel1Display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
set(state.internal.GraphFigure(1),'visible', 'on');


% --------------------------------------------------------------------
function mnu_View_Channel2Display_Callback(hObject, eventdata, handles)
% hObject    handle to mnuChannel2Display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
set(state.internal.GraphFigure(2),'visible', 'on');


% --------------------------------------------------------------------
function mnu_View_Channel3Display_Callback(hObject, eventdata, handles)
% hObject    handle to mnuChannel3Display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
set(state.internal.GraphFigure(3),'visible', 'on');


% --------------------------------------------------------------------
function mnu_View_Channel4Display_Callback(hObject, eventdata, handles)
% hObject    handle to mnuChannel4Display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
set(state.internal.GraphFigure(4),'visible', 'on');


% --------------------------------------------------------------------
function mnu_View_Channel1MaxDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to mnuChannel1MaxDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
set(state.internal.MaxFigure(1),'visible', 'on');

% --------------------------------------------------------------------
function mnu_View_Channel2MaxDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to mnuChannel2MaxDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
set(state.internal.MaxFigure(2),'visible', 'on');


% --------------------------------------------------------------------
function mnu_View_Channel3MaxDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to mnuChannel3MaxDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
set(state.internal.MaxFigure(3),'visible', 'on');


% --------------------------------------------------------------------
function mnu_View_Channel4MaxDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to mnuChannel4MaxDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state
set(state.internal.MaxFigure(4),'visible', 'on');


% --------------------------------------------------------------------
function mnu_View_ChannelMergeDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to mnuChannelMergeDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state;
set(state.internal.MergeFigure,'Visible', 'on');

% --------------------------------------------------------------------
function cbAutoSave_Callback(hObject, eventdata, handles)
% hObject    handle to cbAutoSave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
genericCallback(hObject);
preallocateMemory(true);


%% CREATE FCNS

% --- Executes during object creation, after setting all properties.
function numberOfFramesSnap_CreateFcn(hObject, eventdata, handles)
% hObject    handle to numberOfFramesSnap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function scanRotationSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to scanRotationSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
generic_CreateFcn(hObject,eventdata,handles,[.9 .9 .9]);

% --- Executes during object creation, after setting all properties.
function scanRotation_CreateFcn(hObject, eventdata, handles)
% hObject    handle to scanRotation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function scanShiftSlow_CreateFcn(hObject, eventdata, handles)
% hObject    handle to scanShiftSlow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function scanShiftFast_CreateFcn(hObject, eventdata, handles)
% hObject    handle to scanShiftFast (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function ystep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ystep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function xstep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xstep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function zoomhundredsslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zoomhundredsslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
generic_CreateFcn(hObject,eventdata,handles,[.9 .9 .9]);

% --- Executes during object creation, after setting all properties.
function zoomtensslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zoomtensslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
generic_CreateFcn(hObject,eventdata,handles,[.9 .9 .9]);

% --- Executes during object creation, after setting all properties.
function zoomonesslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zoomonesslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
generic_CreateFcn(hObject,eventdata,handles,[.9 .9 .9]);

% --- Executes during object creation, after setting all properties.
function zoomhundreds_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zoomhundreds (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function zoomtens_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zoomtens (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function zoomones_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zoomones (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function zoomfracslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zoomfracslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function zoomfrac_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zoomfrac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% % --- Executes during object creation, after setting all properties.
% function cbLineScanEnable_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to cbLineScanEnable (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% generic_CreateFcn(hObject,eventdata,handles);

% --- Executes during object creation, after setting all properties.
function etScanAngleMultiplierFast_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etScanAngleMultiplierFast (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function etScanAngleMultiplierSlow_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etScanAngleMultiplierSlow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function configName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to configName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function repeatsDone_CreateFcn(hObject, eventdata, handles)
% hObject    handle to repeatsDone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function repeatsTotal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to repeatsTotal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function baseName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to baseName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function fileCounter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fileCounter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function slicesDone_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slicesDone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function framesTotal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to framesTotal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function framesDone_CreateFcn(hObject, eventdata, handles)
% hObject    handle to framesDone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function slicesTotal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slicesTotal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function userSettingsName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to userSettingsName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function etIterationsTotal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etIterationsTotal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function etIterationsDone_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etIterationsDone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function secondsCounter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to secondsCounter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function etRepeatPeriod_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etRepeatPeriod (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function etCurrentROIID_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etCurrentROIID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');

% --- Executes during object creation, after setting all properties.
function etNumAvgFramesSave_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etNumAvgFramesSave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
generic_CreateFcn(hObject,eventdata,handles,'white');


% --- Executes during object creation, after setting all properties.
function pnlAcqSettings_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pnlAcqSettings (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
generic_CreateFcn(hObject);

% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
generic_CreateFcn(hObject);

% --- Executes during object creation, after setting all properties.
function statusString_CreateFcn(hObject, eventdata, handles)
generic_CreateFcn(hObject);

% --- Executes during object creation, after setting all properties.
function left_CreateFcn(hObject, eventdata, handles)
% hObject    handle to left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,180,[0 0 1]));
end

% --- Executes during object creation, after setting all properties.
function right_CreateFcn(hObject, eventdata, handles)
% hObject    handle to right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,[],[0 0 1]));
end

% --- Executes during object creation, after setting all properties.
function down_CreateFcn(hObject, eventdata, handles)
% hObject    handle to down (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,90,[0 0 1]));
end

% --- Executes during object creation, after setting all properties.
function up_CreateFcn(hObject, eventdata, handles)
% hObject    handle to up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,270,[0 0 1]));
end


%% HELPER FUNCTIONS

function updateScanParameter(hObject)
genericCallback(hObject);
setScanProps(hObject);
return;

function fastConfigButtonDownFcn(hObject)
%Responds to right-click events on fast configuration toggle button

if ~get(hObject,'Value') %Don't respond to right-clicks when configuration is already on
    lastClickPoint = get(ancestor(hObject,'figure'),'CurrentPoint');
    panelPosn = get(ancestor(hObject,'uipanel'),'Position');
    buttonPosn = get(hObject,'Position');
    buttonPosn(1:2) = buttonPosn(1:2) + panelPosn(1:2);
    
    %Ensure that button was pressed /within/ the button's area, not surrounding 5 pixel region. This ensures it was a right-click
    if lastClickPoint(1) > buttonPosn(1) && lastClickPoint(1) < sum(buttonPosn([1 3])) ...
            && lastClickPoint(2) > buttonPosn(2) && lastClickPoint(2) < sum(buttonPosn([2 4]))
        
        configNumStr = deblank(get(hObject,'Tag'));
        configNum = str2num(configNumStr(end));
        loadFastConfig(configNum,true);
    end
end

return;

function toggleCheckedMenu(hObject)
if strcmp(get(hObject,'Checked'),'on')
    set(hObject,'Checked','off');
else
    set(hObject,'Checked','on');
end

% --- Executes on button press in pbLastLine.
function pbLastLine_Callback(hObject, eventdata, handles)
handles.hModel.roiGotoLastLine();

% --- Executes on button press in pbLastLineParent.
function pbLastLineParent_Callback(hObject, eventdata, handles)
handles.hModel.roiGotoLastLine(true);

function generic_CreateFcn(hObject,~,~,backgroundColor)
% Generic CreateFcn called by UI components--ensures that components have
% the correct background color.
%
% backgroundColor: an optional argument specifying a non-default background color to use for this component.

if nargin < 4 || isempty(backgroundColor)
    backgroundColor = get(0,'defaultUicontrolBackgroundColor');
end

if isprop(hObject,'BackgroundColor')
    if ispc && ~isequal(get(hObject,'BackgroundColor'),backgroundColor)
        set(hObject,'BackgroundColor',backgroundColor);
    end
elseif isprop(hObject,'Color')
    if ispc && ~isequal(get(hObject,'Color'),backgroundColor)
        set(hObject,'Color',backgroundColor);
    end
end


% --- Executes on button press in tbToggleLinescan.
function tbToggleLinescan_Callback(hObject, eventdata, handles)
% hObject    handle to tbToggleLinescan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
global state

if get(hObject,'Value') == 1
    state.acq.scanAngleMultiplierSlow = 0;
else
    state.acq.scanAngleMultiplierSlow = ...
        state.hSI.roiDataStructure(state.hSI.ROI_BASE_ID).RSPs.scanAngleMultiplierSlow;
end
updateGUIByGlobal('state.acq.scanAngleMultiplierSlow');

setScanProps();

%Call INI callbacks directly
updateRSPs();
updateScanAngleMultiplierSlow
            

