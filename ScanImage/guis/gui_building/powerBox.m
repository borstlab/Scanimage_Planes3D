function varargout = powerBox(varargin)
% POWERBOX M-file for powerBox.fig
%      POWERBOX, by itself, creates a new POWERBOX or raises the existing
%      singleton*.
%
%      H = POWERBOX returns the handle to a new POWERBOX or the handle to
%      the existing singleton*.
%
%      POWERBOX('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in POWERBOX.M with the given input arguments.
%
%      POWERBOX('Property','Value',...) creates a new POWERBOX or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before powerBox_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to powerBox_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help powerBox

% Last Modified by GUIDE v2.5 17-Jan-2009 14:35:07

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @powerBox_OpeningFcn, ...
    'gui_OutputFcn',  @powerBox_OutputFcn, ...
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

end

% --- Executes just before powerBox is made visible.
function powerBox_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to powerBox (see VARARGIN)

% Choose default command line output for powerBox
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes powerBox wait for user response (see UIRESUME)
% uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = powerBox_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end

% --------------------------------------------------------------------
function cbShowBox_Callback(hObject, eventdata, handles)
genericCallback(hObject);

global state gh
state.init.eom.showBoxArray(state.init.eom.powerBox.beamMenu) = state.init.eom.showBox;

%%%VI111108A: No longer needed
%if state.init.eom.showBox == 1
%     %Display the powerbox, when in use.
%     children = get(gh.powerControl.Settings, 'Children');
%     index = getPullDownMenuIndex(gh.powerControl.Settings, 'Show Power Box');
%     checked = get(children(index), 'Checked');
%
%     if ~strcmpi(checked, 'On')
%         resizePowerControlFigure;
%     end
%end
%%%%%%%%%%%%%%%

%Check whether a box has been selected
if size(state.init.eom.boxHandles, 1) < state.init.eom.powerBox.beamMenu & state.init.eom.showBox

    if state.init.eom.autoSelectFullWidthPowerBox %% & state.init.eom.powerBox.beamMenu == state.init.eom.scanLaserBeam (VI102008B)
        createFullWidthPowerBox;

    else
        %beep; %VI070708A: comment out
        fprintf(2, 'WARNING: Could not enable powerbox, no box selected.\n');%TO21804d  %VI070708A: Call a warning, instead of error
        %%%VI11108A%%%%%%%%%
        state.init.eom.showBox = 0;
        updateGUIByGlobal('state.init.eom.showBox');
        %set(h, 'Value', 0);
        %%%%%%%%%%%%%%%%
        return
    end
end

state.init.eom.changed(state.init.eom.powerBox.beamMenu) = 1;

if size(state.init.eom.boxHandles, 1) < state.init.eom.powerBox.beamMenu
    recth = [];%Doesn't the above `if` statement preclude this possibility? -- Tim 2/18/04
else
    recth = state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, ishandle(state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, :)));
end

%I think this section is redundant and unneeded
if (isempty(recth) | (state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, :) == 0)) & state.init.eom.showBox
    if state.init.eom.autoSelectFullWidthPowerBox %% & state.init.eom.powerBox.beamMenu == state.init.eom.scanLaserBeam (VI102008B)
        createFullWidthPowerBox;
    else
        %beep; %VI070708A: Comment out
        %%%VI11108A%%%%%%%%%
        state.init.eom.showBox = 0;
        updateGUIByGlobal('state.init.eom.showBox');
        %set(h, 'Value', 0);
        %%%%%%%%%%%%%%%%
        fprintf(2, 'WARNING: Can not enable powerbox, no box selected.\n');%TO21804d  %VI070708A: Call a warning, instead of error
        return;
    end
end

if state.init.eom.showBox
    set(recth,'Visible','On');
else
    set(recth,'Visible','Off');
end

%updatePowerBoxStrings; %VI020809A

%TO21804b: Tightly couple the powerbox checkbox and the UncagingPulseImporter's 'enable' button. - Tim O'Connor 2/18/04
if ~ismember(1, state.init.eom.showBoxArray) & state.init.eom.uncagingPulseImporter.enabled
    if state.init.eom.uncagingPulseImporter.coupleToPowerBoxErrors

        %Turn off the UncagingPulseImporter, if there are no powerboxes.
        state.init.eom.uncagingPulseImporter.enabled = 0;
        updateGUIByGlobal('state.init.eom.uncagingPulseImporter.enabled');
        set(gh.uncagingPulseImporter.enableToggleButton, 'ForegroundColor', [0 .6 0]);
        set(gh.uncagingPulseImporter.enableToggleButton, 'String', 'Enable');

        fprintf(2, 'WARNING: The UncagingPulseImporter was enabled when all powerboxes became deselected.\n         The UncagingPulseImporter has been automatically disabled.\n');

    else

        fprintf(2, 'WARNING: The UncagingPulseImporter was enabled when all powerboxes became deselected.\n         The UncagingPulseImporter will remain inactive until a powerbox is enabled.\n');

    end
end

end

% --------------------------------------------------------------------
function pmBeamMenu_Callback(hObject, eventdata, handles)
genericCallback(hObject);

global state
updatePowerBoxGUI(state.init.eom.powerBox.beamMenu);

end

% --------------------------------------------------------------------
function etBoxPowerOn_Callback(hObject, eventdata, handles)
genericCallback(hObject);

global state gh

%Added to allow power box to work in mW. -- Tim O'Connor TO21804a
conversion = 1;
if get(gh.powerControl.mW_radioButton, 'Value') == get(gh.powerControl.mW_radioButton, 'Max')    %in mW

    conversion = (getfield(state.init.eom, ['powerConversion' num2str(state.init.eom.powerBox.beamMenu)]) * ...
        state.init.eom.maxPhotodiodeVoltage(state.init.eom.powerBox.beamMenu) * .01);

    state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu) = round(1 / conversion * state.init.eom.boxPower);
else
    state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu) = round(state.init.eom.boxPower);
end

%Make sure it's within bounds.
if state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu) > 100
    state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu) = 100;
    state.init.eom.boxPower = conversion * state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu);
elseif state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu) < state.init.eom.min
    state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu) = state.init.eom.min(state.init.eom.powerBox.beamMenu);
    state.init.eom.boxPower = conversion * state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu);
end

state.init.eom.changed(state.init.eom.powerBox.beamMenu) = 1;

%Display the rounded off figure.
state.init.eom.boxPower = round(state.init.eom.boxPower);
updateGUIByGlobal('state.init.eom.boxPower');

%updatePowerBoxStrings; %VI020809A

end

% --------------------------------------------------------------------
function etBoxPowerOff_Callback(hObject, eventdata, handles)

genericCallback(hObject);
global state gh

%Added to allow power box to work in mW. -- Tim O'Connor TO21804a
conversion = 1;
if get(gh.powerControl.mW_radioButton, 'Value') == get(gh.powerControl.mW_radioButton, 'Max')    %in mW

    conversion = (getfield(state.init.eom, ['powerConversion' num2str(state.init.eom.powerBox.beamMenu)]) * ...
        state.init.eom.maxPhotodiodeVoltage(state.init.eom.powerBox.beamMenu) * .01);

    state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu) = round(1 / conversion * state.init.eom.boxPowerOff);
else
    state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu) = round(state.init.eom.boxPowerOff);
end

%Make sure it's within bounds.
if state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu) > 100
    state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu) = 100;
    state.init.eom.boxPowerOff = conversion * state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu);
elseif state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu) < state.init.eom.min
    state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu) = state.init.eom.min(state.init.eom.powerBox.beamMenu);
    state.init.eom.boxPowerOff = conversion * state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu);
end

state.init.eom.changed(state.init.eom.powerBox.beamMenu) = 1;

%Display the rounded off figure.
state.init.eom.boxPowerOff = round(state.init.eom.boxPowerOff);
updateGUIByGlobal('state.init.eom.boxPowerOff');

%updatePowerBoxStrings; %VI020809A
end


% --------------------------------------------------------------------
function cbLockBoxOnToMax_Callback(hObject, eventdata, handles)
genericCallback(hObject);

global state gh

if state.init.eom.lockBoxOnToMax
    set(gh.powerBox.etBoxPowerOn,'Enable','off');
    state.init.eom.boxPowerArray = state.init.eom.maxPower;
    state.init.eom.boxPower = state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu);
    updateGUIByGlobal('state.init.eom.boxPower');
else
    set(gh.powerBox.etBoxPowerOn,'Enable','on');
end
end

% --------------------------------------------------------------------
function cbLockBoxOffToMin_Callback(hObject, eventdata, handles)
genericCallback(hObject);

global state gh

if state.init.eom.lockBoxOffToMin
    set(gh.powerBox.etBoxPowerOff,'Enable','off');
    state.init.eom.boxPowerOffArray = state.init.eom.min;
    state.init.eom.boxPowerOff = state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu);
    updateGUIByGlobal('state.init.eom.boxPowerOff');
else
    set(gh.powerBox.etBoxPowerOff,'Enable','on');
end
end

% --------------------------------------------------------------------
function pbSelectPowerBox_Callback(hObject, eventdata, handles)
global state gh;

buttonHandles=[gh.mainControls.grabOneButton gh.mainControls.focusButton gh.mainControls.startLoopButton];
if all(strcmpi(get(buttonHandles,'Visible'),'on'))
    %%%VI021909A%%%%%%%
    ax = si_selectImageFigure();
    if isempty(ax)
        return;
    end
    %%%%%%%%%%%%%%%%%%%
    done=drawPowerBox(ax); %VI021909A
    if done
        setScanProps(h);
        snapShot(1);
    end
else
    beep;
    disp('ROI selection is disabled while acquiring or focusing.');
end
state.init.eom.changed(state.init.eom.powerBox.beamMenu)=1;
end

% --------------------------------------------------------------------
function cbConstrainBox_Callback(hObject, eventdata, handles)
global state gh;

genericCallback(hObject);
state.init.eom.constrainBoxToLine(state.init.eom.powerBox.beamMenu) = state.init.eom.powerBoxUncagingConstraint;

if state.init.eom.powerBoxUncagingConstraint & state.init.eom.showBox
    %Can't use powerBoxButtonDownFcn because it's too vague when choosing the right
    %graphics object works with.
    %Try to locate the (now properly tagged) object.
    fig = get(0,'CurrentFigure');
    obj = findobj('Tag', sprintf('PowerBox%s', num2str(state.init.eom.powerBox.beamMenu)));
    if length(obj) > 1
        obj = obj(1);%Hope it's always the first one...???
    end
    %Change the coordinates.
    pos = get(obj, 'Position');
    oldHeight = pos(4); %VI070908A
    if pos(4) < 0
        pos(4) = -1;
    elseif pos(4) > 0
        pos(4) = 1;
    end

    %Updated to force it to overlap only 1 line on the image. -- Tim O'Connor 6/16/04 TO061604a
    pos = round(pos);

    %Save the new coordinates.
    imsize = [state.acq.pixelsPerLine  state.internal.storedLinesPerFrame]; %VI102209A
    state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, :) = pos;
    state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, [1 3]) = pos([1 3]) ./ imsize(1);
    state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, [2 4]) = pos([2 4]) ./ imsize(2);
    %Update all the channels.
    for j = 1 : state.init.maximumNumberOfInputChannels
        if abs(oldHeight) > 1 %(VI070908A) Only add half-pixel correction when first constraining to line, not subsequent times
            %Added in a half pixel correction (positive in X, negative in Y), to get the box to directly overlap the signal. -- TimO'Connor 6/16/04 - TO061604b
            set(state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, j), 'Position', [(pos(1) + .5) (pos(2) - .5) pos(3) pos(4)]);
        else
            set(state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, j), 'Position', pos);
        end
    end
end
end

function etBoxWidth_Callback(hObject, eventdata, handles)
global state;

genericCallback(hObject);

if ~state.init.eom.showBox
    return;
end

%Try to locate the (now properly tagged) object.
fig = get(0,'CurrentFigure');
obj = findobj('Tag', sprintf('PowerBox%s', num2str(state.init.eom.powerBox.beamMenu)));
if isempty(obj)
    obj = state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, :);
end
if length(obj) > 1
    obj = obj(1);%Hope it's always the first one...???
end

%Change the coordinates.
pos = get(obj, 'Position');

if state.init.eom.boxWidth < 0
    state.init.eom.boxWidth = 0;
elseif state.init.eom.boxWidth  + (state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, 1) ... %VI012109A
        * state.acq.msPerLine) > state.acq.msPerLine
    state.init.eom.boxWidth = state.acq.msPerLine - (state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, 1) ...
        * state.acq.msPerLine);
end

updateGUIByGlobal('state.init.eom.boxWidth', 'Value', round(100 * state.init.eom.boxWidth) / 100, 'Callback', 0);
state.init.eom.powerBoxWidthsInMs(state.init.eom.powerBox.beamMenu) = state.init.eom.boxWidth;

pos(3) = state.init.eom.boxWidth / state.acq.msPerLine * state.acq.pixelsPerLine; %VI012109A
% updateGUIByGlobal('state.init.eom.boxWidth', 'Value', ...
%     pos(3) * (1000 * state.acq.msPerLine) / state.acq.pixelsPerLine), 'Callback', 0);

%Save the new coordinates.
imsize = [state.acq.pixelsPerLine  state.internal.storedLinesPerFrame]; %VI102209A
state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, :) = pos;
state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, [1 3]) = pos([1 3]) ./ imsize(1);
state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, [2 4]) = pos([2 4]) ./ imsize(2);

%Update all the channels.
for j = 1 : state.init.maximumNumberOfInputChannels
    set(state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, j), 'Position', pos);
end
end


function etStartFrame_Callback(hObject, eventdata, handles)
global state gh
genericCallback(hObject);
state.init.eom.changed(state.init.eom.powerBox.beamMenu)=1;
state.init.eom.startFrameArray(state.init.eom.powerBox.beamMenu)=state.init.eom.startFrame;
%updatePowerBoxStrings; %VI020809A

%Tim O'Connor 12/16/03, this should be an immediate warning.
%Grab the text from the gui object, because it changes when switching between linescan and framescan.
if state.init.eom.startFrame > state.init.eom.endFrame
    fprintf(2, 'WARNING: Start Frame/Line (%s) must be less than End Frame/Line (%s).\n', get(gh.powerBox.etStartFrame, 'String'), get(gh.powerBox.etEndFrame, 'String'));
end
end

function etEndFrame_Callback(hObject, eventdata, handles)
global state gh
genericCallback(hObject);
state.init.eom.changed(state.init.eom.powerBox.beamMenu)=1;
state.init.eom.endFrameArray(state.init.eom.powerBox.beamMenu)=state.init.eom.endFrame;
%updatePowerBoxStrings; %VI020809A

%Tim O'Connor 12/16/03, this should be an immediate warning.
%Grab the text from the gui object, because it changes when switching between linescan and framescan.
if state.init.eom.startFrame > state.init.eom.endFrame
    fprintf(2, 'WARNING: ''%s'' must be less than ''%s''.\n', get(gh.powerBox.etStartFrame, 'String'), get(gh.powerBox.etEndFrame, 'String'));
end
end

%% HELPER FUNCTIONS

% --------------------------------------------------------------------
function createFullWidthPowerBox
global state;

%The x parameter is 1, normalized by the pixels per line.
state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, 1) = 1 / state.acq.pixelsPerLine;

%The width parameter is 1 (after normalization).
state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, 3) = 1;

%The y and height parameters are both 1, normalized by the lines per frame.
state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, [2 4]) = 1 / state.internal.storedLinesPerFrame; %VI102209A

position([1 2 4]) = 1;
position(3) = state.acq.pixelsPerLine;

for j = 1 : state.init.maximumNumberOfInputChannels

    state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, j) = rectangle('Position',  position, ...
        'FaceColor', 'none', 'EdgeColor', state.init.eom.boxcolors(state.init.eom.powerBox.beamMenu, :), 'LineWidth', 3, ...
        'Parent', state.internal.axis(j), 'ButtonDownFcn', 'powerBoxButtonDownFcn', 'UserData', state.init.eom.powerBox.beamMenu, ...
        'Tag', sprintf('PowerBox%s', num2str(state.init.eom.powerBox.beamMenu)), 'Visible', 'On');

end

updateGUIByGlobal('state.init.eom.boxWidth', 'Value', round(100 * pos(3) * state.acq.msPerLine / state.acq.pixelsPerLine) / 100, 'Callback', 0); %VI012109A
state.init.eom.powerBoxWidthsInMs(state.init.eom.powerBox.beamMenu) = state.init.eom.boxWidth;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function updatePowerBoxGUI(beam)
% This will uodate the GUIs according to which beam is selected...
global state gh;

if length(state.init.eom.showBoxArray) < state.init.eom.numberOfBeams
    state.init.eom.showBoxArray(state.init.eom.numberOfBeams) = 0;
end
state.init.eom.showBox = state.init.eom.showBoxArray(state.init.eom.powerBox.beamMenu);
updateGUIByGlobal('state.init.eom.showBox');

%TO22004a - Pick up these variables from the GUI, if they don't exist, since config loading can overwrite them.
if length(state.init.eom.endFrameArray) < state.init.eom.powerBox.beamMenu
    state.init.eom.endFrameArray(state.init.eom.powerBox.beamMenu) = state.init.eom.endFrame;
end
state.init.eom.endFrame = state.init.eom.endFrameArray(state.init.eom.powerBox.beamMenu);
updateGUIByGlobal('state.init.eom.endFrame');

if length(state.init.eom.startFrameArray) < state.init.eom.powerBox.beamMenu
    state.init.eom.startFrameArray(state.init.eom.powerBox.beamMenu) = state.init.eom.startFrame;
end
state.init.eom.startFrame = state.init.eom.startFrameArray(state.init.eom.powerBox.beamMenu);
updateGUIByGlobal('state.init.eom.startFrame');

%Convert display to mW or % -- TO21804a
conversion = 1;
if get(gh.powerControl.mW_radioButton, 'Value') == get(gh.powerControl.mW_radioButton, 'Max')
    conversion = (getfield(state.init.eom, ['powerConversion' num2str(beam)]) * ...
        state.init.eom.maxPhotodiodeVoltage(beam) * .01);
end

if length(state.init.eom.boxPowerArray) < state.init.eom.powerBox.beamMenu
    state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu) = state.init.eom.boxPower;
end

%Added to allow power box to work in mW. -- Tim O'Connor TO21804a
state.init.eom.boxPower = round(conversion * state.init.eom.boxPowerArray(state.init.eom.powerBox.beamMenu));
updateGUIByGlobal('state.init.eom.boxPower');


%%%VI110310A%%%
if length(state.init.eom.boxPowerOffArray) < state.init.eom.powerBox.beamMenu
    state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu) = state.init.eom.boxPowerOff;
end

state.init.eom.boxPowerOff = round(conversion * state.init.eom.boxPowerOffArray(state.init.eom.powerBox.beamMenu));
updateGUIByGlobal('state.init.eom.boxPowerOff');
%%%%%%%%%%%%%%%



%Try to locate the (now properly tagged) object.
fig = get(0,'CurrentFigure');
obj = findobj('Tag', sprintf('PowerBox%s', num2str(state.init.eom.powerBox.beamMenu)));
if length(obj) > 1
    obj = obj(1);%Hope it's always the first one...???
end
pos = get(obj, 'Position');

if ~isempty(pos)
    %Change the coordinates.
    pos = get(obj, 'Position');
    updateGUIByGlobal('state.init.eom.boxWidth', 'Value', round(100 * pos(3) * state.acq.msPerLine / state.acq.pixelsPerLine) / 100, 'Callback', 0); %VI012109A
    state.init.eom.powerBoxWidthsInMs(state.init.eom.powerBox.beamMenu) = state.init.eom.boxWidth;
end

%updatePowerBoxStrings; %VI020809A

%%%VI020809A: Removed%%%%%%%%%%%%%%%
% function updatePowerBoxStrings
% % This will uodate the GUIs according to which beam is selected...
% global state
% state.init.eom.showBoxArrayString = mat2str(state.init.eom.showBoxArray);
% updateHeaderString('state.init.eom.showBoxArrayString');
% state.init.eom.endFrameArrayString = mat2str(state.init.eom.endFrameArray);
% updateHeaderString('state.init.eom.endFrameArrayString');
% state.init.eom.startFrameArrayString = mat2str(state.init.eom.startFrameArray);
% updateHeaderString('state.init.eom.startFrameArrayString');
% state.init.eom.boxPowerArrayString = mat2str(state.init.eom.boxPowerArray);
% updateHeaderString('state.init.eom.boxPowerArrayString');
% state.init.eom.boxPowerArrayOffString = mat2str(state.init.eom.boxPowerOffArray);
% updateHeaderString('state.init.eom.boxPowerOffArrayString');
% state.init.eom.powerBoxNormCoordsString = mat2str(state.init.eom.powerBoxNormCoords);
% updateHeaderString('state.init.eom.powerBoxNormCoordsString');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

% --------------------------------------------------------------------
function done = drawPowerBox(handle)
global state gh

done = 0;
state.init.eom.changed(state.init.eom.powerBox.beamMenu) = 1;
setImagesToWhole;


if nargin < 1
    ax=state.internal.axis(logical(state.acq.imagingChannel));
    image=state.internal.imagehandle(logical(state.acq.imagingChannel));
    ax = ax(1);
    image = image(1);
elseif ishandle(handle)
    %     ind = find(handle == state.internal.axis);
    %     if isempty(ind)
    %         return;
    %     end

    ax = handle;
    %image = state.internal.imagehandle(ind);
else
    return;
end

imsize = [state.acq.pixelsPerLine  state.internal.storedLinesPerFrame]; %VI102209A
pos = round(getRectFromAxes(ax,'Cursor','crosshair','nomovegui',1)); %VI071310A %VI021809B  %VI092308A
%pos = [pos(1) round(pos(2)) pos(3) round(pos(4))]; %VI092308A
if pos(3) == 0 || pos(4) == 0
    return;
elseif ~isempty(state.init.eom.boxHandles) & size(state.init.eom.boxHandles, 1) >= state.init.eom.powerBox.beamMenu
    if sum(state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, ishandle(state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, :)))) ~= 0
        delete(state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, ishandle(state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, :))));
        state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, :) = -1;
    end
end

%Constrain to a single line, for uncaging.
if state.init.eom.constrainBoxToLine(state.init.eom.powerBox.beamMenu) & abs(pos(4)) ~= 1
    if pos(4) < 0
        pos(4) = -1;
    elseif pos(4) > 0
        pos(4) = 1;
    end
end

%Updated to force it to overlap only 1 line on the image. -- Tim O'Connor 6/16/04 TO061604a
if state.init.eom.constrainBoxToLine(state.init.eom.powerBox.beamMenu)
    %pos(1) = round(pos(1)); %VI092308A
    pos(2) = round(pos(2));
end

%%%VI102309A: Constrain box to fall within image coordinates %%%%%%%
handleBelow(1,3);
handleBelow(2,4);
handleAbove(1,3,1);
handleAbove(2,4,2);
    function handleBelow(startIdx,spanIdx)
        if pos(startIdx) < 1
            shift = -pos(startIdx)+1;
            pos(startIdx) = pos(startIdx) + shift;
            pos(spanIdx) = pos(spanIdx) - shift;
        end
    end

    function handleAbove(startIdx,spanIdx,imsizeIdx)
        pos(spanIdx) = min(imsize(imsizeIdx)-pos(startIdx),pos(spanIdx));
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% remember coords in a config independent way.
state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, :) = pos;
state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, [1 3]) = state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, [1 3]) ./ imsize(1);
state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, [2 4]) = state.init.eom.powerBoxNormCoords(state.init.eom.powerBox.beamMenu, [2 4]) ./ imsize(2);

%Draw the boxes....
for j = 1 : state.init.maximumNumberOfInputChannels
    %Added in a half pixel correction (positive in X, negative in Y), to get the box to directly overlap the signal. -- TimO'Connor 6/16/04 - TO061604b
    state.init.eom.boxHandles(state.init.eom.powerBox.beamMenu, j) = rectangle('Position', pos, ... %[(pos(1) + .5) (pos(2) - .5) pos(3) pos(4)], ... %VI092208A
        'FaceColor', 'none', 'EdgeColor', state.init.eom.boxcolors(state.init.eom.powerBox.beamMenu, :), 'LineWidth', 2, 'Parent', state.internal.axis(j), ...
        'ButtonDownFcn', 'powerBoxButtonDownFcn', 'UserData', state.init.eom.powerBox.beamMenu, ...
        'Tag', sprintf('PowerBox%s', num2str(state.init.eom.powerBox.beamMenu))); %Added a tag, so it can be found. -- Tim 12/23/03
end
state.init.eom.showBoxArray(state.init.eom.powerBox.beamMenu) = 1;
state.init.eom.showBox = state.init.eom.showBoxArray(state.init.eom.powerBox.beamMenu);
updateGUIByGlobal('state.init.eom.showBox');
%updatePowerBoxStrings; %VI020809A

updateGUIByGlobal('state.init.eom.boxWidth', 'Value', round(100 * pos(3) * state.internal.activeMsPerLine / state.acq.pixelsPerLine) / 100, 'Callback', 0); %VI012109A, VI021909A
state.init.eom.powerBoxWidthsInMs(state.init.eom.powerBox.beamMenu) = state.init.eom.boxWidth;

end