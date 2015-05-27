function varargout = uncagingPulseImporter(varargin)
% UNCAGINGPULSEIMPORTER Application M-file for uncagingPulseImporter.fig
%    FIG = UNCAGINGPULSEIMPORTER launch uncagingPulseImporter GUI.
%    UNCAGINGPULSEIMPORTER('callback_name', ...) invoke the named callback.

% Last Modified by GUIDE v2.5 19-Mar-2004 17:52:30
%% CHANGES
% Tim O'Connor 2/18/04 TO21804b: Make sure that the enable button is tightly coupled with the powerbox selection.
% Vijay Iyer 2/9/09 VI020909A: Allow state.init.eom.showBox to remain an array

if nargin == 0  % LAUNCH GUI

	fig = openfig(mfilename,'reuse');

	% Use system color scheme for figure:
	set(fig,'Color',get(0,'defaultUicontrolBackgroundColor'));

	% Generate a structure of handles to pass to callbacks, and store it. 
	handles = guihandles(fig);
	guidata(fig, handles);

	if nargout > 0
		varargout{1} = fig;
	end

elseif ischar(varargin{1}) % INVOKE NAMED SUBFUNCTION OR CALLBACK

	try
		if (nargout)
			[varargout{1:nargout}] = feval(varargin{:}); % FEVAL switchyard
		else
			feval(varargin{:}); % FEVAL switchyard
		end
	catch
		disp(lasterr);
	end

end


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

% --------------------------------------------------------------------
function varargout = powerConversionFactorText_Callback(h, eventdata, handles, varargin)

genericCallback(h);

return;

% --------------------------------------------------------------------
function varargout = lineConversionFactorText_Callback(h, eventdata, handles, varargin)
global state gh;

genericCallback(h);

if state.init.eom.uncagingPulseImporter.lineConversionFactor < .001
    state.init.eom.uncagingPulseImporter.lineConversionFactor = .001;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.lineConversionFactor');
end

if state.init.eom.uncagingPulseImporter.lineConversionFactor == state.acq.msPerLine %VI012109A
    %The conversion factor makes sense with the current scan settings.
    set(gh.uncagingPulseImporter.lineConversionFactorText, 'ForegroundColor', [0 0 0]);%Black    
else
    %This is probably not the value that the user really wants.
    set(gh.uncagingPulseImporter.lineConversionFactorText, 'ForegroundColor', [1 0 0]);%Red
    fprintf(2, 'Warning: The ''ms / line'' conversion factor in the UncagingPulseImporter does not match the current scan settings of %s ms/line.\n', ...
        num2str(state.acq.msPerLine)); %VI012109A
end

return;

% --------------------------------------------------------------------
function varargout = pathnameText_Callback(h, eventdata, handles, varargin)
global state gh;

genericCallback(h);

if exist(state.init.eom.uncagingPulseImporter.pathnameText) == 7 %It's a directory.
    set(gh.uncagingPulseImporter.pathnameText, 'ForegroundColor', [0 0 0]);%Black
    set(gh.uncagingPulseImporter.expandWindowButton, 'Enable', 'On');
    set(gh.uncagingPulseImporter.enableToggleButton, 'Enable', 'On');
    
    %Count the available pulses.
    %This method of counting is a bit of a cheat, maybe the filename prefix will be needed for specificity.
    state.init.eom.uncagingPulseImporter.pulseCount = length(dir(strcat(state.init.eom.uncagingPulseImporter.pathnameText, '*.mpf')));
else
    set(gh.uncagingPulseImporter.pathnameText, 'ForegroundColor', [1 0 0]);%Red
    
    %There's no need to mess with the cycle editor now.
    set(gh.uncagingPulseImporter.expandWindowButton, 'String', '<<')
    expandWindowButton_Callback(gh.uncagingPulseImporter.expandWindowButton)
    set(gh.uncagingPulseImporter.expandWindowButton, 'Enable', 'Off');
    
    %When expanded, start displaying at position 1.
    state.init.eom.uncagingPulseImporter.position = 1;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.position');
    
    %Mess around with the toggle button.
    set(gh.uncagingPulseImporter.enableToggleButton, 'Enable', 'Off');
    state.init.eom.uncagingPulseImporter.enabled = 0;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.enabled');
    enableToggleButton_Callback(gh.uncagingPulseImporter.enableToggleButton);
    
    if ~isempty(state.init.eom.uncagingPulseImporter.pathnameText)
        fprintf(2, 'Failed to find mpf files, %s is not a directory.\n', state.init.eom.uncagingPulseImporter.pathnameText);
    end
end

return;

% --------------------------------------------------------------------
function varargout = fileBrowseButton_Callback(h, eventdata, handles, varargin)
global state gh;

initfile = 'Open';
if exist(state.init.eom.uncagingPulseImporter.pathnameText) == 7
    initfile = strcat(state.init.eom.uncagingPulseImporter.pathnameText, initfile);
end

[fname, pname] = uiputfile(initfile,'Choose directory with the pulse set in it.');

if ~isempty(pname) & pname ~= 0
    state.init.eom.uncagingPulseImporter.pathnameText = pname;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.pathnameText');
    
    pathnameText_Callback(gh.uncagingPulseImporter.pathnameText);
end

return;

% --------------------------------------------------------------------
function varargout = enableToggleButton_Callback(h, eventdata, handles, varargin)
global state gh;

genericCallback(h);

%TO21804b - Make sure that it's clear that a powerbox must be selected. - Tim O'Connor 2/18/04
if state.init.eom.uncagingPulseImporter.enabled & ~ismember(1, state.init.eom.showBoxArray)
    
    if state.init.eom.uncagingPulseImporter.autoEnableBox
        for i = 1 : state.init.eom.numberOfBeams
            if length(state.init.eom.boxHandles) < i
                state.init.eom.boxHandles(i) = 0;
            end
            if state.init.eom.boxHandles(i) ~= 0 & ishandle(state.init.eom.boxHandles(i)) & ...
                    ~isempty(state.init.eom.powerBoxNormCoords)
% & ~strcmpi(state.init.eom.pockelsCellNames(i), ...
%                    state.init.eom.scanLaserName)
                state.init.eom.showBoxArray(i) = 1;
                updateGUIByGlobal('state.init.eom.beamMenu', 'Value', i, 'Callback', 1);
                updateGUIByGlobal('state.init.eom.showBox', 'Value', 1, 'Callback', 1);
            end
        end
    else
        beep;
        
        if state.init.eom.uncagingPulseImporter.coupleToPowerBoxErrors
            fprintf(2, 'ERROR: Can not enable the UncagingPulseImporter without at least one powerbox selected.\n');
            errordlg('A powerbox must be selected to enable this functionality.', 'UncagingPulseImporter', 'modal');
            
            state.init.eom.uncagingPulseImporter.enabled = 0;
            updateGUIByGlobal('state.init.eom.uncagingPulseImporter.enabled');
            
            return;
        else
            fprintf(2, 'WARNING: The UncagingPulseImporter is enabled, but will not become active until at least one powerbox is selected and enabled.\n');
        end
    end
end

if state.init.eom.uncagingPulseImporter.enabled
    set(gh.uncagingPulseImporter.enableToggleButton, 'ForegroundColor', [1 0 0]);
    set(gh.uncagingPulseImporter.enableToggleButton, 'String', 'Disable');
    
    if exist(state.init.eom.uncagingPulseImporter.pathnameText) ~= 7
        fprintf(2, 'The path to the mpf files must be selected in the UncagingPulseImporter.\n');
    end
else
    set(gh.uncagingPulseImporter.enableToggleButton, 'ForegroundColor', [0 .6 0]);
    set(gh.uncagingPulseImporter.enableToggleButton, 'String', 'Enable');
   
    %TO3204a - Automatically disable the powerbox, for safety. :: Tim O'Connor 3/2/04
    state.init.eom.showBoxArray(:) = 0; %VI020909A
    updateGUIByGlobal('state.init.eom.showBox', 'Value', 0);
    if ~isempty(state.init.eom.boxHandles)
        for i = 1 : prod(size(state.init.eom.boxHandles))
            if state.init.eom.boxHandles(i) > 0 & ishandle(state.init.eom.boxHandles)
                set(state.init.eom.boxHandles(i), 'Visible', 'Off');
            end
        end
    end

end

% state.init.eom.uncagingPulseImporter.enabledArray(state.init.eom.uncagingPulseImporter.beam) = state.init.eom.uncagingPulseImporter.enabled;

return;

% --------------------------------------------------------------------
function varargout = positionUpSlider_Callback(h, eventdata, handles, varargin)
global state gh;

if size(state.init.eom.uncagingPulseImporter.cycleArray, 2) > state.init.eom.uncagingPulseImporter.position
    state.init.eom.uncagingPulseImporter.position = state.init.eom.uncagingPulseImporter.position + 1;
    
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.position');
    positionText_Callback(gh.uncagingPulseImporter.positionText);
end

return;

% --------------------------------------------------------------------
function varargout = positionDownSlider_Callback(h, eventdata, handles, varargin)
global state gh;

if state.init.eom.uncagingPulseImporter.position > 1
    state.init.eom.uncagingPulseImporter.position = state.init.eom.uncagingPulseImporter.position - 1;
    
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.position');
    positionText_Callback(gh.uncagingPulseImporter.positionText);
end

return;

% --------------------------------------------------------------------
function varargout = positionText_Callback(h, eventdata, handles, varargin)
global state gh;

genericCallback(h);

%Keep the value in bounds.
if state.init.eom.uncagingPulseImporter.position > size(state.init.eom.uncagingPulseImporter.cycleArray, 2)
    state.init.eom.uncagingPulseImporter.position = size(state.init.eom.uncagingPulseImporter.cycleArray, 2);
elseif state.init.eom.uncagingPulseImporter.position < 1
    state.init.eom.uncagingPulseImporter.position = 1;
end

updateGUIByGlobal('state.init.eom.uncagingPulseImporter.position');

%Update the values.
updateCycleValues;

return;

% --------------------------------------------------------------------
function varargout = beamLeftSlider1_Callback(h, eventdata, handles, varargin)
global state gh;

if state.init.eom.numberOfBeams < 2
    return;
end

if state.init.eom.uncagingPulseImporter.beam1 > 1
    state.init.eom.uncagingPulseImporter.beam1 = state.init.eom.uncagingPulseImporter.beam1 - 1;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.beam1');
    beamText1_Callback(gh.uncagingPulseImporter.beamText1);
end

return;

% --------------------------------------------------------------------
function varargout = beamRightSlider1_Callback(h, eventdata, handles, varargin)
global state gh;

if state.init.eom.numberOfBeams < 2
    return;
end

if state.init.eom.uncagingPulseImporter.beam1 < size(state.init.eom.uncagingPulseImporter.cycleArray, 1) - 1
    state.init.eom.uncagingPulseImporter.beam1 = state.init.eom.uncagingPulseImporter.beam1 + 1;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.beam1');
    beamText1_Callback(gh.uncagingPulseImporter.beamText1);
end

return;

% --------------------------------------------------------------------
function varargout = beamText1_Callback(h, eventdata, handles, varargin)
global state;

genericCallback(h);

if state.init.eom.numberOfBeams < 2
    state.init.eom.uncagingPulseImporter.beam1 = 1;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.beam1');
    return;
end

if state.init.eom.uncagingPulseImporter.beam1 < 1
    state.init.eom.uncagingPulseImporter.beam1 = 1;
elseif state.init.eom.uncagingPulseImporter.beam1 > size(state.init.eom.uncagingPulseImporter.cycleArray, 1) - 1;
    state.init.eom.uncagingPulseImporter.beam1 = size(state.init.eom.uncagingPulseImporter.cycleArray, 1) - 1;
end

state.init.eom.uncagingPulseImporter.beam2 = state.init.eom.uncagingPulseImporter.beam1 + 1;
updateGUIByGlobal('state.init.eom.uncagingPulseImporter.beam1');
updateGUIByGlobal('state.init.eom.uncagingPulseImporter.beam2');

updateCycleValues;

return;

% --------------------------------------------------------------------
function varargout = beamLeftSlider2_Callback(h, eventdata, handles, varargin)
global gh;

beamLeftSlider1_Callback(gh.uncagingPulseImporter.beamLeftSlider1);

return;

% --------------------------------------------------------------------
function varargout = beamRightSlider2_Callback(h, eventdata, handles, varargin)
global gh;

beamRightSlider1_Callback(gh.uncagingPulseImporter.beamRightSlider1);

return;

% --------------------------------------------------------------------
function varargout = beamText2_Callback(h, eventdata, handles, varargin)
global state;

genericCallback(h);

if state.init.eom.uncagingPulseImporter.beam2 < 2
    state.init.eom.uncagingPulseImporter.beam1 = 2;
elseif state.init.eom.uncagingPulseImporter.beam1 > size(state.init.eom.uncagingPulseImporter.cycleArray, 1);
    state.init.eom.uncagingPulseImporter.beam1 = size(state.init.eom.uncagingPulseImporter.cycleArray, 1);
end

state.init.eom.uncagingPulseImporter.beam1 = state.init.eom.uncagingPulseImporter.beam2 - 1;
updateGUIByGlobal('state.init.eom.uncagingPulseImporter.beam1');
updateGUIByGlobal('state.init.eom.uncagingPulseImporter.beam2');

updateCycleValues;

return;

% --------------------------------------------------------------------
function varargout = expandWindowButton_Callback(h, eventdata, handles, varargin)
global state gh;

pos = get(gh.uncagingPulseImporter.figure1, 'Position');
    
if strcmp(get(gh.uncagingPulseImporter.expandWindowButton, 'String'), '>>')

    set(gh.uncagingPulseImporter.expandWindowButton, 'String', '<<');
    
    if state.init.eom.numberOfBeams > 1
        pos(3) = 88.8;
    else
        pos(3) = 80;
    end

    set(gh.uncagingPulseImporter.figure1, 'Position', pos);
else

    set(gh.uncagingPulseImporter.expandWindowButton, 'String', '>>');
    
    pos(3) = 50;

    set(gh.uncagingPulseImporter.figure1, 'Position', pos);
end

return;

% --------------------------------------------------------------------
function varargout = cycleValueText1_Callback(h, eventdata, handles, varargin)
global state gh;

genericCallback(h);

%Enforce boundary conditions.
if state.init.eom.uncagingPulseImporter.cycleValue1 > state.init.eom.uncagingPulseImporter.pulseCount
    
    state.init.eom.uncagingPulseImporter.cycleValue1 = state.init.eom.uncagingPulseImporter.pulseCount;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.cycleValue1');
    
elseif state.init.eom.uncagingPulseImporter.cycleValue1 < 0
    
    %0 flags this as not having an associated pulse.
    state.init.eom.uncagingPulseImporter.cycleValue1 = 0;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.cycleValue1');
    
end

state.init.eom.uncagingPulseImporter.cycleArray(state.init.eom.uncagingPulseImporter.beam1, state.init.eom.uncagingPulseImporter.position) = ...
    state.init.eom.uncagingPulseImporter.cycleValue1;

state.init.eom.uncagingPulseImporter.cycleArrayString = mat2str(state.init.eom.uncagingPulseImporter.cycleArray);

return;

% --------------------------------------------------------------------
function varargout = cycleValueText2_Callback(h, eventdata, handles, varargin)
global state gh;

genericCallback(h);

%Enforce boundary conditions.
if state.init.eom.uncagingPulseImporter.cycleValue2 > state.init.eom.uncagingPulseImporter.pulseCount
    
    state.init.eom.uncagingPulseImporter.cycleValue2 = state.init.eom.uncagingPulseImporter.pulseCount;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.cycleValue2');
    
elseif state.init.eom.uncagingPulseImporter.cycleValue2 < 0
    
    %0 flags this as not having an associated pulse.
    state.init.eom.uncagingPulseImporter.cycleValue2 = 0;
    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.cycleValue2');
    
end

state.init.eom.uncagingPulseImporter.cycleArray(state.init.eom.uncagingPulseImporter.beam2, state.init.eom.uncagingPulseImporter.position) = ...
    state.init.eom.uncagingPulseImporter.cycleValue2;

%Save the changes.
state.init.eom.uncagingPulseImporter.cycleArrayString = mat2str(state.init.eom.uncagingPulseImporter.cycleArray);

return;

% --------------------------------------------------------------------
function varargout = syncToPhysiologyCheckbox_Callback(h, eventdata, handles, varargin)

genericCallback(h);

return;

% --------------------------------------------------------------------
function varargout = addPosition_Callback(h, eventdata, handles, varargin)
global state gh;

s = size(state.init.eom.uncagingPulseImporter.cycleArray, 2);

%Create the new one.
state.init.eom.uncagingPulseImporter.cycleArray(1 : state.init.eom.numberOfBeams, s + 1) = 0;

%Save the changes.
state.init.eom.uncagingPulseImporter.cycleArrayString = mat2str(state.init.eom.uncagingPulseImporter.cycleArray);

%Update the gui.
state.init.eom.uncagingPulseImporter.position = s + 1;
updateGUIByGlobal('state.init.eom.uncagingPulseImporter.position');
positionText_Callback(gh.uncagingPulseImporter.positionText);

return;

% --------------------------------------------------------------------
function varargout = deletePosition_Callback(h, eventdata, handles, varargin)
global state gh;

s = size(state.init.eom.uncagingPulseImporter.cycleArray, 2);

if size(state.init.eom.uncagingPulseImporter.cycleArray, 2) == 1

    %Remove the only one.
    state.init.eom.uncagingPulseImporter.cycleArray(1, 1) = 0;
    state.init.eom.uncagingPulseImporter.position = s;
    
elseif state.init.eom.uncagingPulseImporter.position < s

    %Remove 1, somewhere before the end.
    tempArr = state.init.eom.uncagingPulseImporter.cycleArray(1 : state.init.eom.numberOfBeams, 1 : state.init.eom.uncagingPulseImporter.position - 1);
    tempArr(1 : state.init.eom.numberOfBeams, state.init.eom.uncagingPulseImporter.position : s - 1) = ...
        state.init.eom.uncagingPulseImporter.cycleArray(1 : state.init.eom.numberOfBeams, state.init.eom.uncagingPulseImporter.position + 1 : s);

    state.init.eom.uncagingPulseImporter.cycleArray = tempArr;

    if state.init.eom.uncagingPulseImporter.position < s
        state.init.eom.uncagingPulseImporter.position = state.init.eom.uncagingPulseImporter.position + 1;
    else
        state.init.eom.uncagingPulseImporter.position = state.init.eom.uncagingPulseImporter.position - 1;
    end
    
else
    
    %Remove the last one.
    tempArr = state.init.eom.uncagingPulseImporter.cycleArray(1 : state.init.eom.numberOfBeams, 1 : s - 1);
   
    state.init.eom.uncagingPulseImporter.cycleArray = tempArr;
    state.init.eom.uncagingPulseImporter.position = s - 1;
    
end

%Save the changes.
state.init.eom.uncagingPulseImporter.cycleArrayString = mat2str(state.init.eom.uncagingPulseImporter.cycleArray);

%Update the display.
updateGUIByGlobal('state.init.eom.uncagingPulseImporter.position');
positionText_Callback(gh.uncagingPulseImporter.positionText);

return;

% --------------------------------------------------------------------
function updateCycleValues
global state gh;

state.init.eom.uncagingPulseImporter.cycleValue1 = ...
    state.init.eom.uncagingPulseImporter.cycleArray(state.init.eom.uncagingPulseImporter.beam1, state.init.eom.uncagingPulseImporter.position);

updateGUIByGlobal('state.init.eom.uncagingPulseImporter.cycleValue1');

%Only change this one if there's more than one beam.
if state.init.eom.numberOfBeams > 1
    state.init.eom.uncagingPulseImporter.cycleValue2 = ...
        state.init.eom.uncagingPulseImporter.cycleArray(state.init.eom.uncagingPulseImporter.beam2, state.init.eom.uncagingPulseImporter.position);

    updateGUIByGlobal('state.init.eom.uncagingPulseImporter.cycleValue2');
end

return;


% --- Executes on button press in resetPosition.
function resetPosition_Callback(hObject, eventdata, handles)
% hObject    handle to resetPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

updateGUIByGlobal('state.init.eom.uncagingPulseImporter.position', 'Value', 1, 'Callback', 1);
