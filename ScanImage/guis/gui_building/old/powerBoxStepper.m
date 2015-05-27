function varargout = powerBoxStepper(varargin)
% POWERBOXSTEPPER Application M-file for powerBoxStepper.fig
%    FIG = POWERBOXSTEPPER launch powerBoxStepper GUI.
%    POWERBOXSTEPPER('callback_name', ...) invoke the named callback.

% Last Modified by GUIDE v2.5 19-Feb-2004 13:22:37

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
function updateString
global state;

state.init.eom.powerBoxStepper.pbsArrayString = mat2str(state.init.eom.powerBoxStepper.pbsArray);

return;

% --------------------------------------------------------------------
function varargout = beamMenu_Callback(h, eventdata, handles, varargin)
global state;

genericCallback(h);

updateGUIByGlobal('state.init.eom.powerBoxStepper.beamSlider', 'Value', state.init.eom.numberOfBeams - state.init.eom.powerBoxStepper.selectedBeam + 1, 'Callback', 0);

updateGUIByGlobal('state.init.eom.powerBoxStepper.xStep', 'Value', state.init.eom.powerBoxStepper.pbsArray(state.init.eom.powerBoxStepper.selectedBeam, 1))
updateGUIByGlobal('state.init.eom.powerBoxStepper.yStep', 'Value', state.init.eom.powerBoxStepper.pbsArray(state.init.eom.powerBoxStepper.selectedBeam, 2))
updateGUIByGlobal('state.init.eom.powerBoxStepper.widthStep', 'Value', state.init.eom.powerBoxStepper.pbsArray(state.init.eom.powerBoxStepper.selectedBeam, 3))
updateGUIByGlobal('state.init.eom.powerBoxStepper.heightStep', 'Value', state.init.eom.powerBoxStepper.pbsArray(state.init.eom.powerBoxStepper.selectedBeam, 4))

return;

% --------------------------------------------------------------------
function varargout = beamSlider_Callback(h, eventdata, handles, varargin)
global state gh;

genericCallback(h);

%Keep things within bounds.
if state.init.eom.powerBoxStepper.beamSlider > state.init.eom.numberOfBeams
    state.init.eom.powerBoxStepper.beamSlider = state.init.eom.numberOfBeams;
elseif state.init.eom.powerBoxStepper.beamSlider < 1
    state.init.eom.powerBoxStepper.beamSlider = 1;
end

%Invert the slider, so that Beam2 is graphically below Beam1, to match the popup menu behavior.
state.init.eom.powerBoxStepper.selectedBeam = state.init.eom.numberOfBeams - state.init.eom.powerBoxStepper.beamSlider + 1;

updateGUIByGlobal('state.init.eom.powerBoxStepper.selectedBeam');

set(gh.powerBoxStepper.beamMenu, 'Value', state.init.eom.powerBoxStepper.selectedBeam);
powerBoxStepper('beamMenu_Callback', gh.powerBoxStepper.beamMenu);

return;

% --------------------------------------------------------------------
function varargout = xStepText_Callback(h, eventdata, handles, varargin)
global state;

genericCallback(h);

state.init.eom.powerBoxStepper.pbsArray(state.init.eom.powerBoxStepper.selectedBeam, 1) = ...
    state.init.eom.powerBoxStepper.xStep;

updateString;

return;

% --------------------------------------------------------------------
function varargout = yStepText_Callback(h, eventdata, handles, varargin)
global state;

genericCallback(h);

state.init.eom.powerBoxStepper.pbsArray(state.init.eom.powerBoxStepper.selectedBeam, 2) = ...
    state.init.eom.powerBoxStepper.yStep;

updateString;

return;

% --------------------------------------------------------------------
function varargout = widthText_Callback(h, eventdata, handles, varargin)
global state;

genericCallback(h);

state.init.eom.powerBoxStepper.pbsArray(state.init.eom.powerBoxStepper.selectedBeam, 3) = ...
    state.init.eom.powerBoxStepper.widthStep;

updateString;

return;

% --------------------------------------------------------------------
function varargout = heightText_Callback(h, eventdata, handles, varargin)
global state;

genericCallback(h);

state.init.eom.powerBoxStepper.pbsArray(state.init.eom.powerBoxStepper.selectedBeam, 4) = ...
    state.init.eom.powerBoxStepper.heightStep;

updateString;

return;

% --------------------------------------------------------------------
function varargout = enableCheckbox_Callback(h, eventdata, handles, varargin)

genericCallback(h);


