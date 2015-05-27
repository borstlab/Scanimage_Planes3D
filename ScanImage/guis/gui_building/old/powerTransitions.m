function varargout = powerTransitions(varargin)
% POWERTRANSITIONS Application M-file for powerTransitions.fig
%    FIG = POWERTRANSITIONS launch powerTransitions GUI.
%    POWERTRANSITIONS('callback_name', ...) invoke the named callback.

% Last Modified by GUIDE v2.0 17-Oct-2003 15:48:53
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
function timeInCurrentUnits = convertTimeFromMs(timeInMs)
global state;

    if state.init.eom.powerTransitions.framesTimeUnits
        timeInCurrentUnits = timeInMs / (state.acq.msPerLine*state.acq.linesPerFrame); %VI012109A
    elseif state.init.eom.powerTransitions.linesTimeUnits
        timeInCurrentUnits = timeInMs / state.acq.msPerLine; %VI012109A
    else
        timeInCurrentUnits = timeInMs;
    end

% --------------------------------------------------------------------
function timeInMs = convertTime2Ms(timeInCurrentUnits)
global state;

    if state.init.eom.powerTransitions.framesTimeUnits
        timeInMs = timeInCurrentUnits * state.acq.msPerLine * state.acq.linesPerFrame; %VI012109A
    elseif state.init.eom.powerTransitions.linesTimeUnits
        timeInMs = timeInCurrentUnits * state.acq.msPerLine; %VI012109A
    else
        timeInMs = timeInCurrentUnits;
    end
        
% --------------------------------------------------------------------
function updateStrings
global state;

    state.init.eom.powerTransitions.powerString = mat2str(state.init.eom.powerTransitions.power);
    state.init.eom.powerTransitions.timeString = mat2str(state.init.eom.powerTransitions.time);
    updateHeaderString('state.init.eom.powerTransitions.powerString');
    updateHeaderString('state.init.eom.powerTransitions.timeString');

    state.init.eom.powerTransitions.protocols(state.init.eom.powerTransitions.currentProtocol, state.init.eom.currentPowerTransitionBeam, ...
        1, :) = state.init.eom.powerTransitions.power(state.init.eom.currentPowerTransitionBeam, :);
    state.init.eom.powerTransitions.protocols(state.init.eom.powerTransitions.currentProtocol, state.init.eom.currentPowerTransitionBeam, ...
        2, :) = state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, :);
    
    %Pack all the protocol/beam/transition sets into one 4 dimensional array encoded as a string.
    numOfProtocols = size(state.init.eom.powerTransitions.protocols, 1);
    protocolString = '';
    for i = 1 : numOfProtocols
        for j = 1 : state.init.eom.numberOfBeams
            if size(state.init.eom.powerTransitions.protocols, 2) < j
                state.init.eom.powerTransitions.protocols(i, j, 1, :) = -1;
                state.init.eom.powerTransitions.protocols(i, j, 2, :) = -1;
            end
            
            tempPower(:, :) = state.init.eom.powerTransitions.protocols(i, j, 1, :);
            tempTime(:, :) = state.init.eom.powerTransitions.protocols(i, j, 2, :);
            protocolString = strcat(protocolString, mat2str(tempPower), ':', mat2str(tempTime), ':');%':' separates arrays within beams.
        end
        
        protocolString = strcat(protocolString, '!');%'!' separates protocols.
    end

    state.init.eom.powerTransitions.protocolString = protocolString;
    
    return;
    
% --------------------------------------------------------------------
function loadCurrent;
global state gh

    %Nothing to do.
    if isempty(state.init.eom.powerTransitions.transitionCount) | ...
            state.init.eom.powerTransitions.transitionCount(state.init.eom.currentPowerTransitionBeam) == 0
        return;
    end
    
    if size(state.init.eom.powerTransitions.power, 1) == 0
        return;
    end

    if size(state.init.eom.powerTransitions.power, 2) == 0
        return;
    end

    %Load the correct time (in the correct units).
    state.init.eom.powerTransitions.guiTime = convertTimeFromMs(state.init.eom.powerTransitions.time(...
        state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition));

    %Load the correct power.
    state.init.eom.powerTransitions.guiPower = state.init.eom.powerTransitions.power(...
        state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition); 
    
    %Update the GUI.
    updateGUIByGlobal('state.init.eom.powerTransitions.guiTime');
    updateGUIByGlobal('state.init.eom.powerTransitions.guiPower');
    powerTransitions('useBinaryTransitions_Callback',gh.powerTransitions.useBinaryTransitions);

    return;

% --------------------------------------------------------------------
function varargout = currentPowerTransition_Callback(h, eventdata, handles, varargin)

    genericCallback(h);
    loadCurrent;
 
    return;

% --------------------------------------------------------------------
function varargout = beamMenu_Callback(h, eventdata, handles, varargin)
global state gh;

    genericCallback(h);

    if size(state.init.eom.powerTransitions.power, 1) < state.init.eom.numberOfBeams
        state.init.eom.powerTransitions.power(state.init.eom.numberOfBeams, :) = -1;
        state.init.eom.powerTransitions.time(state.init.eom.numberOfBeams, :) = -1;
    end
    
    %Decide how many valid transitions are defined for this beam.
    num = length(find(state.init.eom.powerTransitions.power(state.init.eom.currentPowerTransitionBeam, :) ~= -1));
    if state.init.eom.powerTransitions.power(1) == -1
        num = 0;
    end

    %Create the transitions popup menu.
    vals = {};
    for i = 1:num
        vals(i) = cellstr(sprintf('Transition %s',  num2str(i)));
    end    
    
    if isempty(vals)
        vals = cellstr('');
%         return;
    end

    set(gh.powerTransitions.currentPowerTransition, 'String', vals);
    state.init.eom.currentPowerTransition = 1;
    updateGUIByGlobal('state.init.eom.currentPowerTransition');
    loadCurrent;

    %Make sure the gui elements are visible and enabled.
    if state.init.eom.powerTransitions.transitionCount(state.init.eom.currentPowerTransitionBeam) ~= 0
        transitionVisibility(1);
    else
        transitionVisibility(0);
    end
%         set(gh.powerTransitions.delete, 'Enable', 'On');
%         set(gh.powerTransitions.delete, 'Visible', 'On');
%         set(gh.powerTransitions.currentPowerTransition, 'Visible', 'On');
%         set(gh.powerTransitions.time, 'Enable', 'On');
%         set(gh.powerTransitions.power, 'Enable', 'On');
%         set(gh.powerTransitions.time, 'Visible', 'On');
%         set(gh.powerTransitions.power, 'Visible', 'On');
%     else
%         set(gh.powerTransitions.delete, 'Enable', 'Off');
%         set(gh.powerTransitions.delete, 'Visible', 'Off');
%         set(gh.powerTransitions.currentPowerTransition, 'Visible', 'Off');
%         set(gh.powerTransitions.time, 'Enable', 'Off');
%         set(gh.powerTransitions.power, 'Enable', 'Off');
%         set(gh.powerTransitions.time, 'Visible', 'Off');
%         set(gh.powerTransitions.power, 'Visible', 'Off');
%     end

    powerTransitions('useBinaryTransitions_Callback', gh.powerTransitions.useBinaryTransitions);

    return;
    
% --------------------------------------------------------------------
function varargout = time_Callback(h, eventdata, handles, varargin)
global state;

    genericCallback(h);

    %Convert the time from whatever units we're in...
    timeInMs = convertTime2Ms(state.init.eom.powerTransitions.guiTime);
    
    %Check the bounds on it.
    if timeInMs < 0
        
        timeInMs = 0;
        state.init.eom.powerTransitions.guiTime = convertTimeFromMs(timeInMs);
        updateGUIByGlobal('state.init.eom.powerTransitions.guiTime');
        
    end %Any other boundary conditions wanted? Should it not allow "wrapping" or "overrunning"?
    
    %Don't let the same time get used multiple times.
    indices = find(state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, :) == timeInMs);
    if indices & (indices ~= state.init.eom.currentPowerTransition)
        beep;
        disp('Can not specify two power transitions at the same time.');
        return;
    end        

    state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition) = timeInMs;
    
    state.init.eom.changed(state.init.eom.currentPowerTransitionBeam) = 1;

    %Save the current matrices.
    updateStrings;

    return;

% --------------------------------------------------------------------
function varargout = power_Callback(h, eventdata, handles, varargin)
global state;

    genericCallback(h);
    
%     %Check the bounds on it.
%     %It must fall between state.init.eom.minX and 100.
%     if state.init.eom.powerTransitions.guiPower < state.init.eom.min(state.init.eom.currentPowerTransitionBeam)
%         
%         state.init.eom.powerTransitions.guiPower = state.init.eom.min(state.init.eom.currentPowerTransitionBeam);
%         updateGUIByGlobal('state.init.eom.powerTransitions.guiPower');
%         
%     elseif state.init.eom.powerTransitions.guiPower > 100
%         
%         state.init.eom.powerTransitions.guiPower = 100;
%         updateGUIByGlobal('state.init.eom.powerTransitions.guiPower');
%         
%     end        
    %Check the bounds on it.
    %It must fall between 0 and 100.
    if state.init.eom.powerTransitions.guiPower < 0
        
        state.init.eom.powerTransitions.guiPower = 0;
        updateGUIByGlobal('state.init.eom.powerTransitions.guiPower');
        
    elseif state.init.eom.powerTransitions.guiPower > 100
        
        state.init.eom.powerTransitions.guiPower = 100;
        updateGUIByGlobal('state.init.eom.powerTransitions.guiPower');
        
    end        
    
    state.init.eom.powerTransitions.power(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition) = state.init.eom.powerTransitions.guiPower;
    
    state.init.eom.changed(state.init.eom.currentPowerTransitionBeam) = 1;

    %Save the current matrices.
    updateStrings;
    
    return;

% --------------------------------------------------------------------
function varargout = new_Callback(h, eventdata, handles, varargin)
global state gh;

    %Add a new transition to the popup menu.
    if isempty(get(gh.powerTransitions.currentPowerTransition, 'String'))
        vals = {};
    elseif length(get(gh.powerTransitions.currentPowerTransition, 'String')) == 1 & ...
            state.init.eom.powerTransitions.transitionCount(state.init.eom.currentPowerTransitionBeam) == 0
        vals = {};
    elseif ~state.init.eom.powerTransitions.transitionCount(state.init.eom.currentPowerTransitionBeam)
        vals = {};
    elseif isempty(state.init.eom.powerTransitions.transitionCount(state.init.eom.currentPowerTransitionBeam))
        vals = {};
    elseif state.init.eom.powerTransitions.transitionCount(state.init.eom.currentPowerTransitionBeam) == 0
        vals = {};
    else
        vals = get(gh.powerTransitions.currentPowerTransition, 'String');
    end

    vals(length(vals) + 1) = cellstr(sprintf('Transition %s',  num2str(length(vals) + 1)));
    set(gh.powerTransitions.currentPowerTransition, 'String', vals);

    %Initialize the arrays, if necessary.
    if isempty(state.init.eom.powerTransitions.time)
        state.init.eom.powerTransitions.time(1, 1) = -1;
    end
    if isempty(state.init.eom.powerTransitions.power)
        state.init.eom.powerTransitions.power(1, 1) = -1;
    end

    %Work with the new transition.
    state.init.eom.currentPowerTransition = length(vals);
    
    %Pad out the array.
    if size(state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, :), 2) < length(vals)
        state.init.eom.powerTransitions.time(:, length(vals)) = -1;
        state.init.eom.powerTransitions.power(:, length(vals)) = -1;
    end
    if size(state.init.eom.powerTransitions.protocols, 4) < length(vals)
        state.init.eom.powerTransitions.protocols(:, :, :, length(vals)) = -1;
    end

    %Set the default time.
    state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition) = ...
        max(state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, :))+1;

    %Set the default power.
    state.init.eom.powerTransitions.power(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition) = ...
        state.init.eom.min(state.init.eom.currentPowerTransitionBeam);

    transitionVisibility(1);
%     %There's definitely something to delete, now.
%     set(gh.powerTransitions.delete, 'Enable', 'On');
%     set(gh.powerTransitions.delete, 'Visible', 'On');
%     set(gh.powerTransitions.currentPowerTransition, 'Visible', 'On');
% 
%     %The time/power can now be associated with a transition.
%     set(gh.powerTransitions.time, 'Enable', 'On');
%     set(gh.powerTransitions.power, 'Enable', 'On');
%     set(gh.powerTransitions.time, 'Visible', 'On');
%     set(gh.powerTransitions.power, 'Visible', 'On');

    %Flag for a data output update.
    state.init.eom.changed(state.init.eom.currentPowerTransitionBeam) = 1;
    
    %Select the new transition object.
    state.init.eom.currentPowerTransition = length(vals);

    if length(state.init.eom.powerTransitions.transitionCount) > 0
        state.init.eom.powerTransitions.transitionCount(state.init.eom.currentPowerTransitionBeam) = ...
            state.init.eom.powerTransitions.transitionCount(state.init.eom.currentPowerTransitionBeam) + 1;
    else
        state.init.eom.powerTransitions.transitionCount(state.init.eom.currentPowerTransitionBeam) = 1;
    end

    state.init.eom.powerTransitions.transitionCountString = mat2str(state.init.eom.powerTransitions.transitionCount);
    updateHeaderString('state.init.eom.powerTransitions.transitionCountString');

    %Update the GUI.
    updateGUIByGlobal('state.init.eom.currentPowerTransition');

    %Load the current transition.
    loadCurrent;

    %Save the current matrices.
    updateStrings;

    return;

% --------------------------------------------------------------------
function varargout = delete_Callback(h, eventdata, handles, varargin)
global state gh;
   
    vals = get(gh.powerTransitions.currentPowerTransition, 'String');

    if length(vals) == 0

        %This case should never occur, but check for it anyway.
        
        transitionVisibility(0);
%         set(h, 'Enable', 'Off');%Nothing to delete, so disable it.
%         set(gh.powerTransitions.currentPowerTransition, 'Visible', 'Off');
%         %These only make sense in the context of a transition.
%         set(gh.powerTransitions.time, 'Enable', 'Inactive');
%         set(gh.powerTransitions.power, 'Enable', 'Inactive');
%         set(gh.powerTransitions.time, 'Visible', 'Off');
%         set(gh.powerTransitions.power, 'Visible', 'Off');

    elseif length(vals) == 1

        transitionVisibility(0);
%         set(h, 'Enable', 'Off');%Nothing else to delete, so disable it.
%         set(gh.powerTransitions.currentPowerTransition, 'Visible', 'Off');
%         %These only make sense in the context of a transition.
%         set(gh.powerTransitions.time, 'Enable', 'Inactive');
%         set(gh.powerTransitions.power, 'Enable', 'Inactive');
%         set(gh.powerTransitions.time, 'Visible', 'Off');
%         set(gh.powerTransitions.power, 'Visible', 'Off');
        
        %Fill in 'bad' values...
        state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, 1) = -1;
        state.init.eom.powerTransitions.power(state.init.eom.currentPowerTransitionBeam, 1) = -1;

        state.init.eom.powerTransitions.transitionCount(state.init.eom.currentPowerTransitionBeam) = 0;

        state.init.eom.currentPowerTransition = 1;
    else
        newvals = {};
        newvals = vals(1:length(vals) - 1);

        for i = state.init.eom.currentPowerTransition : length(vals) - 1

            %Pull down the next time value.
            state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition) = ...
                state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition + 1);

            %Pull down the next power value.
            state.init.eom.powerTransitions.power(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition) = ...
                state.init.eom.powerTransitions.power(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition + 1);
        end
        
        %Pad out the array properly.
        state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, length(vals)) = -1;
        state.init.eom.powerTransitions.power(state.init.eom.currentPowerTransitionBeam, length(vals)) = -1;

        set(gh.powerTransitions.currentPowerTransition, 'Value', length(newvals));%Watch out for an out-of-bounds error.
        set(gh.powerTransitions.currentPowerTransition, 'String', newvals);
        state.init.eom.currentPowerTransition = max(1,state.init.eom.currentPowerTransition - 1);
    end
    %Decrement the count.
    if state.init.eom.powerTransitions.transitionCount > 1
        state.init.eom.powerTransitions.transitionCount = state.init.eom.powerTransitions.transitionCount - 1;
    end
    state.init.eom.powerTransitions.transitionCountString = mat2str(state.init.eom.powerTransitions.transitionCount);

    %Flag for a data output update.
    state.init.eom.changed(state.init.eom.currentPowerTransitionBeam) = 1;

    %Update the GUI.
%    updateGUIByGlobal('state.init.eom.currentPowerTransition'); %This should work, but it seems to have some really wacky Matlab junk going on.
    loadCurrent;
    %Save the current matrices.
    updateStrings;

    return;

% --------------------------------------------------------------------
function varargout = msTimeUnits_Callback(h, eventdata, handles, varargin)
global gh state;

    genericCallback(h);

    set(h,'Enable','Inactive');
    set(gh.powerTransitions.framesTimeUnits, 'Enable', 'On');
    set(gh.powerTransitions.framesTimeUnits, 'Value', 0);
    state.init.eom.powerTransitions.framesTimeUnits = 0;
    set(gh.powerTransitions.linesTimeUnits, 'Enable', 'On');
    set(gh.powerTransitions.linesTimeUnits, 'Value', 0);
    state.init.eom.powerTransitions.linesTimeUnits = 0;
        
    if ~isempty(state.init.eom.powerTransitions.time)
        timeInMs = convertTime2Ms(state.init.eom.powerTransitions.guiTime);
        state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition) = timeInMs;
    end

    loadCurrent;

    return;

% --------------------------------------------------------------------
function varargout = framesTimeUnits_Callback(h, eventdata, handles, varargin)
global gh state;

    genericCallback(h);

    set(h,'Enable','Inactive');
    set(gh.powerTransitions.msTimeUnits, 'Enable', 'On');
    set(gh.powerTransitions.msTimeUnits, 'Value', 0);
    state.init.eom.powerTransitions.msTimeUnits = 0;
    set(gh.powerTransitions.linesTimeUnits, 'Enable', 'On');
    set(gh.powerTransitions.linesTimeUnits, 'Value', 0);
    state.init.eom.powerTransitions.linesTimeUnits = 0;
        
    if ~isempty(state.init.eom.powerTransitions.time)
        timeInMs = convertTime2Ms(state.init.eom.powerTransitions.guiTime);
        state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition) = timeInMs;
    end

    loadCurrent;

    return;
    
% --------------------------------------------------------------------
function varargout = linesTimeUnits_Callback(h, eventdata, handles, varargin)
global gh state;

    genericCallback(h);

    set(h,'Enable','Inactive');
    set(gh.powerTransitions.msTimeUnits, 'Enable', 'On');
    set(gh.powerTransitions.msTimeUnits, 'Value', 0);
    state.init.eom.powerTransitions.msTimeUnits = 0;
    set(gh.powerTransitions.framesTimeUnits, 'Enable', 'On');
    set(gh.powerTransitions.framesTimeUnits, 'Value', 0);
    state.init.eom.powerTransitions.framesTimeUnits = 0;
        
    if ~isempty(state.init.eom.powerTransitions.time)
        timeInMs = convertTime2Ms(state.init.eom.powerTransitions.guiTime);
        state.init.eom.powerTransitions.time(state.init.eom.currentPowerTransitionBeam, state.init.eom.currentPowerTransition) = timeInMs;
    end

    loadCurrent;

    return;

% --------------------------------------------------------------------
function varargout = useBinaryTransitions_Callback(h, eventdata, handles, varargin)
global state gh;

    genericCallback(h);
    
    if state.init.eom.powerTransitions.useBinaryTransitions
        set(gh.powerTransitions.power, 'Enable', 'off');
    else
        set(gh.powerTransitions.power, 'Enable', 'on');
    end
    
    state.init.eom.changed(:) = 1;
    
    return;

% --------------------------------------------------------------------
function initializeProtocols
global state;

    %Load the protocols variable from the encoded string.
    if length(state.init.eom.powerTransitions.protocols) == 0
        loadProtocolsFromString;
    end
    %If there was nothing to load, just take the current values.
    if length(state.init.eom.powerTransitions.protocols) == 0
        %Protocols are identified along dimension 1.
        %Dimension two identifies the whether it's a power or time matrix, as follows:
        % 1 = power; 2 = time;
        
        for i = 1:numberOfBeams
            state.init.eom.powerTransitions.protocols(1, i, 1, :) = state.init.eom.powerTransitions.power;
            state.init.eom.powerTransitions.protocols(1, i, 2, :) = state.init.eom.powerTransitions.time;
        end     
    end
    
    return;

% -------------------------------------------------------------------
function varargout = protocol_Callback(h, eventdata, handles, varargin)
global state gh;
    
    genericCallback(h);

    initializeProtocols;

    %Initialize the arrays.
    state.init.eom.powerTransitions.time = -1 * ones(state.init.eom.numberOfBeams, size(state.init.eom.powerTransitions.protocols, 4));
    state.init.eom.powerTransitions.power = state.init.eom.powerTransitions.time;

    for i = 1:state.init.eom.numberOfBeams
        %Actually load the data...
        state.init.eom.powerTransitions.power(i, :) = state.init.eom.powerTransitions.protocols(state.init.eom.powerTransitions.currentProtocol, i, 1, :);
        state.init.eom.powerTransitions.time(i, :) = state.init.eom.powerTransitions.protocols(state.init.eom.powerTransitions.currentProtocol, i, 2, :);

        %Count the valid transitions.
        count = 0;
        if state.init.eom.powerTransitions.power(i, 1) ~= -1
           count = ...
                length(find(state.init.eom.powerTransitions.protocols(state.init.eom.powerTransitions.currentProtocol, i, 1, :) ~= -1));
        end

        if length(count) == 0
            state.init.eom.powerTransitions.transtionCount(i) = size(state.init.eom.powerTransitions.protocols, 4);
        elseif length(count) == 1
            state.init.eom.powerTransitions.transitionCount(i) = count;
        else
            state.init.eom.powerTransitions.transitionCount(i) = count(1);
        end
    end

    %Let things progress as normal...
    %Automatically select beam #1 when loading a protocol, right?
    state.init.eom.currentPowerTransitionBeam = 1;
    updateGUIByGlobal('state.init.eom.currentPowerTransitionBeam');%Why didn't this work properly?
    beamMenu_Callback(gh.powerTransitions.beamMenu, [], gh.powerTransitions);
    powerTransitions('useBinaryTransitions_Callback', gh.powerTransitions.useBinaryTransitions);

    return;

% --------------------------------------------------------------------
function varargout = addProtocol_Callback(h, eventdata, handles, varargin)
global state gh;

    initializeProtocols;

    set(gh.powerTransitions.deleteProtocol, 'Enable', 'on');
        
    numOfProtocols = size(state.init.eom.powerTransitions.protocols, 1) + 1;
    if numOfProtocols == 1
        numOfProtocols = 2;
    end

    %Clear out the variables.
    state.init.eom.powerTransitions.time = -1 * ones(state.init.eom.numberOfBeams, size(state.init.eom.powerTransitions.protocols, 4));
    state.init.eom.powerTransitions.power = state.init.eom.powerTransitions.time;
    state.init.eom.powerTransitions.protocols(numOfProtocols, :, :, :) = -1;
    
    %Update the list of protocols.
    protocolList = get(gh.powerTransitions.protocol, 'String');
    if strcmp(class(protocolList), 'char')
        protocolList = cellstr(protocolList);
    end
    protocolList(numOfProtocols) = cellstr(sprintf('Protocol %s',  num2str(numOfProtocols)));
    set(gh.powerTransitions.protocol, 'String', protocolList);

    state.init.eom.powerTransitions.transitionCount = 0;
    state.init.eom.powerTransitions.currentProtocol = numOfProtocols;
    
    %Save the changes.
    updateStrings;
    
    %Automatically select beam #1 when adding a protocol, right?
    state.init.eom.powerTransitions.currentProtocol = numOfProtocols;
    updateGUIByGlobal('state.init.eom.powerTransitions.currentProtocol');%Why didn't this work???
    protocol_Callback(gh.powerTransitions.protocol, [], gh.powerTransitions);

    return;

% --------------------------------------------------------------------
function varargout = deleteProtocol_Callback(h, eventdata, handles, varargin)
global state gh;

    initializeProtocols;

    numOfProtocols = size(state.init.eom.powerTransitions.protocols, 1);

    if numOfProtocols < 3
        set(gh.powerTransitions.deleteProtocol, 'Enable', 'off');

        if numOfProtocols == 1
            return;%There will always be at least 1 protocol.
        end
    end

    %Shift things down.
    if state.init.eom.powerTransitions.currentProtocol < numOfProtocols
        state.init.eom.powerTransitions.protocols(state.init.eom.powerTransitions.currentProtocol:(numOfProtocols - 1), :, :, :) = ...
            state.init.eom.powerTransitions.protocols((state.init.eom.powerTransitions.currentProtocol + 1):numOfProtocols, :, :, :);
    end

    %Trim it down...
    state.init.eom.powerTransitions.protocols = state.init.eom.powerTransitions.protocols(1:(numOfProtocols - 1), :, :, :);

    %Select the next in line.
    if state.init.eom.powerTransitions.currentProtocol > numOfProtocols - 1
        state.init.eom.powerTransitions.currentProtocol = numOfProtocols - 1;
    end

    %Remove one item from the menu.
    protocolList = get(gh.powerTransitions.protocol, 'String');
    protocolList = protocolList(1 : (length(protocolList)-1));
    set(gh.powerTransitions.protocol, 'String', protocolList);

    updateGUIByGlobal('state.init.eom.powerTransitions.currentProtocol');%Why didn't this work???
    protocol_Callback(gh.powerTransitions.protocol, [], gh.powerTransitions);

    %Save the changes.
    updateStrings;

    return;

% --------------------------------------------------------------------
function varargout = SyncToPhys_Callback(h, eventdata, handles, varargin)
global state gh;

    children = get(gh.powerTransitions.Options, 'Children');
    index = getPullDownMenuIndex(gh.powerTransitions.Options, 'SyncToPhysiology');
    checked = get(children(index), 'Checked');

    if strcmpi(checked, 'On')
        set(children(index), 'Checked', 'off');
        state.init.eom.powerTransitions.syncToPhysiology = 0;
    else
        set(children(index), 'Checked', 'on');
        state.init.eom.powerTransitions.syncToPhysiology = 1;
    end


    return;
    
% --------------------------------------------------------------------
function varargout = transitionVisibility(on)
global gh;

if on
    %There's definitely something to delete, now.
    set(gh.powerTransitions.delete, 'Enable', 'On');
    set(gh.powerTransitions.delete, 'Visible', 'On');
    set(gh.powerTransitions.currentPowerTransition, 'Visible', 'On');

    %The time/power can now be associated with a transition.
    set(gh.powerTransitions.time, 'Enable', 'On');
    set(gh.powerTransitions.power, 'Enable', 'On');
    set(gh.powerTransitions.time, 'Visible', 'On');
    set(gh.powerTransitions.power, 'Visible', 'On');
else
    %There's nothing to delete.
    set(gh.powerTransitions.delete, 'Enable', 'Off');
    set(gh.powerTransitions.delete, 'Visible', 'Off');
    set(gh.powerTransitions.currentPowerTransition, 'Visible', 'Off');

    %The time/power can't be associated with a transition.
    set(gh.powerTransitions.time, 'Enable', 'Off');
    set(gh.powerTransitions.power, 'Enable', 'Off');
    set(gh.powerTransitions.time, 'Visible', 'Off');
    set(gh.powerTransitions.power, 'Visible', 'Off');
end

% --------------------------------------------------------------------
function loadProtocolsFromString
global state gh;

if isempty(state.init.eom.powerTransitions.protocolString) | ...
        ~strcmpi(class(state.init.eom.powerTransitions.protocolString), 'char');
    return;
end

%Some standard.ini files have the state.init.eom.powerTransitions.protocols variable saved
%this is done as a string, and doesn't really work (nor should it... yet).
%In order to prevent Matlab from complaining, make sure that the variable is an empty array
%of type double. -- Tim O'Connor 12/15/03
if strcmpi(class(state.init.eom.powerTransitions.protocols), 'char')
    state.init.eom.powerTransitions.protocols = [];
end

protocols = strread(state.init.eom.powerTransitions.protocolString, '%s', 'delimiter', '!');%Parse out into protocols.
protocolNames = {};
for i = 1 : length(protocols)
    
    protocolNames{i} = sprintf('Protocol %s', num2str(i));
    matrices = strread(protocols{i} , '%s', 'delimiter', ':');%Parse out into beams.
    
    for j = 1 : state.init.eom.numberOfBeams
        state.init.eom.powerTransitions.protocols(i, j, 1, :) = str2num(matrices{2 * j - 1});%Power
        state.init.eom.powerTransitions.protocols(i, j, 2, :) = str2num(matrices{2 * j});%Time
    end
    
end

set(gh.powerTransitions.protocol, 'String', protocolNames);

if size(state.init.eom.powerTransitions.protocols) > 0
    state.init.eom.currentPowerTransition = 1;
    transitionVisibility(1);
    updateGUIByGlobal('state.init.eom.currentPowerTransition');
    currentPowerTransition_Callback(gh.powerTransitions.currentPowerTransition);
else
    transitionVisibility(0);
end

return;
