function varargout = powerControl(varargin)
global state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EOM_GUI Application M-file for eom_gui.fig
%    FIG = EOM_GUI launch eom_gui GUI.
%    EOM_GUI('callback_name', ...) invoke the named callback.
% 
% Last Modified by GUIDE v2.5 30-Aug-2011 20:51:11
%
% Changes:
%   TPMOD_1: Modified 12/31/03 Tom Pologruto - Added checkbox to set
%   whether or not to poll the photodiode upon power change.
%   TO21804a Tim O'Connor 2/18/04 - Allow power box to work in mW.
%   TO21804d Tim O'Connor 2/18/04 - Added some sensible/useful error messages.
%   TO22004a Tim O'Connor 2/18/04 - Fix missing variables, due to bad config loading.
%   TO22704a Tim O'Connor 2/27/04 - Created uncagingMapper.
%   TO061604a Tim O'Connor 6/16/04 - Updated to force it to overlap only 1 line on the image,
%   TO061604b Tim O'Connor 6/16/04 - Added in a half pixel correction (positive in X, negative in Y), to get the box to directly overlap the signal.
%   VI092208A Vijay Iyer 9/22/08 - Remove premature rounding and pixel 'corrections' for PowerBox. Downsize PowerBox rectangle thickness to 2.
%   VI102008A Vijay Iyer 10/20/08 - Moved beamMenu_Callback() code to shared callback updateBeamSelection()
%   VI102008B Vijay Iyer 10/20/08 - Eliminate use of scanLaserBeam state variable
%   VI111108A Vijay Iyer 11/11/08 - Move 'showBox' control to the Power Box area and add a 'showPowerBox' control
%   VI021009A Vijay Iyer 02/10/09 - Show/Hide PowerBox GUI via toggle button; use tetherGUIs() to determine initial relative position
%   VI051109A Vijay Iyer 05/11/09 - Use tetherGUIs undconditionally
%   VI112309A Vijay Iyer 11/23/09 - Add option for 'directMode' specifying when power level changes should update the power level immediately, even though not scanning
%   VI112309B Vijay Iyer 11/23/09 - Remove reference to defunct 'powerBoxText' text control
%   VI010810A Vijay Iyer 01/08/10 - Enable P vs Z is now a scalar, i.e. applies to all beams, not per-beam
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin == 0  % LAUNCH GUI

	fig = openfig(mfilename,'reuse');


	% Generate a structure of handles to pass to callbacks, and store it. 
	handles = guihandles(fig);
	guidata(fig, handles);
    
	if nargout > 0
		varargout{1} = fig;
    end
    
    %%%VI120108A%%%%%%%%%%%%%%%%%%%%%%
    set(fig,'KeyPressFcn',@genericKeyPressFunction);
    %Ensure all children respond to key presses, when they have the focus (for whatever reason)
    kidControls = findall(fig,'Type','uicontrol');
    for i=1:length(kidControls)
        if ~strcmpi(get(kidControls(i),'Style'),'edit')
            set(kidControls(i),'KeyPressFcn',@genericKeyPressFunction);
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
elseif ischar(varargin{1}) % INVOKE NAMED SUBFUNCTION OR CALLBACK

	try
		if (nargout)
			[varargout{1:nargout}] = feval(varargin{:}); % FEVAL switchyard
		else
			feval(varargin{:}); % FEVAL switchyard
		end
    catch ME %VI112210A
        most.idioms.reportError(ME);
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

function msg = debug(handles)
%global state;

%msg = sprintf('\nmaxPower_Slider:\n Max=%2.0f\n Min=%2.0f\n Val=%2.0f\nmaxLimit_Slider:\n Max=%2.0f\n Min=%2.0f\n Val=%2.0f\neom.maxPower=%2.0f\neom.min=%2.0f\neom.maxLimit=%2.0f\n', ...
 %   get(handles.maxPower_Slider, 'Max'), get(handles.maxPower_Slider, 'Min'), get(handles.maxPower_Slider, 'Value'), ...
  %  get(handles.maxLimit_Slider, 'Max'), get(handles.maxLimit_Slider, 'Min'), get(handles.maxLimit_Slider, 'Value'), ...
   % state.init.eom.maxPower, state.init.eom.min, state.init.eom.maxLimit);

return;

% --------------------------------------------------------------------
% pre - Calibrated.
% post - state.init.eom.maxPower(i) = maxPower_Slider.Value
%        <ensureState>
function varargout = maxPower_Slider_Callback(h, eventdata, handles, varargin)
global state gh

genericCallback(h);

beamIdx = state.init.eom.beamMenu;
state.init.eom.maxPower(beamIdx) = round(state.init.eom.maxPowerDisplaySlider);
state.init.eom.changed(beamIdx) = 1;
ensureEomGuiStates;

%%%VI112010A%%%%%%
if state.init.eom.allowSliderDuringFocus && strcmp(get(gh.mainControls.focusButton, 'String'),'ABORT') % focusing now....
    if ~state.init.eom.hAO.isTaskDoneQuiet()
        state.init.eom.hAO.abort();
        state.init.eom.hAO.control('DAQmx_Val_Task_Unreserve');
    end
    
    voltageLevel = state.init.eom.lut(beamIdx,state.init.eom.maxPower(beamIdx));
    state.init.eom.(sprintf('hAOPark%d',beamIdx)).writeAnalogData(voltageLevel,.05);
end
%%%%%%%%%%%%%%%%%%


% --------------------------------------------------------------------
% pre - Calibrated.
% post - state.init.eom.maxPower(i) = maxPower_Text.Value
%        <ensureState>
function varargout = maxPowerText_Callback(h, eventdata, handles, varargin)
global state gh

genericCallback(h);

if get(gh.powerControl.mW_radioButton, 'Value') == get(gh.powerControl.mW_radioButton, 'Max')    %in mW 
    conversion = (getfield(state.init.eom, ['powerConversion' num2str(state.init.eom.beamMenu)]) * ...
        state.init.eom.maxPhotodiodeVoltage(state.init.eom.beamMenu) * .01);       
    state.init.eom.maxPower(state.init.eom.beamMenu) = round(1 / conversion * state.init.eom.maxPowerDisplay);
else
    state.init.eom.maxPower(state.init.eom.beamMenu) = round(state.init.eom.maxPowerDisplay);
end

state.init.eom.changed(state.init.eom.beamMenu) = 1;
ensureEomGuiStates;
setScanProps(h);

% --------------------------------------------------------------------
% pre - Calibrated.
% post - state.init.eom.maxLimit(i) = maxLimit.String
%        <ensureState>
function varargout = maxLimit_Callback(h, eventdata, handles, varargin)
global state gh;

    set(h, 'String', num2str(round(str2num(get(h, 'String')))));
    state.init.eom.maxLimit(state.init.eom.beamMenu) = str2num(get(h, 'String'));
    ensureEomGuiStates;
    
    if state.init.eom.changed(state.init.eom.beamMenu)
        setScanProps(h);
    end

% --------------------------------------------------------------------
% pre - Calibrated.
% post - state.init.eom.maxLimit(i) = maxLimit_Slider.Value
%        <ensureState>
function varargout = maxLimit_Slider_Callback(h, eventdata, handles, varargin)
global state gh;

state.init.eom.maxLimit(state.init.eom.beamMenu) = round(get(h, 'Value'));
ensureEomGuiStates;
if state.init.eom.changed(state.init.eom.beamMenu)
    setScanProps(h);
end

% --------------------------------------------------------------------
% pre - Calibrated.
%       power to voltage conversion is configured
% post - power readout is in mW units
%        <ensureState>
function varargout = mW_radioButton_Callback(h, eventdata, handles, varargin)
global state gh
    val = get(h, 'Value');
    set(h,'Enable','inactive');
    
    if val == get(h, 'Max')
        set(gh.powerControl.percent_radioButton, 'Value', get(h, 'Min'),'Enable','on');
    else
        set(gh.powerControl.percent_radioButton, 'Value', get(h, 'Max'),'Enable','on');        
    end
    
    %set(gh.powerControl.powerBoxText, 'String', 'Power [mW]'); %VI112309B
    
    state.init.eom.powerInMw = 1;
    
    ensureEomGuiStates;
    
    return;

% --------------------------------------------------------------------
% pre - Calibrated.
% post - power readout is in % units
%        <ensureState>
function varargout = percent_radioButton_Callback(h, eventdata, handles, varargin)
global state gh
    val = get(h, 'Value');
    set(h,'Enable','inactive');
    if val == get(h, 'Max')
        set(gh.powerControl.mW_radioButton, 'Value', get(h, 'Min'),'Enable','on');
    else
        set(gh.powerControl.mW_radioButton, 'Value', get(h, 'Max'),'Enable','on');
    end
    
    %Added to allow power box to work in mW. -- Tim O'Connor TO21804a
    %set(gh.powerControl.powerBoxText, 'String', 'Power [%]'); %VI112309B
    
    state.init.eom.powerInMw = 0;
    
    ensureEomGuiStates;
    
    return;

% --------------------------------------------------------------------
% pre - Calibrated.
% post - controls for the appropriate beamMenu are displayed
function varargout = beamMenu_Callback(h, eventdata, handles, varargin)
global state gh;
%     previous = state.init.eom.beamMenu;

    genericCallback(h);

%%%VI102008A: Following code has been moved to updateBeamSelection() shared callback    
%     set(gh.powerControl.boxConstrainBox, 'Value', state.init.eom.constrainBoxToLine(state.init.eom.beamMenu));
% %     state.init.eom.showBox = state.init.eom.showBoxArray(state.init.eom.beamMenu);
% %     updateGUIByGlobal('state.init.eom.showBox');
% %     state.init.eom.boxPower = state.init.eom.boxPowerArray(state.init.eom.beamMenu);
% %     updateGUIByGlobal('state.init.eom.boxPower');    
% %     state.init.eom.startFrame = state.init.eom.startFrameArray(state.init.eom.beamMenu);
% %     updateGUIByGlobal('state.init.eom.startFrame');    
% %     state.init.eom.endFrame = state.init.eom.endFrameArray(state.init.eom.beamMenu);
% %     updateGUIByGlobal('state.init.eom.endFrame');
% 
%     ensureEomGuiStates;
% 
%     state.init.eom.beamMenuSlider = state.init.eom.numberOfBeams - state.init.eom.beamMenu + 1;
%     updateGUIByGlobal('state.init.eom.beamMenuSlider');
%     set(gh.powerControl.beamMenuSlider, 'Value', state.init.eom.beamMenuSlider);
% 
%     updatePowerGUI(state.init.eom.beamMenu);
% 
% %     %The powerbox or custom timings may need recalculation.
% %     state.init.eom.changed(state.init.eom.beamMenu) = 1;
% %     state.init.eom.changed(previous) = 1;

% --------------------------------------------------------------------
function varargout = usePowerArray_Callback(h, eventdata, handles, varargin)
genericCallback(h);
global state gh
state.init.eom.changed(state.init.eom.beamMenu)=1;

if state.init.eom.usePowerArray
    set(get(gh.powerTransitions.figure1,'Children'),'Enable','On');
end

% --------------------------------------------------------------------
function varargout = beamMenuSlider_Callback(h, eventdata, handles, varargin)
global state gh;

    genericCallback(h);

    %Keep things within bounds.
    if state.init.eom.beamMenuSlider > state.init.eom.numberOfBeams
        state.init.eom.beamMenuSlider = state.init.eom.numberOfBeams;
    elseif state.init.eom.beamMenuSlider < 1
        state.init.eom.beamMenuSlider = 1;
    end

    %Invert the slider, so that Beam2 is graphically below Beam1, to match the popup menu behavior.
    state.init.eom.beamMenu = state.init.eom.numberOfBeams - state.init.eom.beamMenuSlider + 1;

    updateGUIByGlobal('state.init.eom.beamMenu'); %Again, why doesn't this ever work properly???

    set(gh.powerControl.beamMenu, 'Value', state.init.eom.beamMenu);
    powerControl('beamMenu_Callback', gh.powerControl.beamMenu);



% --- Executes during object creation, after setting all properties.
function boxWidth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to boxWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% -----------------VI111108A----------------------------------------
function tbShowPowerBox_Callback(hObject, eventdata, handles)

global state
 
%%%VI021009A%%%%%%%%%
if get(hObject,'Value')
    tetherGUIs('powerControl','powerBox','rightcenter'); %VI051109A
    seeGUI('gh.powerBox.figure1'); %VI011709A
    set(hObject,'String', '<<');
else
    hideGUI('gh.powerBox.figure1');
    set(hObject,'String', '>>');
end
%%%%%%%%%%%%%%%%%%%%%%


% ------------------------VI112309A------------------------------------
function cbDirectMode_Callback(hObject, eventdata, handles) 
genericCallback(hObject);

%For all beams, update Pockels signal to min or specified power level depending on new directMode state
global state
state.init.eom.changed(1:state.init.eom.numberOfBeams);
ensureEomGuiStates(1:state.init.eom.numberOfBeams);





%%VI011709A: Removed %%%%%%%%%
% checked = get(hObject,'Value');
% 
% children = get(gh.powerControl.Settings, 'Children');
% index = getPullDownMenuIndex(gh.powerControl.Settings, 'Show Power Box');
% 
% if checked
%     set(children(index), 'Checked', 'off'); %set to opposite of what resizePowerControlFigure() will set it to
%     set(hObject,'String','Hide <<');
% else
%     set(children(index), 'Checked', 'on'); %set to opposite of what resizePowerControlFigure() will set it to
%     set(hObject,'String','Power Box >>');
% end
% 
% resizePowerControlFigure();
%%%%%%%%%%%%%%%%%%%%%%

% --------------------------------------------------------------------
function etZLengthConstant_Callback(hObject, eventdata, handles)
global state
genericCallback(hObject); %Handles value checking
state.init.eom.powerLzStoredArray(state.init.eom.beamMenu) = state.init.eom.powerLz; 


% --------------------------------------------------------------------
function cbEnablePvsZ_Callback(hObject, eventdata, handles)
global state
genericCallback(hObject); %VI010810A
%state.init.eom.powerVsZEnableArray(state.init.eom.beamMenu) = get(hObject,'Value'); %VI010810A

% --------------------------------------------------------------------
function cbSliderDuringFocus_Callback(hObject, eventdata, handles)
genericCallback(hObject);

% --------------------------------------------------------------------
function tbRecordPvsZ_Callback(hObject, eventdata, handles)


% --------------------------------------------------------------------
function pbGetPvsZ_Callback(hObject, eventdata, handles)








% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
hideGUI(hObject);

%% CREATE FCNS

% --- Executes during object creation, after setting all properties.
function beamMenuSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to beamMenuSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function beamMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to beamMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function maxPowerText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxPowerText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function maxPower_Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxPower_Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function maxLimit_Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxLimit_Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function maxLimit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxLimit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
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
