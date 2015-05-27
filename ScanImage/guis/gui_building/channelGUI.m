function varargout = channelGUI(varargin)
global state
% CHANNELGUI Application M-file for channelGUI.fig
%    FIG = CHANNELGUI launch channelGUI GUI.
%    CHANNELGUI('callback_name', ...) invoke the named callback.

% Last Modified by GUIDE v2.5 19-Nov-2012 15:55:17

if nargin == 0  % LAUNCH GUI

	fig = openfig(mfilename,'reuse');

	% Use system color scheme for figure:
	set(fig,'Color',get(0,'defaultUicontrolBackgroundColor'));

	% Generate a structure of handles to pass to callbacks, and store it. 
	handles = guihandles(fig);
	guidata(fig, handles);

	if nargout > 0
		varargout{1} = fig;
%%
	end

elseif ischar(varargin{1}) % INVOKE NAMED SUBFUNCTION OR CALLBACK

	try
		[varargout{1:nargout}] = feval(varargin{:}); % FEVAL switchyard
    catch ME %VI101910A
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



% --------------------------------------------------------------------
function varargout = cbAcquire1_Callback(h, eventdata, handles, varargin)
% Stub for Callback of the uicontrol handles.cbSave2.
% updates all the checkboxes accross a row when you acquire one channel.
global gh state
state.internal.channelChanged=1;

val = get(h, 'Value');
	if val == 1
		state.acq.savingChannel1 = 1;
		updateGUIByGlobal('state.acq.savingChannel1');
		state.acq.imagingChannel1 = 1;
		updateGUIByGlobal('state.acq.imagingChannel1');
        %%%VI120610A: Removed %%%
        %         state.acq.maxImage1 = 1;
        % 		updateGUIByGlobal('state.acq.maxImage1');
        %%%%%%%%%%%%%%%%%%%%%%%%%%
		updateNumberOfChannels;
		
	elseif val == 0 
		state.acq.savingChannel1 = 0;
		updateGUIByGlobal('state.acq.savingChannel1');
		state.acq.imagingChannel1 = 0;
		updateGUIByGlobal('state.acq.imagingChannel1');
        state.acq.maxImage1 = 0;
		updateGUIByGlobal('state.acq.maxImage1');
		updateNumberOfChannels;
	else
	end
genericCallbackLocal(h);

% --------------------------------------------------------------------
function varargout = cbAcquire2_Callback(h, eventdata, handles, varargin)
% Stub for Callback of the uicontrol handles.cbImage1.
% updates all the checkboxes accross a row when you acquire one channel.
global gh state
state.internal.channelChanged=1;

val = get(h, 'Value');
if val == 1
    state.acq.savingChannel2 = 1;
    updateGUIByGlobal('state.acq.savingChannel2');
    state.acq.imagingChannel2 = 1;
    updateGUIByGlobal('state.acq.imagingChannel2');
    state.acq.focusingChannel2 = 1;
    %     state.acq.maxImage2 = 1;
    % 	updateGUIByGlobal('state.acq.maxImage2');
    updateNumberOfChannels;
elseif val == 0
    state.acq.savingChannel2 = 0;
    updateGUIByGlobal('state.acq.savingChannel2');
    state.acq.imagingChannel2 = 0;
    updateGUIByGlobal('state.acq.imagingChannel2');
    state.acq.maxImage2 = 0;
	updateGUIByGlobal('state.acq.maxImage2');
    updateNumberOfChannels;
else
end
genericCallbackLocal(h)

% --------------------------------------------------------------------
function varargout = cbAcquire3_Callback(h, eventdata, handles, varargin)
% Stub for Callback of the uicontrol handles.cbImage2.
% updates all the checkboxes accross a row when you acquire one channel.
global gh state
state.internal.channelChanged=1;

val = get(h, 'Value');
if val == 1
    state.acq.savingChannel3 = 1;
    updateGUIByGlobal('state.acq.savingChannel3');
    state.acq.imagingChannel3 = 1;
    updateGUIByGlobal('state.acq.imagingChannel3');
    %     state.acq.maxImage3 = 1;
    % 	updateGUIByGlobal('state.acq.maxImage3');
    updateNumberOfChannels;
elseif val == 0
    state.acq.savingChannel3 = 0;
    updateGUIByGlobal('state.acq.savingChannel3');
    state.acq.imagingChannel3 = 0;
    updateGUIByGlobal('state.acq.imagingChannel3');
    state.acq.maxImage3 = 0;
	updateGUIByGlobal('state.acq.maxImage3');
    updateNumberOfChannels;
else
end
genericCallbackLocal(h)

%--------------------------------------------------------------------
function cbAcquire4_Callback(h, eventdata, handles, varargin)
% Stub for Callback of the uicontrol handles.cbAcquire4.
% updates all the checkboxes accross a row when you acquire one channel.
global gh state
state.internal.channelChanged=1;

val = get(h, 'Value');
if val == 1
    state.acq.savingChannel4 = 1;
    updateGUIByGlobal('state.acq.savingChannel4');
    state.acq.imagingChannel4 = 1;
    updateGUIByGlobal('state.acq.imagingChannel4');
    %     state.acq.maxImage4 = 1;
    % 	updateGUIByGlobal('state.acq.maxImage4');
    updateNumberOfChannels;
elseif val == 0
    state.acq.savingChannel4 = 0;
    updateGUIByGlobal('state.acq.savingChannel4');
    state.acq.imagingChannel4 = 0;
    updateGUIByGlobal('state.acq.imagingChannel4');
    state.acq.maxImage4 = 0;
	updateGUIByGlobal('state.acq.maxImage4');
    updateNumberOfChannels;
else
end
genericCallbackLocal(h)

% --------------------------------------------------------------------
function pmVoltageRange1_Callback(h, eventdata, handles)
global state
state.internal.channelChanged = 1;
genericCallbackLocal(h);

% --------------------------------------------------------------------
function pmVoltageRange2_Callback(h, eventdata, handles)
global state
state.internal.channelChanged = 1;
genericCallbackLocal(h);

% --------------------------------------------------------------------
function pmVoltageRange3_Callback(h, eventdata, handles)
global state
state.internal.channelChanged = 1;
genericCallbackLocal(h);

% --------------------------------------------------------------------
function pmVoltageRange4_Callback(h, eventdata, handles)
global state
state.internal.channelChanged = 1;
genericCallbackLocal(h);

% --------------------------------------------------------------------
function cbMergeChannel_Callback(h, eventdata, handles)
genericCallback(h);

%%%VI011109A: Removed %%%%%%%%%%%%%%
% if get(h,'Value') %turn on color merge    
%     set(state.internal.MergeFigure,'Visible','on');
%     set(gh.channelGUI.cbMergeFocusOnly,'Enable','on');     
%     set(gh.channelGUI.cbMergeBlueAsGray,'Enable','on'); %VI111708A
% else %turn off color merge
%     set(state.internal.MergeFigure,'Visible','off');
%     set(gh.channelGUI.cbMergeFocusOnly,'Enable','off');
%     set(gh.channelGUI.cbMergeBlueAsGray,'Enable','off'); %VI111708A
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  


% --------------------------------------------------------------------
function cbMergeFocusOnly_Callback(h, eventdata, handles)
genericCallback(h);


% --------------------------------------------------------------------
function pmMergeColor1_Callback(h, eventdata, handles)
genericCallback(h);

% --------------------------------------------------------------------
function pmMergeColor2_Callback(h, eventdata, handles)
genericCallback(h);

% --------------------------------------------------------------------
function pmMergeColor3_Callback(h, eventdata, handles)
genericCallback(h);

% --------------------------------------------------------------------
function pmMergeColor4_Callback(h, eventdata, handles)
genericCallback(h);


% --------------------------------------------------------------------
function pbSaveCFG_Callback(hObject, eventdata, handles)
saveCurrentConfig();

% --------------------------------------------------------------------
function pbSaveUSR_Callback(hObject, eventdata, handles)
saveCurrentUserSettings();

function cbMax1_Callback(h, eventdata, handles)
global state
state.internal.channelChanged = 1;
genericCallbackLocal(h);

function cbMax2_Callback(h, eventdata, handles)
global state
state.internal.channelChanged = 1;
genericCallbackLocal(h);

function cbMax3_Callback(h, eventdata, handles)
global state
state.internal.channelChanged = 1;
genericCallbackLocal(h);

function cbMax4_Callback(h, eventdata, handles)
global state
state.internal.channelChanged = 1;
genericCallbackLocal(h);

% --------------------------------------------------------------------
% function figure1_CloseRequestFcn(hObject, eventdata, handles)
% global state
% 
% if state.internal.channelChanged == 1;
% 	hideGUI('gh.channelGUI.figure1');
% 	applyChannelSettings;
% else
% 	hideGUI('gh.channelGUI.figure1');
% 	state.internal.channelChanged=0;
% end
% 	
% updateChannelMergeParameters();

function textColormap1_Callback(h, eventdata, handles)
genericCallback(h);

function textColormap2_Callback(h, eventdata, handles)
genericCallback(h);

function textColormap3_Callback(h, eventdata, handles)
genericCallback(h);

function textColormap4_Callback(h, eventdata, handles)
genericCallback(h);

% function pbAdvanced_Callback(h, eventdata, handles)
% offset = 30; % the horizontal size by which to grow the figure
% toggleAdvancedPanel(h,offset,'x');

function genericCallbackLocal(h)
% override genericCallback to update changes immediately after GUI field change
global state

genericCallback(h);

if state.internal.channelChanged == 1 && ~state.internal.loading
	applyChannelSettings;
end
%updateChannelMergeParameters();
figure(ancestor(h,'figure'));


% --- Executes on selection change in pmImageColormap.
function pmImageColormap_Callback(hObject, eventdata, handles)
% hObject    handle to pmImageColormap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns pmImageColormap contents as cell array
%        contents{get(hObject,'Value')} returns selected item from pmImageColormap
genericCallback(hObject);

% --- Executes during object creation, after setting all properties.
function pmImageColormap_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmImageColormap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
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
hideGUI(hObject);


%% PMT OFFSETS

function etPMTOffset1_Callback(hObject, eventdata, handles)
% hObject    handle to etPMTOffset1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etPMTOffset1 as text
%        str2double(get(hObject,'String')) returns contents of etPMTOffset1 as a double


% --- Executes during object creation, after setting all properties.
function etPMTOffset1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPMTOffset1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function etPMTOffset2_Callback(hObject, eventdata, handles)
% hObject    handle to etPMTOffset2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etPMTOffset2 as text
%        str2double(get(hObject,'String')) returns contents of etPMTOffset2 as a double


% --- Executes during object creation, after setting all properties.
function etPMTOffset2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPMTOffset2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function etPMTOffset3_Callback(hObject, eventdata, handles)
% hObject    handle to etPMTOffset3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etPMTOffset3 as text
%        str2double(get(hObject,'String')) returns contents of etPMTOffset3 as a double


% --- Executes during object creation, after setting all properties.
function etPMTOffset3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPMTOffset3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function etPMTOffset4_Callback(hObject, eventdata, handles)
% hObject    handle to etPMTOffset4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etPMTOffset4 as text
%        str2double(get(hObject,'String')) returns contents of etPMTOffset4 as a double


% --- Executes during object creation, after setting all properties.
function etPMTOffset4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etPMTOffset4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pbGetPMTOffsets.
function pbGetPMTOffsets_Callback(hObject, eventdata, handles)
% hObject    handle to pbGetPMTOffsets (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
startPMTOffsets();

% --- Executes on button press in cbAutoReadPMTOffsets.
function cbAutoReadPMTOffsets_Callback(hObject, eventdata, handles)
% hObject    handle to cbAutoReadPMTOffsets (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbAutoReadPMTOffsets
genericCallback(hObject);


% --- Executes on button press in cbAutoSubtractPMTOffset1.
function cbAutoSubtractPMTOffset1_Callback(hObject, eventdata, handles)
% hObject    handle to cbAutoSubtractPMTOffset1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbAutoSubtractPMTOffset1
genericCallback(hObject);

% --- Executes on button press in cbAutoSubtractPMTOffset2.
function cbAutoSubtractPMTOffset2_Callback(hObject, eventdata, handles)
% hObject    handle to cbAutoSubtractPMTOffset2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbAutoSubtractPMTOffset2
genericCallback(hObject);

% --- Executes on button press in cbAutoSubtractPMTOffset3.
function cbAutoSubtractPMTOffset3_Callback(hObject, eventdata, handles)
% hObject    handle to cbAutoSubtractPMTOffset3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbAutoSubtractPMTOffset3
genericCallback(hObject);


% --- Executes on button press in cbAutoSubtractPMTOffset4.
function cbAutoSubtractPMTOffset4_Callback(hObject, eventdata, handles)
% hObject    handle to cbAutoSubtractPMTOffset4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbAutoSubtractPMTOffset4
genericCallback(hObject);


% --- Executes on button press in cbSave1.
function cbSave1_Callback(hObject, eventdata, handles)
% hObject    handle to cbSave1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbSave1
updateChannelSaveOrImage(hObject,1);

% --- Executes on button press in cbSave2.
function cbSave2_Callback(hObject, eventdata, handles)
% hObject    handle to cbSave2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbSave2
updateChannelSaveOrImage(hObject,2);

% --- Executes on button press in cbSave3.
function cbSave3_Callback(hObject, eventdata, handles)
% hObject    handle to cbSave3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbSave3
updateChannelSaveOrImage(hObject,3);

% --- Executes on button press in cbSave4.
function cbSave4_Callback(hObject, eventdata, handles)
% hObject    handle to cbSave4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbSave4
updateChannelSaveOrImage(hObject,4);

% --- Executes on button press in cbImage1.
function cbImage1_Callback(hObject, eventdata, handles)
% hObject    handle to cbImage1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbImage1
updateChannelSaveOrImage(hObject,1);

% --- Executes on button press in cbImage2.
function cbImage2_Callback(hObject, eventdata, handles)
% hObject    handle to cbImage2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbImage2
updateChannelSaveOrImage(hObject,2);

% --- Executes on button press in cbImage3.
function cbImage3_Callback(hObject, eventdata, handles)
% hObject    handle to cbImage3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbImage3
updateChannelSaveOrImage(hObject,3);

% --- Executes on button press in cbImage4.
function cbImage4_Callback(hObject, eventdata, handles)
% hObject    handle to cbImage4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbImage4
updateChannelSaveOrImage(hObject,4);

%% HELPER FUNCTIONS
function updateChannelSaveOrImage(hObject,chanNumber)
global state
state.internal.channelChanged = 1;
if get(hObject,'Value') == 1
    acqChanString = sprintf('acquiringChannel%d',chanNumber);
    state.acq.(acqChanString) = 1;
    updateGUIByGlobal(sprintf('state.acq.%s',acqChanString));
    updateNumberOfChannelsAcquire();
end
genericCallbackLocal(hObject);


