function varargout = positionGUI(varargin)
% POSITIONGUI MATLAB code for positionGUI.fig
%      POSITIONGUI, by itself, creates a new POSITIONGUI or raises the existing
%      singleton*.
%
%      H = POSITIONGUI returns the handle to a new POSITIONGUI or the handle to
%      the existing singleton*.
%
%      POSITIONGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in POSITIONGUI.M with the given input arguments.
%
%      POSITIONGUI('Property','Value',...) creates a new POSITIONGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before positionGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to positionGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help positionGUI

% Last Modified by GUIDE v2.5 18-Sep-2012 15:33:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @positionGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @positionGUI_OutputFcn, ...
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


% --- Executes just before positionGUI is made visible.
function positionGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to positionGUI (see VARARGIN)

% Choose default command line output for positionGUI
handles.output = hObject;


%Initialize PropControls
handles.pcPositionTable = most.gui.control.ColumnArrayTable(findobj(hObject,'Tag','tblPosition'),[],6);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes positionGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = positionGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function pbClearAll_Callback(hObject, eventdata, handles)
handles.hModel.roiClearAllPositions();

function pbRemove_Callback(hObject, eventdata, handles)
handles.hController.removePosition_Callback();
set(handles.pbGoto,'Enable','off');
set(handles.pbRemove,'Enable','off');

% --- Executes on key press with focus on tblROI and none of its controls.
function tblROI_KeyPressFcn(hObject, eventdata, handles)

% --- Executes when selected cell(s) is changed in tblPosition.
function tblPosition_CellSelectionCallback(hObject, eventdata, handles)
tbl_CellSelectionCallback(hObject,eventdata,handles);

function tbl_CellSelectionCallback(hObject,eventdata,handles)

global state;

if isempty(eventdata.Indices)
    return; % DEQ20110614 - not sure where these spurious calls are coming from...
end

persistent robot;
if isempty(robot)
	robot = most.testing.Robot();
end

% rudimentary lock mechanism to prevent Robot clicks from triggering callback logic
persistent isLocked;
if isempty(isLocked)
	isLocked = false;
end

if isLocked
	return;
end

tablePosn = getpixelposition(hObject);
figurePosn = getpixelposition(gcf);
leftBound = figurePosn(1) + tablePosn(1) + 15; % MAGICNUMBER: left x offset    
rightBound = leftBound + tablePosn(3) - 55; % MAGICNUMBER: right x offset

% we can use a Robot to simulate mouse clicks, enabling a hackish "row select" behavior
if ~isLocked
	isLocked = true;

	initialMousePosn = get(0,'PointerLocation');
	
	robot.leftRelease(); % necessary; otherwise, the subsequent move is interpreted as a drag-select
	robot.moveAbsolute([leftBound initialMousePosn(2)]);	
	robot.leftClick();
	robot.moveAbsolute([rightBound initialMousePosn(2)]);
	robot.keyPress('shift');
	robot.leftClick();
	robot.keyRelease('shift');	
	robot.moveAbsolute(initialMousePosn);

	drawnow; % flush the queued/skipped events while we still have the lock
	isLocked = false;
end

selectedTableCells('posn',eventdata.Indices);

% multiple selection
if isempty(eventdata.Indices) || size(eventdata.Indices,1) > 1
    set(handles.pbGoto,'Enable','off');
    
% 	tableData = get(handles.(get(hObject,'Tag')),'Data');
% 	rowIndices = unique(eventdata.Indices(:,1));
% 	selectedIDs = zeros(length(rowIndices));
% 	for i = 1:length(rowIndices)
% 		if ~isempty(tableData{rowIndices(i),1})
% 			selectedID = [];
% 		elseif isnan(str2double(tableData{rowIndices(i),1}))
% 			selectedIDs(i) = 0; % 'base' case
% 		else
% 			selectedIDs(i) = str2double(tableData{rowIndices(i),1});
% 		end
% 	end
% 	handles.hController.updateSelectedROI(selectedIDs);
else
    % determine and cache the selected ROI/Position ID
    ROI_ID_COL_IDX = 1;
    ROI_TYPE_COL_IDX = 3;
    row = eventdata.Indices(1);
    tableData = get(handles.(get(hObject,'Tag')),'Data');
    if ~isempty(tableData(row,:))
		if isempty(tableData{row,ROI_ID_COL_IDX})
			selectedID = [];
		elseif isnan(str2double(tableData{row,ROI_ID_COL_IDX}(1))) % '(1)' indexing due to possibility of asterisk being present...
			selectedID = 0; % 'base' case
		else
			selectedID = str2double(tableData{row,ROI_ID_COL_IDX}(1));
		end
        type = tableData{row,ROI_TYPE_COL_IDX};
    else
        selectedID = [];
        type = '';
    end

    if state.motor.motorOn
        handles.hController.updateSelectedPosition(selectedID);
    else
        set(handles.pbGoto,'Enable','off'); 
    end
    
    if ~isempty(selectedID)
        if state.motor.motorOn
            set(handles.pbGoto,'Enable','on');
        end

		if selectedID ~= handles.hModel.ROI_BASE_ID
			set(handles.pbRemove,'Enable','on');
		else
			set(handles.pbRemove,'Enable','off');
		end
    else
        set(handles.pbGoto,'Enable','off'); 
        set(handles.pbRemove,'Enable','off');
    end
end

% --------------------------------------------------------------------
function mnuFile_Callback(hObject, eventdata, handles)
% hObject    handle to mnuFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function mnuSave_Callback(hObject, eventdata, handles)
handles.hModel.roiSaveAs();

% --------------------------------------------------------------------
function mnuLoad_Callback(hObject, eventdata, handles)
handles.hModel.roiLoad();

% --- Executes on button press in pbGoto.
function pbGoto_Callback(hObject, eventdata, handles)
handles.hModel.roiGotoPosition();

% --- Executes on button press in pbSave.
function pbSave_Callback(hObject, eventdata, handles)
handles.hModel.roiSaveAs();

% --- Executes on button press in pbLoad.
function pbLoad_Callback(hObject, eventdata, handles)
handles.hModel.roiLoad();

% --- Executes on button press in tbPositionDisplay.
function tbPositionDisplay_Callback(hObject, eventdata, handles)
if get(hObject,'Value')
    %seeGUI('state.internal.roifigureNew');
else
    %hideGUI('state.internal.roifigureNew');
end

% --- Executes on button press in pbShiftXY.
function pbShiftXY_Callback(hObject, eventdata, handles)
handles.hModel.roiShiftPosition('xy');

% --- Executes on button press in pbShiftXYZ.
function pbShiftXYZ_Callback(hObject, eventdata, handles)
handles.hModel.roiShiftPosition('xyz');

% --- Executes on button press in tbROIGUI.
function tbROIGUI_Callback(hObject, eventdata, handles)
if get(hObject,'Value')
	tetherGUIs('positionGUI','roiGUI','righttop');
else
	hideGUI('gh.roiGUI.figure1')
end

% --- Executes on button press in cbAbsoluteCoords.
function cbAbsoluteCoords_Callback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);



function etToleranceX_Callback(hObject, eventdata, handles)
% hObject    handle to etToleranceX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etToleranceX as text
%        str2double(get(hObject,'String')) returns contents of etToleranceX as a double
handles.hController.updateModel(hObject,eventdata,handles);

% --- Executes during object creation, after setting all properties.
function etToleranceX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etToleranceX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function etToleranceY_Callback(hObject, eventdata, handles)
% hObject    handle to etToleranceY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etToleranceY as text
%        str2double(get(hObject,'String')) returns contents of etToleranceY as a double
handles.hController.updateModel(hObject,eventdata,handles);

% --- Executes during object creation, after setting all properties.
function etToleranceY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etToleranceY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function etToleranceZ_Callback(hObject, eventdata, handles)
% hObject    handle to etToleranceZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etToleranceZ as text
%        str2double(get(hObject,'String')) returns contents of etToleranceZ as a double
handles.hController.updateModel(hObject,eventdata,handles);

% --- Executes during object creation, after setting all properties.
function etToleranceZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etToleranceZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function etToleranceZZ_Callback(hObject, eventdata, handles)
% hObject    handle to etToleranceZZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etToleranceZZ as text
%        str2double(get(hObject,'String')) returns contents of etToleranceZZ as a double
handles.hController.updateModel(hObject,eventdata,handles);

% --- Executes during object creation, after setting all properties.
function etToleranceZZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etToleranceZZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% % --- Executes on button press in tbAdvanced.
% function tbAdvanced_Callback(hObject, eventdata, handles)
% % hObject    handle to tbAdvanced (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hint: get(hObject,'Value') returns toggle state of tbAdvanced
% toggleAdvancedPanel(hObject,4,'y');


% --- Executes on button press in cbIgnoreSecZ.
function cbIgnoreSecZ_Callback(hObject, eventdata, handles)
% hObject    handle to cbIgnoreSecZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbIgnoreSecZ
handles.hController.updateModel(hObject,eventdata,handles);

