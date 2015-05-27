function varargout = roiGUI(varargin)
% ROIGUI MATLAB code for roiGUI.fig
%      ROIGUI, by itself, creates a new ROIGUI or raises the existing
%      singleton*.
%
%      H = ROIGUI returns the handle to a new ROIGUI or the handle to
%      the existing singleton*.
%
%      ROIGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROIGUI.M with the given input arguments.
%
%      ROIGUI('Property','Value',...) creates a new ROIGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before roiGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to roiGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help roiGUI

% Last Modified by GUIDE v2.5 22-Mar-2012 15:58:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @roiGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @roiGUI_OutputFcn, ...
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


% --- Executes just before roiGUI is made visible.
function roiGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to roiGUI (see VARARGIN)

% Choose default command line output for roiGUI
handles.output = hObject;


%Initialize PropControls
handles.pcROITable = most.gui.control.ColumnArrayTable(findobj(hObject,'Tag','tblROI'),[],10);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes roiGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = roiGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function tblROICategoryOptions_CellEditCallback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);


function cbViewAll_Callback(hObject, eventdata, handles)
handles.hModel.viewAll_Callback(hObject,eventdata,handles);


function cbAutoSelectAll_Callback(hObject, eventdata, handles)
handles.hModel.autoSelectAll_Callback(hObject,eventdata,handles);
% DEQ20110429 - not sure why this doesn't work...it seems that 'handles.pcROICategoryOptions' was passed by value...not sure why
% handles.pcROICategoryOptions.setColumn(viewAllColIdx,get(hObject,'Value'));
% state.hSICtl.hROICategoryOptions.setColumn(autoSelectColIdx,logical(get(hObject,'Value')));


function pbClearAll_Callback(hObject, eventdata, handles)
handles.hModel.roiClearAll();
	
function pbRemove_Callback(hObject, eventdata, handles)
handles.hController.removeROI_Callback();
handles.hController.disableROIControlButtons();

function cbShowIDs_Callback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);

% --- Executes on key press with focus on tblROI and none of its controls.
function tblROI_KeyPressFcn(hObject, eventdata, handles)

function tblROI_CellSelectionCallback(hObject,eventdata,handles)

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
rightBound = leftBound + tablePosn(3) - 35; % MAGICNUMBER: right x offset

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

selectedTableCells('roi',eventdata.Indices);

% multiple selection
if isempty(eventdata.Indices) || size(eventdata.Indices,1) > 1
    set(handles.pbShow,'Enable','off');
    
	tableData = get(handles.(get(hObject,'Tag')),'Data');
	rowIndices = unique(eventdata.Indices(:,1));
	selectedIDs = zeros(1,length(rowIndices));
	for i = 1:length(rowIndices)
		if isempty(tableData{rowIndices(i),1})
			selectedIDs = [];
			break;
		elseif isnan(str2double(tableData{rowIndices(i),1}))
			selectedIDs(i) = 0; % 'base' case
		else
			selectedIDs(i) = str2double(tableData{rowIndices(i),1});
		end
	end
	handles.hController.updateSelectedROI(selectedIDs);
else
    % determine and cache the selected ROI/Position ID
    ROI_ID_COL_IDX = 1;
    ROI_TYPE_COL_IDX = 3;
    row = eventdata.Indices(1);
    tableData = get(handles.(get(hObject,'Tag')),'Data');
    if ~isempty(tableData(row,:))
        s = tableData{row,ROI_ID_COL_IDX}; %string value
        
        if isempty(s)
            selectedID = [];
        else
            s(s=='*') = [];
            
            if isnan(str2double(s))
                selectedID = 0; %'base' ROI case
            else
                selectedID = str2double(s);
            end
        end

        type = tableData{row,ROI_TYPE_COL_IDX};
    else
        selectedID = [];
        type = '';
	end
	
	handles.hController.updateSelectedROI(selectedID);
    
	% we just updated the selected ID, but SI3 might have modified it (due to a motor move)
    if ~isempty(handles.hModel.selectedROIID)
		% enable the 'show' button (but only if it isn't a point/line)
		if ismember(type,{'point' 'line'}) || selectedID == handles.hModel.ROI_BASE_ID
			set(handles.pbShow,'Enable','off');
		else
			set(handles.pbShow,'Enable','on');
		end
		
		if selectedID ~= handles.hModel.ROI_BASE_ID
			set(handles.pbRemove,'Enable','on');
		else
			set(handles.pbRemove,'Enable','off');
        end
        
        if ~handles.hModel.roiGotoOnSelect
            set(handles.pbGoto,'Enable','on');
        else
            set(handles.pbGoto,'Enable','off');
        end 
    else
        set(handles.pbShow,'Enable','off'); 
        set(handles.pbRemove,'Enable','off');
        set(handles.pbGoto,'Enable','off');
    end
end

% --------------------------------------------------------------------
function mnuFile_Callback(hObject, eventdata, handles)
% hObject    handle to mnuFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function mnuSettings_Callback(hObject, eventdata, handles)
% hObject    handle to mnuSettings (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function mnuSave_Callback(hObject, eventdata, handles)
handles.hModel.roiSaveAs();

% --------------------------------------------------------------------
function mnuLoad_Callback(hObject, eventdata, handles)
handles.hModel.roiLoad();

% --- Executes on button press in pbShow.
function pbShow_Callback(hObject, eventdata, handles)
handles.hController.roiShow_Callback();

% --- Executes on button press in pbSave.
function pbSave_Callback(hObject, eventdata, handles)
handles.hModel.roiSaveAs();

% --- Executes on button press in pbLoad.
function pbLoad_Callback(hObject, eventdata, handles)
handles.hModel.roiLoad();

% --- Executes on button press in pbroidisplay.
function pbROIDisplay_Callback(hObject, eventdata, handles)
seeGUI('gh.roiDisplayGUI.figure1');

% if get(hObject,'Value')
%     seeGUI('gh.roiDisplayGUI.figure1');
% else
%     hideGUI('gh.roiDisplayGUI.figure1');
% end

% --- Executes on button press in pbMacroMosaic.
function pbMacroMosaic_Callback(hObject, eventdata, handles)
handles.hModel.roiMacroMosaic();

% --- Executes on button press in ppMacroGrid.
function pbMacroGrid_Callback(hObject, eventdata, handles)
handles.hModel.roiMacroGrid();

% --- Executes on button press in pbCopyToCycle.
function pbMacroToCycle_Callback(hObject, eventdata, handles)
handles.hModel.roiMacroCopyToCycle();

% --- Executes on button press in pbUpOneLevel.
function pbUpOneLevel_Callback(hObject, eventdata, handles)
displayedROICached = handles.hModel.shownROI;
handles.hController.roiUpOneLevel_Callback();%true);

if displayedROICached ~= handles.hModel.shownROI
    handles.hController.disableROIControlButtons();
end

function etAngleToMicrons_Callback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);

% --------------------------------------------------------------------
function mnuGotoOnAdd_Callback(hObject, eventdata, handles)
mnuCheckedItem_Callback(hObject, eventdata, handles);
handles.hController.roiGotoOnAdd_Helper(strcmpi(get(hObject,'Checked'),'on'),handles);

% --------------------------------------------------------------------
function mnuGotoOnSelect_Callback(hObject, eventdata, handles)
mnuCheckedItem_Callback(hObject, eventdata, handles);
handles.hController.roiGotoOnSelect_Helper(strcmpi(get(hObject,'Checked'),'on'),handles);

function mnuCheckedItem_Callback(hObject, eventdata, handles)
% Generic callback that handles menu items being checked/unchecked.
toggleCheckedMenu(hObject);
handles.hController.updateModel(hObject,eventdata,handles);

function toggleCheckedMenu(hObject)
if strcmp(get(hObject,'Checked'),'on')
    set(hObject,'Checked','off'); 
else
    set(hObject,'Checked','on');
end

function etDisplayDepth_Callback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);

% --- Executes on button press in cbGotoOnSelect.
function cbGotoOnSelect_Callback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);
handles.hController.roiGotoOnSelect_Helper(get(hObject,'Value'),handles);

% --- Executes on button press in cbSnapOnSelect.
function cbSnapOnSelect_Callback(hObject, eventdata, handles)
handles.hController.updateModel(hObject,eventdata,handles);

% --- Executes on button press in pbClearShown.
function pbClearShown_Callback(hObject, eventdata, handles)
handles.hModel.roiClearShown();

% --- Executes on button press in pbPosnGUI.
function pbPosnGUI_Callback(hObject, eventdata, handles)
tetherGUIs('roiGUI','positionGUI','righttop');
% if get(hObject,'Value')
% 	tetherGUIs('roiGUI','positionGUI','righttop');
% else
% 	hideGUI('gh.positionGUI.figure1');
% end

% --------------------------------------------------------------------
function mnuWarnOnMove_Callback(hObject, eventdata, handles)
toggleCheckedMenu(hObject);
handles.hController.updateModel(hObject,eventdata,handles);


% --- Executes on button press in tbAdvanced.
function tbAdvanced_Callback(hObject, eventdata, handles)
toggleAdvancedPanel(hObject,4,'y');

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


% --- Executes during object creation, after setting all properties.
function pbUpOneLevel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pbUpOneLevel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global state;
if isfield(state,'hSI') && ~state.hSI.mdlInitialized % prevents GUIDE from loading CData
    set(hObject,'CData',most.gui.loadIcon('arrow.bmp',16,270,[0 0 1]));
end


% --- Executes on button press in pbGoto.
function pbGoto_Callback(hObject, eventdata, handles)
% hObject    handle to pbGoto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.hModel.roiGotoROI();
