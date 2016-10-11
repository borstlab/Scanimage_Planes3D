function varargout = imageControls(varargin)
%IMAGECONTROLS M-file for imageControls.fig
%      IMAGECONTROLS, by itself, creates a new IMAGECONTROLS or raises the existing
%      singleton*.
%
%      H = IMAGECONTROLS returns the handle to a new IMAGECONTROLS or the handle to
%      the existing singleton*.
%
%      IMAGECONTROLS('Property','Value',...) creates a new IMAGECONTROLS using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to imageControls_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      IMAGECONTROLS('CALLBACK') and IMAGECONTROLS('CALLBACK',hObject,...) call the
%      local function named CALLBACK in IMAGECONTROLS.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help imageControls

% Last Modified by GUIDE v2.5 05-Dec-2011 11:43:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @imageControls_OpeningFcn, ...
                   'gui_OutputFcn',  @imageControls_OutputFcn, ...
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


% --- Executes just before imageControls is made visible.
function imageControls_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for imageControls
handles.output = hObject;

%Ensure all controls/panels respond to key presses, when they have the focus (for whatever reason)
fig = openfig(mfilename, 'reuse');
set(fig,'KeyPressFcn',@genericKeyPressFunction);
kidControls = findall(fig,'Type','uicontrol');
for i=1:length(kidControls)
    if ~strcmpi(get(kidControls(i),'Style'),'edit')
        set(kidControls(i),'KeyPressFcn',@genericKeyPressFunction);
    end
end

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes imageControls wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
hideGUI('gh.imageControls.figure1');


% --- Outputs from this function are returned to the command line.
function varargout = imageControls_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


%%%VI022009A: Removed %%%%%%%%%%%%%%%%%%%%%%%
% --------------------------------------------------------------------
% function varargout = zoom_Callback(h, eventdata, handles, varargin)
% % Stub for Callback of the uicontrol handles.intensity.
% global state gh
% setImagesToWhole; 
% tag=get(h,'tag');
% channel=str2num(tag(end));
% string=get(h,'String');
% if strcmpi(string,'Zoom')
%     handle=state.internal.GraphFigure(channel);
%     zoom(handle,'on');
%     set(h,'String','Out');
%     set(h,'UserData',handle);
% else
%     zoom(get(h,'UserData'),'off');
%     ax=findobj(get(h,'UserData'),'type','axes');
%     xlim=get(ax,'XLim');
%     ylim=get(ax,'YLim');
%     change=0;
%     if xlim(1)==0
%         xlim(1)=1;
%         change=1;
%     end
%     if ylim(1)==0
%         ylim(1)=1;
%         change=1;
%     end
%     set(h,'String','ZOOM');
%     set(h,'UserData',[]);
%     if change
%         set(ax,'YLim',ylim,'XLim',xlim);
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on button press in tbAdvanced.
function tbAdvanced_Callback(hObject, eventdata, handles)
% hObject    handle to tbAdvanced (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of tbAdvanced
global gh
offset = 12; % the vertical size by which to grow the figure
toggleAdvancedPanel(hObject,offset,'y');

% --- Executes on button press in pbSaveUSR.
function pbSaveUSR_Callback(hObject, eventdata, handles)
% hObject    handle to pbSaveUSR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveCurrentUserSettings();


% --- Executes on button press in pbGetPMTOffsets.
function pbGetPMTOffsets_Callback(hObject, eventdata, handles)
% hObject    handle to pbGetPMTOffsets (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
startPMTOffsets();

% --- Executes on button press in cbAverageSamples.
function cbAverageSamples_Callback(hObject, eventdata, handles)
% hObject    handle to cbAverageSamples (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbAverageSamples
genericCallback(hObject);

% --- Executes on button press in cbShowCrosshair.
function cbShowCrosshair_Callback(hObject, eventdata, handles)
% hObject    handle to cbShowCrosshair (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbShowCrosshair
genericCallback(hObject);


function etRollingAverage_Callback(hObject, eventdata, handles)
% hObject    handle to etRollingAverage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etRollingAverage as text
%        str2double(get(hObject,'String')) returns contents of etRollingAverage as a double
genericCallback(hObject);


% --- Executes on button press in cbLockRollAvg2AcqAvg.
function cbLockRollAvg2AcqAvg_Callback(hObject, eventdata, handles)
% hObject    handle to cbLockRollAvg2AcqAvg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbLockRollAvg2AcqAvg
genericCallback(hObject); 

%Switch to rolling average=1
global state
if ~state.acq.lockAvgFrames
    state.acq.numAvgFramesDisplay = 1;
    updateGUIByGlobal('state.acq.numAvgFramesDisplay','Callback',true);
end    

% --------------------------------------------------------------------
function varargout = pbStats_Callback(h, eventdata, handles, varargin)
global state
%%%VI022009A:Removed %%%%%%
% tag=get(h,'tag');
% channel=str2num(tag(end));
% handle=state.internal.GraphFigure(channel);
% ax=findobj(handle,'type','axes');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI022009A%%%%%%
[ax,hIm,channel] = si_selectImageFigure(); %VI051310A
if isempty(ax)
    return;
end
if isempty(channel)
    disp('Cannot compute statistics for selected image');
    return;
end
%%%%%%%%%%%%%%%%%%

xbounds = get(hIm,'XData'); %VI051310A
ybounds = get(hIm,'YData'); %VI051310A

ximagebounds = round(get(ax,'XLim')); %VI051310A
yimagebounds = round(get(ax,'YLim')); %VI051310A

xindices = intersect(xbounds(1):xbounds(2),ximagebounds(1):ximagebounds(2)); %VI051310A
yindices = intersect(ybounds(1):ybounds(2),yimagebounds(1):yimagebounds(2)); %VI051310A

data=state.acq.acquiredData{1}{channel}(yindices,xindices); %VI092210A %VI051310A
Image_Stats.mean=mean(data(:)); %VI022009B
Image_Stats.std=double(std(single(data(:)))); %VI022009B
Image_Stats.max=max(max(data));
Image_Stats.min=min(min(data));
Image_Stats.pixels=numel(data)
assignin('base','Image_Stats',Image_Stats);





% --------------------------------------------------------------------
function varargout = pbHistogram_Callback(h, eventdata, handles, varargin)
global gh state
%%%VI022009A:Removed %%%%%%
% tag=get(h,'tag');
% channel=str2num(tag(end));
% handle=state.internal.GraphFigure(channel);
% ax=findobj(handle,'type','axes');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI022009A%%%%%%
[ax,hIm,channel] = si_selectImageFigure(); %VI051310A
if isempty(ax)
    return;
end
if isempty(channel)
    disp('Cannot compute histogram for selected image');
    return;
end
%%%%%%%%%%%%%%%%%%

xbounds = get(hIm,'XData'); %VI051310A
ybounds = get(hIm,'YData'); %VI051310A

ximagebounds = round(get(ax,'XLim')); %VI051310A
yimagebounds = round(get(ax,'YLim')); %VI051310A

xindices = intersect(xbounds(1):xbounds(2),ximagebounds(1):ximagebounds(2)); %VI051310A
yindices = intersect(ybounds(1):ybounds(2),yimagebounds(1):yimagebounds(2)); %VI051310A

data=state.acq.acquiredData{1}{channel}(yindices,xindices); %VI092210A %VI051310A
f=figure('DoubleBuffer','on','color','w','NumberTitle','off','Name','Pixel Histogram',...
    'PaperPositionMode','auto','PaperOrientation','landscape'); 
hist(double(reshape(data,numel(data),1)),256);
set(get(gca,'XLabel'),'String','Pixel Intensity','FontWeight','bold','FontSize',12);
set(get(gca,'YLabel'),'String','Number of Pixels','FontWeight','bold','FontSize',12); 

state.internal.figHandles = [f state.internal.figHandles]; %VI110708A

% --------------------------------------------------------------------
function varargout = pmTargetFigure_Callback(h, eventdata, handles, varargin)
genericCallback(h);


% --------------------------------------------------------------------
function varargout = tbZoom_Callback(h, eventdata, handles, varargin)
guiToolToggle(h,@zoom);

% --------------------------------------------------------------------
function varargout = tbDataTip_Callback(h, eventdata, handles, varargin)
guiToolToggle(h,@datacursormode);


% --------------------------------------------------------------------
function varargout = pmImageColormap_Callback(h, eventdata, handles, varargin)
genericCallback(h);
%applyColormap(h,eventdata,handles);

function blackEditChan1_Callback(hObject, eventdata, handles)
% hObject    handle to blackEditChan1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of blackEditChan1 as text
%        str2double(get(hObject,'String')) returns contents of blackEditChan1 as a double
genericLUTEdit_Callback(hObject);

% --- Executes on slider movement.
function blackSlideChan1_Callback(hObject, eventdata, handles)
% hObject    handle to blackSlideChan1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
genericLUTSlider_Callback(hObject);

function blackEditChan2_Callback(hObject, eventdata, handles)
% hObject    handle to blackEditChan2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of blackEditChan2 as text
%        str2double(get(hObject,'String')) returns contents of blackEditChan2 as a double
genericLUTEdit_Callback(hObject);

% --- Executes on slider movement.
function blackSlideChan2_Callback(hObject, eventdata, handles)
% hObject    handle to blackSlideChan2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
genericLUTSlider_Callback(hObject);


function blackEditChan3_Callback(hObject, eventdata, handles)
% hObject    handle to blackEditChan3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of blackEditChan3 as text
%        str2double(get(hObject,'String')) returns contents of blackEditChan3 as a double
genericLUTEdit_Callback(hObject);

% --- Executes on slider movement.
function blackSlideChan3_Callback(hObject, eventdata, handles)
% hObject    handle to blackSlideChan3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
genericLUTSlider_Callback(hObject);

function blackEditChan4_Callback(hObject, eventdata, handles)
% hObject    handle to blackEditChan4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of blackEditChan4 as text
%        str2double(get(hObject,'String')) returns contents of blackEditChan4 as a double
genericLUTEdit_Callback(hObject);

% --- Executes on slider movement.
function blackSlideChan4_Callback(hObject, eventdata, handles)
% hObject    handle to blackSlideChan4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
genericLUTSlider_Callback(hObject);

function whiteEditChan1_Callback(hObject, eventdata, handles)
% hObject    handle to whiteEditChan1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of whiteEditChan1 as text
%        str2double(get(hObject,'String')) returns contents of whiteEditChan1 as a double
genericLUTEdit_Callback(hObject);

% --- Executes on slider movement.
function whiteSlideChan1_Callback(hObject, eventdata, handles)
% hObject    handle to whiteSlideChan1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
genericLUTSlider_Callback(hObject);

function whiteEditChan2_Callback(hObject, eventdata, handles)
% hObject    handle to whiteEditChan2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of whiteEditChan2 as text
%        str2double(get(hObject,'String')) returns contents of whiteEditChan2 as a double
genericLUTEdit_Callback(hObject);

% --- Executes on slider movement.
function whiteSlideChan2_Callback(hObject, eventdata, handles)
% hObject    handle to whiteSlideChan2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
genericLUTSlider_Callback(hObject);

function whiteEditChan3_Callback(hObject, eventdata, handles)
% hObject    handle to whiteEditChan3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of whiteEditChan3 as text
%        str2double(get(hObject,'String')) returns contents of whiteEditChan3 as a double
genericLUTEdit_Callback(hObject);

% --- Executes on slider movement.
function whiteSlideChan3_Callback(hObject, eventdata, handles)
% hObject    handle to whiteSlideChan3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
genericLUTSlider_Callback(hObject);

function whiteEditChan4_Callback(hObject, eventdata, handles)
% hObject    handle to whiteEditChan4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of whiteEditChan4 as text
%        str2double(get(hObject,'String')) returns contents of whiteEditChan4 as a double
genericLUTEdit_Callback(hObject);

% --- Executes on slider movement.
function whiteSlideChan4_Callback(hObject, eventdata, handles)
% hObject    handle to whiteSlideChan4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
genericLUTSlider_Callback(hObject);

%% HELPERS

function varargout = genericLUTEdit_Callback(h)
% Handler for LUT edit controls

global state

genericCallback(h);

if strfind(get(h,'Tag'),'black')
    maxVal = state.internal.maxLUTValue - 1;
else
    maxVal = state.internal.maxLUTValue;
end

if str2double(get(h,'String')) > maxVal
    set(h,'String',num2str(maxVal));
    updateGUIGlobal(h);
end

setImagesToWhole;
updateClim;

return;

% --------------------------------------------------------------------
function varargout = genericLUTSlider_Callback(h)
% Handler for LUT slider controls

global state gh

genericCallback(h);
setImagesToWhole;
updateClim;
tagButton=get(h,'tag');

% Toggle GUI tool on for selected image figure, or off for all image figures
function guiToolToggle(h,guiToolFunc)
global state

if get(h,'Value')
    setImagesToWhole();

    hAx = si_selectImageFigure();
    if isempty(hAx)
        return;
    end
    
    guiToolFunc(ancestor(hAx,'figure'),'on');    
else
    imageHandles = [state.internal.imagehandle state.internal.mergeimage];
    arrayfun(@(handle) guiToolFunc(ancestor(handle,'figure'),'off'),imageHandles);
end


%% SETTINGS MENU

% --------------------------------------------------------------------
function mnu_Settings_AverageSamples_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_Settings_AverageSamples (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
toggleCheckMark(hObject);
genericCallback(hObject);

% --------------------------------------------------------------------
function mnu_Settings_ShowCrosshair_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_Settings_ShowCrosshair (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
toggleCheckMark(hObject);
genericCallback(hObject);

% --------------------------------------------------------------------
function mnu_Settings_SaveUserSettings_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_Settings_SaveUserSettings (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveCurrentUserSettings();


% --------------------------------------------------------------------
function mnu_Settings_SaveUserSettingsAs_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_Settings_SaveUserSettingsAs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
saveCurrentUserSettingsAs();

% --------------------------------------------------------------------
function mnu_Settings_ShowUserSettings_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_Settings_ShowUserSettings (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
seeGUI('gh.userPreferenceGUI.figure1');

%% PMT OFFSETS MENU


% --------------------------------------------------------------------
function mnu_PMTOffsets_GetPMTOffsets_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_PMTOffsets_GetPMTOffsets (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
startPMTOffsets();

% --------------------------------------------------------------------
function mnu_PMTOffsets_AutoRead_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_PMTOffsets_AutoRead (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
toggleCheckMark(hObject);
genericCallback(hObject);

% --------------------------------------------------------------------
function mnu_PMTOffsets_AutoSubtractChan1_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_PMTOffsets_AutoSubtractChan1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
toggleCheckMark(hObject);
genericCallback(hObject);


% --------------------------------------------------------------------
function mnu_PMTOffsets_AutoSubtractChan2_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_PMTOffsets_AutoSubtractChan2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
toggleCheckMark(hObject);
genericCallback(hObject);

% --------------------------------------------------------------------
function mnu_PMTOffsets_AutoSubtractChan3_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_PMTOffsets_AutoSubtractChan3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
toggleCheckMark(hObject);
genericCallback(hObject);

% --------------------------------------------------------------------
function mnu_PMTOffsets_AutoSubtractChan4_Callback(hObject, eventdata, handles)
% hObject    handle to mnu_PMTOffsets_AutoSubtractChan4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
toggleCheckMark(hObject);
genericCallback(hObject);


%% CREATE FCNS

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

% --- Executes during object creation, after setting all properties.
function etRollingAverage_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etRollingAverage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etFrameSelections_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etFrameSelections (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function etFrameSelFactor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etFrameSelFactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function pmTargetFigure_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmTargetFigure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function blackSlideChan3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to blackSlideChan3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function blackEditChan3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to blackEditChan3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function whiteSlideChan3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to whiteSlideChan3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function whiteEditChan3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to whiteEditChan3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function blackSlideChan2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to blackSlideChan2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function whiteSlideChan2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to whiteSlideChan2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function blackEditChan2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to blackEditChan2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function whiteEditChan2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to whiteEditChan2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function blackSlideChan4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to blackSlideChan4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function blackEditChan4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to blackEditChan4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function whiteSlideChan4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to whiteSlideChan4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function whiteEditChan4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to whiteEditChan4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function blackSlideChan1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to blackSlideChan1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function blackEditChan1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to blackEditChan1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function whiteEditChan1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to whiteEditChan1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function whiteSlideChan1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to whiteSlideChan1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end


