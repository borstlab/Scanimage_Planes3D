function varargout = climGUI(varargin)
% CLIMGUI   - Generic GUI Image Handle Browser.
%    CLIMGUI is the mfile called from the program manager
%    to start the GUI called CLIMGUI.  This GUI can be
%    used as part of any program in the program 
%    manager.
%    
%    Multiple instances of this GUI are also allowed.
%    
%    See also PROGMANAGER, ADDPROGRAM, STARTPROGRAM
   
% Last Modified by GUIDE v2.5 16-Feb-2004 11:16:43

% Begin initialization code - DO NOT EDIT
gui_Singleton = 0;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @climGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @climGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin & isstr(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function climGUI_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
guidata(hObject, handles);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function varargout = climGUI_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function minpixel_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function minpixel_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function maxpixel_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function maxpixel_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function minpixelSlider_CreateFcn(hObject, eventdata, handles)
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function minpixelSlider_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function maxpixelSlider_CreateFcn(hObject, eventdata, handles)
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function maxpixelSlider_Callback(hObject, eventdata, handles)
setlocal(progmanager,hObject,'currentimagedata',zeros(128,128,1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function getImage_Callback(hObject, eventdata, handles)
updateImages(hObject);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function image_lbox_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function image_lbox_Callback(hObject, eventdata, handles)
prog_manager_obj=progmanager;
currenthandle=getlocal(prog_manager_obj,hObject,'allimagehandles',getlocal(prog_manager_obj,hObject,'image_lbox',1));
if ~ishandle(currenthandle)
    updateImages(hObject);
    return
end
cdata=get(currenthandle,'CData');
clims=get(get(currenthandle,'Parent'),'CLim');
set(getlocal(prog_manager_obj,hObject,'currentimagehandle'),'CData',cdata,'Parent',getlocalgh(prog_manager_obj,hObject,'copied_axes'));
axis(getlocalgh(prog_manager_obj,hObject,'copied_axes'),'tight');
setlocal(prog_manager_obj,hObject,'minpixel',clims(1));
setlocal(prog_manager_obj,hObject,'maxpixel',clims(2));
drawnow expose;%TO042309A - Not using expose can cause C-spawned events to fire out of order.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function updateImages(hObject,eventdata,handles)

prog_manager_obj=progmanager;   % referecne to program manager
shh=get(0,'ShowHiddenHandles');
set(0,'ShowHiddenHandles','on');
fig_h = findobj(0,'type','figure');
imgagehandles=findobj(fig_h,'type','image');
set(0,'ShowHiddenHandles',shh);
if ~isempty(imgagehandles)
    thisimage=find(imgagehandles==getlocal(prog_manager_obj,hObject,'currentimagehandle'));
    imgagehandles(find(imgagehandles==getlocal(prog_manager_obj,hObject,'currentimagehandle')))=[];
    setlocal(prog_manager_obj,hObject,'allimagehandles',imgagehandles);
    names=num2str(imgagehandles);
    setlocalgh(prog_manager_obj,hObject,'image_lbox','String',names);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function genericStartFcn(fig_handle,eventdata,handles)
% general function that sets up variables for this GUI. executed by the
% program manager after variabes are assigned.
prog_manager_obj=progmanager;   % referecne to program manager
setlocal(prog_manager_obj,fig_handle,'currentimagedata',zeros(128,128,1));
setlocal(prog_manager_obj,fig_handle,'currentimagehandle',...
    image(getlocal(prog_manager_obj,fig_handle,'currentimagedata'),'Parent',getlocalgh(prog_manager_obj,fig_handle,'copied_axes')));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out=makeGlobalCellArray(fig_handle,eventdata,handles)
% teststructure.name=rand(3,3,3);
% teststructure.cells={'aaa',22222};
% teststructure.sub.sub1=[1 2 3];
out={
    'minpixel',0,'Class','double','Gui','minpixel','Gui','minpixelSlider','Min',-15000,'Max',15000,...
    'maxpixel',0,'Class','double','Gui','maxpixel','Gui','maxpixelSlider','Min',-15000,'Max',15000,...
    'image_lbox',1,'Class','int','Gui','image_lbox','Min',1};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function genericUpdateFcn(fig_handle,eventdata,handles)
			
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out=getVersion(fig_handle,eventdata,handles)
out=1;