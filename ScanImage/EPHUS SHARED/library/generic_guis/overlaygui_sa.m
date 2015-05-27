function varargout = overlaygui_sa(varargin)
% OVERLAYGUI_SA   - Generic GUI for Overlaying Images in RGB image.
% 	OVERLAYGUI_SA is the mfile called from the program manager
% 	to start the GUI called OVERLAYGUI_SA.  This GUI can be
% 	used as part of any program in the program 
% 	manager.
% 	
% 	Multiple instances of this GUI are also allowed.
% 	
% 	OVERLAYGUI_SA will allow the user to load TIF files
%   that are intensity images and place them
%   into different layers of an RGB image, giving the 
%   effect of overlaying.
%   
% 	The images can also be filtered, shifted, and rotated.
%   
% 	See also PROGMANAGER, ADDPROGRAM, STARTPROGRAM

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @overlaygui_sa_OpeningFcn, ...
                   'gui_OutputFcn',  @overlaygui_sa_OutputFcn, ...
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

% --- Executes just before overlaygui_sa is made visible.
function overlaygui_sa_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for overlaygui_sa
handles.output = hObject;
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = overlaygui_sa_OutputFcn(hObject, eventdata, handles)
% Get default command line output from handles structure
varargout{1} = handles.output;

% End initialization code - DO NOT EDIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START CALLBACKS FOR GUI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function filterImage_Callback(hObject, eventdata, handles)
switchString(hObject,'Filter','Unfilter');
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function redEditBox_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function redEditBox_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function greenEditBox_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function greenEditBox_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function blueEditBox_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function blueEditBox_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function redOpen_Callback(hObject, eventdata, handles)
prog_manager_obj=progmanager;   % referecne to program manager
[canceled,data,filename,header,path]=openimage(gcbf,getlocal(prog_manager_obj,hObject,'redPath'));
setlocal(prog_manager_obj,hObject,'redPath',filename);
if ~canceled
	updateColorProps(gcbf,'red',data,filename,header,path);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function greenOpen_Callback(hObject, eventdata, handles)
prog_manager_obj=progmanager;   % referecne to program manager
[canceled,data,filename,header,path]=openimage(gcbf,getlocal(progmanager,hObject,'greenPath'));
setlocal(prog_manager_obj,hObject,'greenPath',filename);
if ~canceled
	updateColorProps(gcbf,'green',data,filename,header,path);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function blueOpen_Callback(hObject, eventdata, handles)
prog_manager_obj=progmanager;   % referecne to program manager
[canceled,data,filename,header,path]=openimage(gcbf,getlocal(progmanager,hObject,'bluePath'));
setlocal(prog_manager_obj,hObject,'bluePath',filename);
if ~canceled
	updateColorProps(gcbf,'blue',data,filename,header,path);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function redStrength_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function redStrength_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function greenStrength_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function greenStrength_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function blueStrength_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function blueStrength_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function redStrengthSlider_CreateFcn(hObject, eventdata, handles)
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function redStrengthSlider_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function greenStrengthSlider_CreateFcn(hObject, eventdata, handles)
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function greenStrengthSlider_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function blueStrengthSlider_CreateFcn(hObject, eventdata, handles)
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function blueStrengthSlider_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function overallStrength_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function overallStrength_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function overallStrengthSlider_CreateFcn(hObject, eventdata, handles)
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function overallStrengthSlider_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function copyImage_Callback(hObject, eventdata, handles)
copyImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exportImage_Callback(hObject, eventdata, handles)
exportImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function redOn_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function greenOn_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function blueOn_Callback(hObject, eventdata, handles)
updateImage(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function colorSelector_Callback(hObject, eventdata, handles)
setlocal(progmanager,hObject,'colorSelectorString',getMenuEntry(hObject));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function colorSelector_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function shiftLeft_Callback(hObject, eventdata, handles)
moveImage(gcbf,hObject,getMenuEntry(getlocalgh(progmanager,hObject,'colorSelector')));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function shiftRight_Callback(hObject, eventdata, handles)
moveImage(gcbf,hObject,getMenuEntry(getlocalgh(progmanager,hObject,'colorSelector')));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function shiftDown_Callback(hObject, eventdata, handles)
moveImage(gcbf,hObject,getMenuEntry(getlocalgh(progmanager,hObject,'colorSelector')));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function shiftUp_Callback(hObject, eventdata, handles)
moveImage(gcbf,hObject,getMenuEntry(getlocalgh(progmanager,hObject,'colorSelector')));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function rotateCCW_Callback(hObject, eventdata, handles)
moveImage(gcbf,hObject,getMenuEntry(getlocalgh(progmanager,hObject,'colorSelector')));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function rotateCW_Callback(hObject, eventdata, handles)
moveImage(gcbf,hObject,getMenuEntry(getlocalgh(progmanager,hObject,'colorSelector')));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function fineMovement_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function zoom_Callback(hObject, eventdata, handles)
current=switchString(hObject,'Zoom','Off');
if strcmpi(current,'Off')
	zoom(gcbf,'On');
else
	zoom(gcbf,'Off');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function fineMovementAmount_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function fineMovementAmount_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function fineMovementAmountSlider_CreateFcn(hObject, eventdata, handles)
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function fineMovementAmountSlider_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% END OF CALBACKS FOR GUI.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BELOW ARE FUNCTIONS CALLED DURING THE CALLBACKS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [canceled,data,filename,header,pathname]=openimage(fig_handle,startPath)
prog_manager_obj=progmanager;   % referecne to program manager

startPath=getlocal(progmanager,fig_handle,'lastPathSelected');
if isempty(startPath)
	startPath=getlocal(progmanager,fig_handle,'lastPathSelected');
end
[filename, pathname] = uigetfile([startPath '*.tif'] , 'Choose image to load...');
canceled=0;
data=[];
header=[];
if isnumeric(filename)
	filename='';
	pathname=startPath;
	canceled=1;
	return
else
	fullfilename=fullfile(pathname,filename); 
	info=imfinfo(fullfilename);
	frames = length(info);
	framenumber=1;
	if isfield(info(1),'ImageDescription')
		header=info(1).ImageDescription;
		header=parseHeader(header);
	end
	if frames>1
		beep;
		prompt={'Enter Frame To Load:'};,def={'1'};,dlgTitle='*** This Image Has more than 1 Frame ***';,lineNo=1;
		answer=inputdlg(prompt,dlgTitle,lineNo,def);
		if isempty(answer)
			filename='';
			pathname=startPath;
			canceled=1;
			return
		else
			framenumber=str2num(answer{1});
		end
	end
	data=double(imread(fullfilename,framenumber));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function updateColorProps(fig_handle,color,data,filename,header,path);
prog_manager_obj=progmanager;   % referecne to program manager
setlocal(prog_manager_obj,fig_handle,[color 'EditBox'],filename);
setlocal(prog_manager_obj,fig_handle,[color 'Data'],data);
setlocal(prog_manager_obj,fig_handle,[color 'Header'],header);
setlocal(prog_manager_obj,fig_handle,[color 'Path'],path);
setlocal(prog_manager_obj,fig_handle,[color 'Min'],min(reshape(data,1,numel(data))));
setlocal(prog_manager_obj,fig_handle,[color 'Max'],max(reshape(data,1,numel(data))));
setlocal(prog_manager_obj,fig_handle,'lastPathSelected',path);
updateImage(fig_handle);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function updateImage(fig_handle,varargin)
% Get name of GUI and Program from the figure
prog_manager_obj=progmanager;   % referecne to program manager
channelsOn=logical([getlocal(prog_manager_obj,fig_handle,'redOn') getlocal(prog_manager_obj,fig_handle,'greenOn') getlocal(prog_manager_obj,fig_handle,'blueOn')]);
if ~any(channelsOn)
	setlocal(prog_manager_obj,fig_handle,'currentimagedata',0);
	set(getlocal(prog_manager_obj,fig_handle,'currentimagehandle'),'CData',getlocal(prog_manager_obj,fig_handle,'currentimagedata'));
	return
end
channelNames={'red','green','blue'};
for colors=1:length(channelNames)
	if channelsOn(colors)
		sizeArray(colors,:)=size(getlocal(prog_manager_obj,fig_handle,[channelNames{colors} 'Data']));
		if any(sizeArray(colors,:) == [0 0])
			channelsOn(colors)=0;
		end
	else
		sizeArray(colors,:)=[0 0];
	end
end
imagesizes=sizeArray(channelsOn,:);
if length(unique(imagesizes))>1
	beep;
	warning('OverlayGUI/UpdateImage: Image Sizes are not equal');
	return
elseif isempty(imagesizes)
	return
end
setlocal(prog_manager_obj,fig_handle,'currentimagedata',zeros([imagesizes(1,:) 3]));
currentmaxPixel=0;
for colors=1:length(channelNames)
	if channelsOn(colors)
        newmaxPixel=max(currentmaxPixel,max(max(max(getlocal(prog_manager_obj,fig_handle,[channelNames{colors} 'Data'])))));
		setlocal(prog_manager_obj,fig_handle,'currentMaxPixel',newmaxPixel);
        tempdata=getlocal(prog_manager_obj,fig_handle,[channelNames{colors} 'Data']);
        tempdata=(tempdata.*getlocal(prog_manager_obj,fig_handle,[channelNames{colors} 'Strength']))./getlocal(prog_manager_obj,fig_handle,['currentMaxPixel']);
        if getlocal(prog_manager_obj,fig_handle,'filterImage')
          tempdata=medfilt2(tempdata,[3 3]);
		end
		setlocal(prog_manager_obj,fig_handle,'currentimagedata',tempdata,':',':',colors);
	end
end
tempdata=getlocal(prog_manager_obj,fig_handle,'currentimagedata');
tempdata=tempdata*getlocal(prog_manager_obj,fig_handle,'overallStrength');

tempdata(tempdata>1)=1;
tempdata(tempdata<0)=0;
setlocal(prog_manager_obj,fig_handle,'currentimagedata',tempdata);

set(getlocal(prog_manager_obj,fig_handle,'currentimagehandle'),'CData',getlocal(prog_manager_obj,fig_handle,'currentimagedata'));

if ~getlocal(prog_manager_obj,fig_handle,'zoom')
	set(getlocalgh(prog_manager_obj,fig_handle,'overlayaxes'),'XLim',[1 imagesizes(1,1)],'YLim',[1 imagesizes(1,2)]);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function copyImage(fig_handle,varargin)
f=exportImage(fig_handle);
print(f, '-dbitmap');
close(f);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function f=exportImage(fig_handle,varargin)
prog_manager_obj=progmanager;   % referecne to program manager
screensize=get(0,'ScreenSize');
x=get(getlocal(prog_manager_obj,fig_handle,'currentimagehandle'),'XData');
y=get(getlocal(prog_manager_obj,fig_handle,'currentimagehandle'),'YData');
f = figure('numberTitle', 'off','Color','White','DoubleBuffer','On','Position',[.25*screensize(3:4) range(x) range(y)]);
newax=axes('NextPlot','Add','Parent',f,'YDir','Reverse');
newhandle=copyobj(getlocal(prog_manager_obj,fig_handle,'currentimagehandle'),newax);
axis(newax,'image');
set(newax,'Position',[0 0 1 1],'Units','Normalized','Visible','off');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out=moveImage(fig_handle,hObject,color)
prog_manager_obj=progmanager;   % referecne to program manager
todo=get(hObject,'Tag');
movefactorv=0;
movefactorh=0;
rotate=0;
movefactor=20;
if getlocal(prog_manager_obj,fig_handle,'fineMovement')
	movefactor=getlocal(prog_manager_obj,fig_handle,'fineMovementAmount');
end
color=translate(color,{'Red','red','Green','green','Blue','blue'});
switch todo
	case 'shiftLeft'
		movefactorh=-movefactor;
	case 'shiftRight'
		movefactorh=movefactor;
	case 'shiftUp'
		movefactorv=-movefactor;
	case 'shiftDown'
		movefactorv=movefactor;
	case 'rotateCCW'
		rotate=movefactor;
	case 'rotateCW'
		rotate=-movefactor;
	otherwise
end
if any([movefactorh movefactorv] ~=0)
	setlocal(prog_manager_obj,fig_handle,[color 'Data'],circshift(getlocal(prog_manager_obj,fig_handle,[color 'Data']),[movefactorv movefactorh]));
elseif rotate~=0
	setlocal(prog_manager_obj,fig_handle,[color 'Data'],imrotate(getlocal(prog_manager_obj,fig_handle,[color 'Data']),rotate,'crop'));
end
updateImage(fig_handle);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exportImageUIContextCallback(hObject,varargin)
% UIContext Menu callbacks using fhandles have a special format.  See Help
% for details.
exportImage(get(get(hObject,'Parent'),'Parent'),varargin{:})

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function copyImageUIContextCallback(hObject,varargin)
% UIContext Menu callbacks using fhandles have a special format.  See Help
% for details.
copyImage(get(get(hObject,'Parent'),'Parent'),varargin{:})

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function genericStartFcn(fig_handle,eventdata,handles)
% general function that sets up variables for this GUI. executed by the
% program manager after variabes are assigned.
prog_manager_obj=progmanager;   % referecne to program manager
setlocal(prog_manager_obj,fig_handle,'currentimagedata',zeros(256,256,3));
cmenu = uicontextmenu('Parent',fig_handle);
uimenu(cmenu, 'Label', 'Export to New Figure', 'Callback', {@exportImageUIContextCallback});
uimenu(cmenu, 'Label', 'Copy To Clipboard', 'Callback', {@copyImageUIContextCallback});
parent_axes=getlocalgh(prog_manager_obj,fig_handle,'overlayaxes');
im_handle=image(getlocal(prog_manager_obj,fig_handle,'currentimagedata'),'Parent',parent_axes,'UIContextMenu',cmenu);
setlocal(prog_manager_obj,fig_handle,'currentimagehandle',im_handle);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out=makeGlobalCellArray(fig_handle,eventdata,handles)
out={
    'lastPathSelected','',...
    'redPath','','Config',1,...
    'greenPath','','Config',1,...
    'bluePath','','Config',1,...
    'redEditBox','','Gui','redEditBox',...
    'greenEditBox','','Gui','greenEditBox',...
    'blueEditBox','','Gui','blueEditBox',...
    'redStrength',1,'Config',1,'Gui','redStrength','Gui','redStrengthSlider','Class','double','Min',0,'Max',100,...
    'greenStrength',1,'Config',1,'Gui','greenStrength','Gui','greenStrengthSlider','Class','double','Min',0,'Max',100,...
    'blueStrength',1,'Config',1,'Gui','blueStrength','Gui','blueStrengthSlider','Class','double','Min',0,'Max',100,...
    'overallStrength',1,'Config',1,'Gui','overallStrength','Gui','overallStrengthSlider','Class','double','Min',0,'Max',100,...
    'redOn',1,'Gui','redOn','Class','bool',...
    'greenOn',1,'Gui','greenOn','Class','bool',...
    'blueOn',0,'Gui','blueOn','Class','bool',...
    'redData','',...
    'redMin','',...
    'redmax','',...
    'greenData','',...
    'greenMin','',...
    'greenmax','',...
    'blueData','',...
    'blueMin','',...
    'bluemax','',...
    'filterImage',0,'Gui','filterImage','Class','bool',...
    'currentMaxPixel',1,...
    'colorSelector',1,'Gui','colorSelector','Class','int',...
    'colorSelectorString','red',...
    'fineMovement',0,'Gui','fineMovement','Class','bool',...
    'zoom',0,'Gui','zoom','Class','bool',...
    'fineMovementAmount',5,'Gui','fineMovementAmount','Gui','fineMovementAmountSlider','Min',0,'Max',50,'Class','int',...
};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function genericUpdateFcn(fig_handle,eventdata,handles)
updateImage(fig_handle);
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out=getVersion(fig_handle,eventdata,handles)
out=1;




