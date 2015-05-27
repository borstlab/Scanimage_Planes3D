function varargout = alignGUI(varargin)
% ALIGNGUI M-file for alignGUI.fig
%      ALIGNGUI, by itself, creates a new ALIGNGUI or raises the existing
%      singleton*.
%
%      H = ALIGNGUI returns the handle to a new ALIGNGUI or the handle to
%      the existing singleton*.
%
%      ALIGNGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ALIGNGUI.M with the given input arguments.
%
%      ALIGNGUI('Property','Value',...) creates a new ALIGNGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before alignGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to alignGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help alignGUI

% Last Modified by GUIDE v2.5 17-Nov-2009 17:59:28

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @alignGUI_OpeningFcn, ...
    'gui_OutputFcn',  @alignGUI_OutputFcn, ...
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


% --- Executes just before alignGUI is made visible.
function alignGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to alignGUI (see VARARGIN)

% Choose default command line output for alignGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes alignGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = alignGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function tbParkAtOffset_Callback(hObject, eventdata, handles)

global state gh
figure(gh.mainControls.figure1);

if get(hObject,'Value')
    scim_pointLaser([state.init.scanOffsetAngleX state.init.scanOffsetAngleY],false); %VI071511A: Use new scim_pointLaser() fcn %VI101510A: Use new 'transmit' flag option
    set(hObject,'String','BEAM ON!','ForegroundColor',[.5 0 0]);
else
    scim_parkLaser();
    set(hObject,'String','PARK 0','ForegroundColor',[.043 .518 .78]);
end

% resp = questdlg('Click OK to park laser beam at scan center for beam alignment/measurement. Shutter will be opened.','Park Beam @ Center','OK', 'Cancel', 'OK'); %VI121908B
% if strcmpi(resp,'OK') %VI121908B
%     scim_parkLaser([state.init.scanOffsetAngleX state.init.scanOffsetAngleY]);
%     h = msgbox('Click when done aligning and/or measuring the laser beam','Park Beam @ Center','modal');
%     uiwait(h);
%     scim_parkLaser;
% end


% --------------------------------------------------------------------
function pbSetOffset_Callback(hObject, eventdata, handles)

global state gh

if state.acq.scanRotation
    h=msgbox('Cannot set X/Y ScanOffsetAngle with non-zero rotation. Collect image with rotation set to 0 before setting X/Y ScanOffsetAngle.');
    uiwait(h);
else
    h=gh.mainControls.focusButton;
    if strcmpi(get(h,'String'),'Abort') %presently focusing
        abortFocus;
    end

    %Flag that Scan Offset has been changed
    updateGUIByGlobal('state.internal.scanOffsetChanged','Value',1,'Callback',1);

    %Adjust ScanOffsetAngle and shift values
    if state.acq.fastScanningX %VI092010A
        state.init.scanOffsetAngleX = state.init.scanOffsetAngleX + state.acq.scanShiftFast; %VI092010A  
        state.init.scanOffsetAngleY = state.init.scanOffsetAngleY + state.acq.scanShiftSlow; %VI092010A
    else
        state.init.scanOffsetAngleX = state.init.scanOffsetAngleX + state.acq.scanShiftSlow; %VI092010A
        state.init.scanOffsetAngleY = state.init.scanOffsetAngleY + state.acq.scanShiftFast; %VI092010A
    end


    state.acq.scanShiftSlow = 0; %VI092010A
    state.acq.scanShiftFast = 0; %VI092010A

    %Update GUI values
    updateGUIByGlobal('state.init.scanOffsetAngleX');
    updateGUIByGlobal('state.init.scanOffsetAngleY');
    updateGUIByGlobal('state.acq.scanShiftSlow'); %VI092010A
    updateGUIByGlobal('state.acq.scanShiftFast'); %VI092010A

    %Refresh scan settings
    applyConfigurationSettings; %Though scanOffsetAngleX/Y are not part of configuration, handle this as if a new configruation was loaded

end

% --------------------------------------------------------------------
function pbSaveINI_Callback(hObject, eventdata, handles)
resp = questdlg(['INI files are commonly shared between multiple users on a rig.' sprintf('\n') 'Save new Offset values to INI file anyway?'],'WARNING', 'Yes','No','No');
switch resp
    case 'Yes'
        readOrWriteOffset(1);      
    case 'No'
        %all done!
end

% --------------------------------------------------------------------
function pbLoadINI_Callback(hObject, eventdata, handles)
readOrWriteOffset(0);

function readOrWriteOffset(writeFlag)
global state

err = false;

%Open the file
iniFileName = [state.iniPath filesep state.iniName '.ini'];
iniFID = fopen(iniFileName,'r+');

vars = {'scanOffsetAngleX' 'scanOffsetAngleY'};
newLines = cell(2,1);
foundLines = [];

while ~feof(iniFID)
    foundLine = false;
    finished = false;

    posn = ftell(iniFID);
    origLine = fgets(iniFID);
    
    for i=1:length(vars)        
        [startChar,endChar,matchStr] = regexp(origLine,[vars{i} '=(?:-?[0-9]+\.?[0-9]*)'],'start','end','match','once');
        
        if ~isempty(startChar)
            foundLines = [foundLines posn];
            posns(i) = posn+(startChar-1);

            if writeFlag
                newLines{i} = [vars{i} '=' num2str(state.init.(vars{i}),'%0.2f')];
                extraSpaces(i) = (endChar-startChar+1) - length(newLines{i});
            else
                offsetVals(i) = str2num(matchStr(length(vars{i})+2:end));
            end
        end
    end
    
    if length(foundLines) == length(vars) %finished!       
        break; %all done
    end
end

%Write or read the values
if length(foundLines) == length(vars) %finished correctly
    for i=1:length(vars)
        if writeFlag
            fseek(iniFID,posns(i),'bof');
            fprintf(iniFID,'%s',newLines{i});
            for i=1:length(extraSpaces(i))
                fprintf(iniFID,' ');
            end
        else
            state.init.(vars{i}) = offsetVals(i);
            updateGUIByGlobal(['state.init.' vars{i}]);                    
        end
    end
    
    %Update scan parameters
    if ~writeFlag
        applyConfigurationSettings();
    end    
else    
    err = true;
end

%Close the file
fclose(iniFID);

if err
   error('Failed to find all the expected variables');
else
    %Flag that Scan Offset is no longer 'changed' (i.e., relative to INI file value)
    updateGUIByGlobal('state.internal.scanOffsetChanged','Value',0,'Callback',1);    
end


% --------------------------------------------------------------------
% This is an invisible toggle button which reflects the scanOffsetChanged state
function tbScanOffsetChanged_Callback(hObject, eventdata, handles)
global state gh

if state.internal.scanOffsetChanged
    set([gh.alignGUI.pbLoadINI gh.alignGUI.pbSaveINI],'Enable','on');
else
    set([gh.alignGUI.pbLoadINI gh.alignGUI.pbSaveINI],'Enable','off');
end


% --------------------------------------------------------------------
function figure1_CloseRequestFcn(hObject, eventdata, handles)
global state
state.internal.showAlignGUI = 0;
updateGUIByGlobal('state.internal.showAlignGUI','Callback',1);


