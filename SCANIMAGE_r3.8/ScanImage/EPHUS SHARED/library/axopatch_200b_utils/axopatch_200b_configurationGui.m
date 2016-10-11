function varargout = axopatch_200b_configurationGui(varargin)
% AXOPATCH_200B_CONFIGURATIONGUI M-file for axopatch_200b_configurationGui.fig
%      AXOPATCH_200B_CONFIGURATIONGUI, by itself, creates a new AXOPATCH_200B_CONFIGURATIONGUI or raises the existing
%      singleton*.
%
%      H = AXOPATCH_200B_CONFIGURATIONGUI returns the handle to a new AXOPATCH_200B_CONFIGURATIONGUI or the handle to
%      the existing singleton*.
%
%      AXOPATCH_200B_CONFIGURATIONGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in AXOPATCH_200B_CONFIGURATIONGUI.M with the given input arguments.
%
%      AXOPATCH_200B_CONFIGURATIONGUI('Property','Value',...) creates a new AXOPATCH_200B_CONFIGURATIONGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before axopatch_200b_configurationGui_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to axopatch_200b_configurationGui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help axopatch_200b_configurationGui

% Last Modified by GUIDE v2.5 05-May-2005 18:34:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @axopatch_200b_configurationGui_OpeningFcn, ...
                   'gui_OutputFcn',  @axopatch_200b_configurationGui_OutputFcn, ...
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
return;

% ------------------------------------------------------------------
% --- Executes just before axopatch_200b_configurationGui is made visible.
function axopatch_200b_configurationGui_OpeningFcn(hObject, eventdata, handles, varargin)

% Choose default command line output for axopatch_200b_configurationGui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
return;

% ------------------------------------------------------------------
% --- Outputs from this function are returned to the command line.
function varargout = axopatch_200b_configurationGui_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;
return;

% ------------------------------------------------------------------
function out = makeGlobalCellArray(hObject, eventdata, handles)

out = {
       'hObject', hObject, ...
       'amplifier', [], ...
       'scaledOutputBoardID', 1, 'Class', 'Numeric', 'Gui', 'scaledOutputBoardID', ...
       'scaledOutputChannelID', 0, 'Class', 'Numeric', 'Gui', 'scaledOutputChannelID', ...
       'vComBoardID', 1, 'Class', 'Numeric', 'Gui', 'vComBoardID', ...
       'vComChannelID', 0, 'Class', 'Numeric', 'Gui', 'vComChannelID', ...
       'gainBoard', 1, 'Class', 'Numeric', 'Gui', 'gainBoard', ...
       'gainChannel', 0, 'Class', 'Numeric', 'Gui', 'gainChannel', ...
       'modeBoard', 1, 'Class', 'Numeric', 'Gui', 'modeBoard', ...
       'modeChannel', 0, 'Class', 'Numeric', 'Gui', 'modeChannel', ...
       'vHoldBoard', 1, 'Class', 'Numeric', 'Gui', 'vHoldBoard', ...
       'vHoldChannel', 0, 'Class', 'Numeric', 'Gui', 'vHoldChannel', ...
       'vClampInputFactor', 1, 'Class', 'Numeric', 'Gui', 'vClampInputFactor', ...
       'iClampInputFactor', 1, 'Class', 'Numeric', 'Gui', 'iClampInputFactor', ...
       'vClampOutputFactor', 1, 'Class', 'Numeric', 'Gui', 'vClampOutputFactor', ...
       'iClampOutputFactor', 1, 'Class', 'Numeric', 'Gui', 'iClampOutputFactor', ...
       'beta', 1, 'Class', 'Numeric', 'Gui', 'beta', ...
       'changesMade', 0, ...
   };

return;

% ------------------------------------------------------------------
function genericStartFcn(hObject, eventdata, handles)

%TO120905B: The @progmanager will change the close function for this GUI. Fix that here. -- Tim O'Connor 12/9/05
set(getParent(hObject, 'figure'), 'CloseFcn', 'closeProgram(progmanager, gcf)');

return;

% ------------------------------------------------------------------
function genericUpdateFcn(hObject, eventdata, handles)

return;

% ------------------------------------------------------------------
function genericCloseFcn(hObject, eventdata, handles)

%Apply the changes.
if getLocal(progmanager, hObject, 'changesMade')
    save = questdlg('Apply changes?', 'Apply changes?', 'Yes', 'No', 'Yes');
    if strcmpi(save, 'Yes')
        commitChanges(hObject);
    end
end

%In case @progmanager's deleteObjectsOnClose is in effect. The modified amplifier should persist.
setLocal(progmanager, hObject, 'amplifier', []);

return;

% ------------------------------------------------------------------
function out = getVersion(hObject, eventdata, handles)

out = 0.1;

return;

% ------------------------------------------------------------------
function gainBoard_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function gainBoard_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function gainChannel_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function gainChannel_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function modeBoard_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function modeBoard_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function modeChannel_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function modeChannel_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function vHoldBoard_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function vHoldBoard_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function vHoldChannel_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function vHoldChannel_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function vClampInputFactor_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function vClampInputFactor_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function iClampInputFactor_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function iClampInputFactor_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function vClampOutputFactor_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function vClampOutputFactor_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function iClampOutputFactor_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function iClampOutputFactor_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function vComBoardID_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function vComBoardID_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function vComChannelID_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function vComChannelID_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function scaledOutputBoardID_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function scaledOutputBoardID_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function scaledInputChannelID_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function scaledInputChannelID_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function scaledOutputChannelID_Callback(hObject, eventdata, handles)

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function scaledOutputChannelID_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
return;

% ------------------------------------------------------------------
function commitChanges(hObject)

if ~getLocal(progmanager, hObject, 'changesMade')
    return;
end

amp = getLocal(progmanager, hObject, 'amplifier');
if isempty(amp)
    warning('No amplifier attached to this configuration GUI.');
    return;
end

set(amp, 'gain_daq_board_id', getLocal(progmanager, hObject, 'gainBoard'));
set(amp, 'mode_daq_board_id', getLocal(progmanager, hObject, 'modeBoard'));
set(amp, 'v_hold_daq_board_id', getLocal(progmanager, hObject, 'vHoldBoard'));
set(amp, 'gain_channel', getLocal(progmanager, hObject, 'gainChannel'));
set(amp, 'mode_channel', getLocal(progmanager, hObject, 'modeChannel'));
set(amp, 'v_hold_channel', getLocal(progmanager, hObject, 'vHoldChannel'));
set(amp, 'i_clamp_input_factor', getLocal(progmanager, hObject, 'iClampInputFactor'));
set(amp, 'v_clamp_input_factor', getLocal(progmanager, hObject, 'vClampInputFactor'));
set(amp, 'i_clamp_output_factor', getLocal(progmanager, hObject, 'iClampOutputFactor'));
set(amp, 'v_clamp_output_factor', getLocal(progmanager, hObject, 'vClampOutputFactor'));
set(amp, 'scaledOutputBoardID', getLocal(progmanager, hObject, 'scaledOutputBoardID'));
set(amp, 'scaledOutputChannelID', getLocal(progmanager, hObject, 'scaledOutputChannelID'));
set(amp, 'vComBoardID', getLocal(progmanager, hObject, 'vComBoardID'));
set(amp, 'vComChannelID', getLocal(progmanager, hObject, 'vComChannelID'));
set(amp, 'beta', getLocal(progmanager, hObject, 'beta'));

name = get(amp, 'name');

gainName = [name '-gain'];
configureChannel(hObject, gainName, getLocal(progmanager, hObject, 'gainBoard'), ...
    getLocal(progmanager, hObject, 'gainChannel'), 1);
setScaledOutputChannelNames(amp, gainName);
enableChannel(dm, gainName);

modeName = [name '-mode'];
configureChannel(hObject, modeName, getLocal(progmanager, hObject, 'modeBoard'), ...
    getLocal(progmanager, hObject, 'modeChannel'), 1);
setScaledOutputChannelNames(amp, modeName);
enableChannel(dm, modeName);

vhName = [name '-v_hold'];
configureChannel(hObject, vhName, getLocal(progmanager, hObject, 'vHoldBoard'), ...
    getLocal(progmanager, hObject, 'vHoldChannel'), 1);
setScaledOutputChannelNames(amp, vhName);
enableChannel(dm, vhName);

soName = [name '_scaledOutput'];
configureChannel(hObject, soName, getLocal(progmanager, hObject, 'scaledOutputBoardID'), ...
    getLocal(progmanager, hObject, 'scaledOutputChannelID'), 1);
setScaledOutputChannelNames(amp, soName);
enableChannel(dm, soName);

vcName = [name '_vCom'];
configureChannel(hObject, vcName, getLocal(progmanager, hObject, 'vComBoardID'), ...
    getLocal(progmanager, hObject, 'vComChannelID'), 1);
setVComChannelName(amp, vcName);
enableChannel(dm, vcName);

setLocal(progmanager, hObject, 'changesMade', 0);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'Off');

return;

% ------------------------------------------------------------------
function configureChannel(hObject, channelName, boardId, channelId, ioFlag)

if hasChannel(dm, channelName)
    denameInputChannel(dm, channelName);
end
if hasChannel(dm, boardId, channelId, ioFlag)
    warndlg(sprintf('Channel %s on board %s is already in use as channel %s.', num2str(channelId), num2str(boardID), ...
        getChannelName(boardId, channelId), 'Conflict', 'modal'));
end
if ioFlag == 0
    nameOutputChannel(boardId, channelId, channelName);
else
    nameInputChannel(boardId, channelId, channelName);
end

return;

% ------------------------------------------------------------------
% --- Executes on button press in applyChanges.
function applyChanges_Callback(hObject, eventdata, handles)

commitChanges(hObject);

return;

% ------------------------------------------------------------------
% --- Executes on button press in refresh.
function refresh_Callback(hObject, eventdata, handles)

axopatch_200b_refreshGUI(hObject);

return;

% --- Executes during object creation, after setting all properties.
function beta_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% ------------------------------------------------------------------
function beta_Callback(hObject, eventdata, handles)

beta = getLocal(progmanager, hObject, 'beta');
if ~(beta == 1 | beta == 0.1)
    beta = 1;
    warndlg(sprintf('Illegal beta value entered: %s', num2str(beta)));
    setLocal(progmanager, hObject, 'beta');
end

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;

% ------------------------------------------------------------------
% --- Executes during object creation, after setting all properties.
function alpha_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% ------------------------------------------------------------------
function alpha_Callback(hObject, eventdata, handles)

alpha = getLocal(progmanager, hObject, 'alpha');
if ~ismember(alpha, [0.5 5 50 500])
    alpha = 1;
    warndlg(sprintf('Illegal alpha value entered: %s', num2str(alpha)));
    setLocal(progmanager, hObject, 'alpha');
end

setLocal(progmanager, hObject, 'changesMade', 1);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'On');

return;