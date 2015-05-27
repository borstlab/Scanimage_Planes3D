function varargout = uncagingMapper(varargin)
% UNCAGINGMAPPER M-file for uncagingMapper.fig
%      UNCAGINGMAPPER, by itself, creates a new UNCAGINGMAPPER or raises the existing
%      singleton*.
%
%      H = UNCAGINGMAPPER returns the handle to a new UNCAGINGMAPPER or the handle to
%      the existing singleton*.
%
%      UNCAGINGMAPPER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNCAGINGMAPPER.M with the given input arguments.
%
%      UNCAGINGMAPPER('Property','Value',...) creates a new UNCAGINGMAPPER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before uncagingMapper_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to uncagingMapper_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help uncagingMapper

% Last Modified by GUIDE v2.5 15-May-2007 13:55:19

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @uncagingMapper_OpeningFcn, ...
                   'gui_OutputFcn',  @uncagingMapper_OutputFcn, ...
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


% --- Executes just before uncagingMapper is made visible.
function uncagingMapper_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to uncagingMapper (see VARARGIN)

% Choose default command line output for uncagingMapper
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes uncagingMapper wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = uncagingMapper_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function xText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%-------------------------------------------------------------------
function xText_Callback(hObject, eventdata, handles)
global state;

genericCallback(hObject);

if state.init.eom.uncagingMapper.x < 0
    state.init.eom.uncagingMapper.x = 0;
elseif state.init.eom.uncagingMapper.x > 1
    state.init.eom.uncagingMapper.x = 1;
end

state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, 1) = ...
    state.init.eom.uncagingMapper.x;

%Make sure it gets truncated, as necessary.
updateGUIByGlobal('state.init.eom.uncagingMapper.duration', 'Callback', 1);

updatePixelDisplay;

return;

% --- Executes during object creation, after setting all properties.
function yText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%-------------------------------------------------------------------
function yText_Callback(hObject, eventdata, handles)
global state;

genericCallback(hObject);

if state.init.eom.uncagingMapper.y < 0
    state.init.eom.uncagingMapper.y = 0;
elseif state.init.eom.uncagingMapper.y > 1
    state.init.eom.uncagingMapper.y = 1;
end

state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, 2) = ...
    state.init.eom.uncagingMapper.y;

updatePixelDisplay;

return;

% --- Executes during object creation, after setting all properties.
function durationText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to durationText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%-------------------------------------------------------------------
function durationText_Callback(hObject, eventdata, handles)
global state;

genericCallback(hObject);

if state.init.eom.uncagingMapper.duration < 0
    updateGUIByGlobal('state.init.eom.uncagingMapper.duration', 'Value', 0, 'Callback', 0);
elseif state.init.eom.uncagingMapper.duration + state.init.eom.uncagingMapper.x * state.acq.fillFraction * state.acq.msPerLine > ... %VI012109A
        state.acq.fillFraction * state.acq.msPerLine
    updateGUIByGlobal('state.init.eom.uncagingMapper.duration', 'Value', state.acq.fillFraction * state.acq.msPerLine ... %VI012109A
        - state.init.eom.uncagingMapper.x * state.acq.fillFraction * state.acq.msPerLine);
end

state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, 3) = ...
    state.init.eom.uncagingMapper.duration;

updatePixelDisplay;

return;

% --- Executes during object creation, after setting all properties.
function autoPowerText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to autoPowerText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%-------------------------------------------------------------------
function autoPowerText_Callback(hObject, eventdata, handles)
global state gh;

genericCallback(hObject);

conversion = (getfield(state.init.eom, ['powerConversion' num2str(state.init.eom.uncagingMapper.beam)]) * ...
    state.init.eom.maxPhotodiodeVoltage(state.init.eom.uncagingMapper.beam) * .01);
    
if state.init.eom.powerInMw
    state.init.eom.uncagingMapper.autoPower = 1 / conversion * state.init.eom.uncagingMapper.autoPower;
end

if state.init.eom.uncagingMapper.autoPower > 100
    state.init.eom.uncagingMapper.autoPower = 100;
    
    if state.init.eom.powerInMw
        state.init.eom.uncagingMapper.autoPower = state.init.eom.uncagingMapper.autoPower * conversion;
    end
    
    updateGUIByGlobal('state.init.eom.uncagingMapper.autoPower');
elseif state.init.eom.uncagingMapper.autoPower < state.init.eom.min(state.init.eom.uncagingMapper.beam)
    state.init.eom.uncagingMapper.autoPower = state.init.eom.min(state.init.eom.uncagingMapper.beam);
    
    if state.init.eom.powerInMw
        state.init.eom.uncagingMapper.autoPower = state.init.eom.uncagingMapper.autoPower * conversion;
    end
    
    updateGUIByGlobal('state.init.eom.uncagingMapper.autoPower');
end

%Needs updating.
set(gh.uncagingMapper.autoButton, 'ForeGround', [1 0 0]);

return;

% --- Executes during object creation, after setting all properties.
function pixelSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pixelSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes on slider movement.
function pixelSlider_Callback(hObject, eventdata, handles)
global state;

genericCallback(hObject);

if state.init.eom.uncagingMapper.pixelSliderPosition > state.init.eom.uncagingMapper.pixelSliderLast | ...
        state.init.eom.uncagingMapper.pixelSliderPosition == 1

    if state.init.eom.uncagingMapper.pixel < size(state.init.eom.uncagingMapper.pixels, 2)

        pixel = findLastValidPixel;

        %Increment.
        if ~isempty(pixel) & state.init.eom.uncagingMapper.pixel < pixel
            updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', state.init.eom.uncagingMapper.pixel + 1, 'Callback', 0);
        end
    end
    
elseif state.init.eom.uncagingMapper.pixelSliderPosition < state.init.eom.uncagingMapper.pixelSliderLast | ...
        state.init.eom.uncagingMapper.pixelSliderPosition == 0

    if state.init.eom.uncagingMapper.pixel > 1
        %Decrement.
        updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Callback', 0, 'Value', state.init.eom.uncagingMapper.pixel - 1);
        
    end
end

state.init.eom.uncagingMapper.pixelSliderLast = state.init.eom.uncagingMapper.pixelSliderPosition;

updatePixelDisplay;

return;

% --- Executes during object creation, after setting all properties.
function pixelText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pixelText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%-------------------------------------------------------------------
function pixelText_Callback(hObject, eventdata, handles)

genericCallback(hObject);

updatePixelDisplay;

return;

% --- Executes during object creation, after setting all properties.
function pixelsText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pixelsText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%-------------------------------------------------------------------
function pixelsText_Callback(hObject, eventdata, handles)
global state gh;

genericCallback(hObject);

%Can not be less than 4.
if state.init.eom.uncagingMapper.numberOfPixels < 2
    state.init.eom.uncagingMapper.numberOfPixels = 2;
    updateGUIByGlobal('state.init.eom.uncagingMapper.numberOfPixels');
end

%Make sure it's divisible by 4.
r = rem(state.init.eom.uncagingMapper.numberOfPixels, 4);
if r ~= 0 & state.init.eom.uncagingMapper.numberOfPixels ~= 2
    %Always bump up to the next higest value.
    updateGUIByGlobal('state.init.eom.uncagingMapper.numberOfPixels', 'Value', state.init.eom.uncagingMapper.numberOfPixels - r + 4);
end

%Keep the slider clued in to what's going on.
if state.init.eom.uncagingMapper.numberOfPixels < state.init.eom.uncagingMapper.lastNum

    %We've just decremented.
    updateGUIByGlobal('state.init.eom.uncagingMapper.sliderPosition', 'Value', 0);
    state.init.eom.uncagingMapper.sliderLast = 0;

elseif state.init.eom.uncagingMapper.numberOfPixels > state.init.eom.uncagingMapper.lastNum

    %We've just incremented.
    updateGUIByGlobal('state.init.eom.uncagingMapper.sliderPosition', 'Value', 1);
    state.init.eom.uncagingMapper.sliderLast = 1;

end

set(gh.uncagingMapper.autoButton, 'ForeGround', [1 0 0]);

updateGUIByGlobal('state.init.eom.uncagingMapper.autoDuration', 'Value', 10 * round(100 * state.acq.msPerLine / state.init.eom.uncagingMapper.numberOfPixels) / 1000); %VI012109A

%Hang on to this for the next time around, so we can tell if it's a decrement or increment.
state.init.eom.uncagingMapper.lastNum = state.init.eom.uncagingMapper.numberOfPixels;
        
return;

% --- Executes during object creation, after setting all properties.
function pixelsSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pixelsSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes on slider movement.
function pixelsSlider_Callback(hObject, eventdata, handles)
global state gh;

genericCallback(hObject);

if state.init.eom.uncagingMapper.sliderPosition > state.init.eom.uncagingMapper.sliderLast | ...
        state.init.eom.uncagingMapper.sliderPosition == 1

    updateGUIByGlobal('state.init.eom.uncagingMapper.numberOfPixels', 'Callback', 0, 'Value', state.init.eom.uncagingMapper.numberOfPixels * 2);
    
elseif state.init.eom.uncagingMapper.sliderPosition < state.init.eom.uncagingMapper.sliderLast | ...
        state.init.eom.uncagingMapper.sliderPosition == 0
    
    if state.init.eom.uncagingMapper.numberOfPixels > 2
    
        updateGUIByGlobal('state.init.eom.uncagingMapper.numberOfPixels', 'Callback', 0, 'Value', state.init.eom.uncagingMapper.numberOfPixels / 2);
        
    end
end

%Make sure it's divisible by 4.
r = rem(state.init.eom.uncagingMapper.numberOfPixels, 4);
if r ~= 0 & state.init.eom.uncagingMapper.numberOfPixels ~= 2
    %Always bump up to the next higest value.
    updateGUIByGlobal('state.init.eom.uncagingMapper.numberOfPixels', 'Value', state.init.eom.uncagingMapper.numberOfPixels - r + 4);
end

state.init.eom.uncagingMapper.sliderLast = state.init.eom.uncagingMapper.sliderPosition;

updateGUIByGlobal('state.init.eom.uncagingMapper.autoDuration', 'Value', 10 * round(100 * state.acq.msPerLine / state.init.eom.uncagingMapper.numberOfPixels) / 1000); %VI012109A

%Needs updating.
set(gh.uncagingMapper.autoButton, 'ForeGround', [1 0 0]);

% fprintf(1, 'Max: %s\nMin: %s\nVal: %s\nPixels: %s\n\n', num2str(get(hObject, 'Max')), num2str(get(hObject, 'Min')), num2str(get(hObject, 'Value')), ...
%     num2str(state.init.eom.uncagingMapper.numberOfPixels));
return;

% --- Executes during object creation, after setting all properties.
function autoDurationText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to autoDurationText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

return;

%-------------------------------------------------------------------
function autoDurationText_Callback(hObject, eventdata, handles)
global state gh;

genericCallback(hObject);

sampleTime = 1 / ...
    getAOProperty(state.acq.dm, state.init.eom.pockelsCellNames{state.init.eom.uncagingMapper.beam}, 'SampleRate');
if state.init.eom.uncagingMapper.autoDuration < 1000 * sampleTime
    updateGUIByGlobal('state.init.eom.uncagingMapper.autoDuration', 'Value',  1000 * sampleTime);
% elseif state.init.eom.uncagingMapper.autoDuration > 1
%     updateGUIByGlobal('state.init.eom.uncagingMapper.autoDuration', 'Value', 1);
end

if state.init.eom.uncagingMapper.autoDuration > (state.acq.msPerLine / state.init.eom.uncagingMapper.numberOfPixels) %VI012109A
    updateGUIByGlobal('state.init.eom.uncagingMapper.autoDuration', 'Value', 10 * round(100 * state.acq.msPerLine / state.init.eom.uncagingMapper.numberOfPixels) / 1000); %VI012109A
end

%Needs updating.
set(gh.uncagingMapper.autoButton, 'ForeGround', [1 0 0]);

return;

% --- Executes during object creation, after setting all properties.
function orientationMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to orientationMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on selection change in orientationMenu.
function orientationMenu_Callback(hObject, eventdata, handles)
global gh;

genericCallback(hObject);

%Needs updating.
set(gh.uncagingMapper.autoButton, 'ForeGround', [1 0 0]);

return;

% --- Executes during object creation, after setting all properties.
function powerText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to powerText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

%-------------------------------------------------------------------
function powerText_Callback(hObject, eventdata, handles)
global state;

genericCallback(hObject);

if state.init.eom.powerInMw
    conversion = (getfield(state.init.eom, ['powerConversion' num2str(state.init.eom.uncagingMapper.beam)]) * ...
        state.init.eom.maxPhotodiodeVoltage(state.init.eom.uncagingMapper.beam) * .01);
    
    state.init.eom.uncagingMapper.power = state.init.eom.uncagingMapper.power * conversion;
else
    conversion = 1;
end

if state.init.eom.uncagingMapper.power * conversion < state.init.eom.min(state.init.eom.uncagingMapper.beam)
    state.init.eom.uncagingMapper.power = state.init.eom.min(state.init.eom.uncagingMapper.beam) / conversion;
elseif state.init.eom.uncagingMapper.power * conversion > 100
    state.init.eom.uncagingMapper.power = 100 / conversion;
end

state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, 4) = ...
    conversion * state.init.eom.uncagingMapper.power;

updateGUIByGlobal('state.init.eom.uncagingMapper.power');

return;

% --- Executes on button press in autoButton.
function autoButton_Callback(hObject, eventdata, handles)
global state gh;

xyCoords = zeros(state.init.eom.uncagingMapper.numberOfPixels ^ 2, 2);

if isempty(state.init.eom.uncagingMapper.pixels)
    state.init.eom.uncagingMapper.pixels = -1 * ones(state.init.eom.numberOfBeams, ...
        state.init.eom.uncagingMapper.numberOfPixels ^ 2 , 4);
end

k = 1;
n = state.init.eom.uncagingMapper.numberOfPixels / 2;
for i = 1 : n
    for j = 1 : n
        %This order is important to give some time in the beginning
        %to collect a baseline, and time at the end, to allow for the final
        %decay to occur.
        xyCoords(k, 1) = i + n;
        xyCoords(k, 2) = j + n;
        
        xyCoords(k + 1, 1) = i;
        xyCoords(k + 1, 2) = j;
        
        xyCoords(k + 2, 1) = i;
        xyCoords(k + 2, 2) = j + n;
        
        xyCoords(k + 3, 1) = i + n;
        xyCoords(k + 3, 2) = j;
        
        k = k + 4;
    end
end
xyCoords = xyCoords - 1;

% figure;plot(xyCoords(:, 1), xyCoords(:, 2), '.')
switch state.init.eom.uncagingMapper.orientation
    case 1
        %Default is top-left.
        
    case 2
        %Shift right.
        xyCoords(:, 1) = xyCoords(:, 1) + 1 - state.init.eom.uncagingMapper.autoDuration;
        
    case 3
        %Center.
        %Shift right by half a pixel.
        xyCoords(:, 1) = xyCoords(:, 1) + .5 - (state.init.eom.uncagingMapper.autoDuration / 2);
        %Shift down by half a pixel.
        xyCoords(:, 2) = xyCoords(:, 2) + .5;
        
    case 4
        %Shift down.
        xyCoords(:, 2) = xyCoords(:, 2) + 1;
        
    case 5
        %Shift down and right.
        xyCoords(:, 1) = xyCoords(:, 1) + 1 - state.init.eom.uncagingMapper.autoDuration;
        xyCoords(:, 2) = xyCoords(:, 2) + 1;
        
    otherwise
        error('UncagingMapper: Unknown orientation for auto-generating uncaging map.');
end

xyCoords = xyCoords ./ state.init.eom.uncagingMapper.numberOfPixels;
%Alex, comment this line out, to not display a plot.
% figure;plot(xyCoords(:, 1), -1 * xyCoords(:, 2), xyCoords(:, 1) + state.init.eom.uncagingMapper.duration, -1 * xyCoords(:, 2), '.'), xlim([0 1]), ylim([-1 0])
% hold on;
% plot(xyCoords(:, 1) + state.init.eom.uncagingMapper.duration, -1 * xyCoords(:, 2), '.')

if state.init.eom.uncagingMapper.shutterBlank
    newCoords = zeros(state.init.eom.uncagingMapper.numberOfPixels ^ 2 + 1, 2);
    newCoords(1, 1) = xyCoords(1, 1);
    newCoords(1, 2) = xyCoords(1, 2);

    newCoords(2 : end, :) = xyCoords(:, :);
    xyCoords = newCoords;
end

if size(state.init.eom.uncagingMapper.pixels, 2) > size(xyCoords, 1)
    state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, size(xyCoords, 1) : end, :) = -1;
    xyCoords(end + 1 : size(state.init.eom.uncagingMapper.pixels, 2), :) = -1;
elseif size(xyCoords, 1) > size(state.init.eom.uncagingMapper.pixels, 2)
    state.init.eom.uncagingMapper.pixels(:, size(state.init.eom.uncagingMapper.pixels, 2) : size(xyCoords, 1), :) = -1;
end

state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, ...
    :, [1 2]) = xyCoords;
state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, ...
    :, 3) = state.init.eom.uncagingMapper.autoDuration;

%Don't forget to store power in %.
if state.init.eom.powerInMw
    conversion = (getfield(state.init.eom, ['powerConversion' num2str(state.init.eom.uncagingMapper.beam)]) * ...
        state.init.eom.maxPhotodiodeVoltage(state.init.eom.uncagingMapper.beam) * .1);
else
    conversion = 1;
end

state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, :, 4) = ...
    conversion * state.init.eom.uncagingMapper.autoPower;
if state.init.eom.uncagingMapper.shutterBlank
    state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, 1, 4) = ...
        state.init.eom.min(state.init.eom.uncagingMapper.beam);
end

% set(gh.uncagingMapper.powerText, 'String', ...
%     num2str(round(state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, 1, 4) * conversion)));

enablePixelEditor(1);
state.init.eom.changed(state.init.eom.uncagingMapper.beam);

if ~isempty(state.init.eom.uncagingMapper.pixelGenerationUserFunction)
    fprintf(1, 'UncagingMapper: calling pixelGenerationUserFunction ''%s''.\n', state.init.eom.uncagingMapper.pixelGenerationUserFunction);
    try
        eval(sprintf('state.init.eom.uncagingMapper.pixels = %s(state.init.eom.uncagingMapper.pixels);', ...
            state.init.eom.uncagingMapper.pixelGenerationUserFunction));
    catch
    fprintf(1, 'UncagingMapper: error evaluating pixelGenerationUserFunction ''%s'' - %s.\n', state.init.eom.uncagingMapper.pixelGenerationUserFunction, lasterr);
    end
end

state.init.eom.uncagingMapper.position = 1;
updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', 1, 'Callback', 1);
updateHeaderString('state.init.eom.uncagingMapper.pixels');

%No longer needs updating.
set(gh.uncagingMapper.autoButton, 'ForeGround', [0 0 0]);

return;

% --- Executes on button press in perGrabRadioButton.
function perGrabRadioButton_Callback(hObject, eventdata, handles)
global gh;

genericCallback(hObject);
set(hObject, 'Enable', 'Inactive');
set(gh.uncagingMapper.perFrameRadioButton, 'Enable', 'On');
updateGUIByGlobal('state.init.eom.uncagingMapper.perFrame', 'Value', 0);
set(gh.uncagingMapper.perFrameRadioButton, 'Value', 0);
set(gh.uncagingMapper.loop, 'Enable', 'On');

%TO051507A
set(gh.uncagingMapper.singleFrame, 'Enable', 'On');
updateGUIByGlobal('state.init.eom.uncagingMapper.singleFrame', 'Value', 0);
set(gh.uncagingMapper.singleFrame, 'Value', 0);

return;

% --- Executes on button press in perFrameRadioButton.
function perFrameRadioButton_Callback(hObject, eventdata, handles)
global gh;

genericCallback(hObject);
set(hObject, 'Enable', 'Inactive');
set(gh.uncagingMapper.perGrabRadioButton, 'Enable', 'On');
updateGUIByGlobal('state.init.eom.uncagingMapper.perGrab', 'Value', 0);
set(gh.uncagingMapper.perGrabRadioButton, 'Value', 0);
set(gh.uncagingMapper.loop, 'Enable', 'Off');

%TO051507A
set(gh.uncagingMapper.singleFrame, 'Enable', 'On');
updateGUIByGlobal('state.init.eom.uncagingMapper.singleFrame', 'Value', 0);
set(gh.uncagingMapper.singleFrame, 'Value', 0);

return;

% --- Executes on button press in syncToPhysiologyCheckbox.
function syncToPhysiologyCheckbox_Callback(hObject, eventdata, handles)
genericCallback(hObject);

return;

% --- Executes during object creation, after setting all properties.
function beamMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to beamMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on selection change in beamMenu.
function beamMenu_Callback(hObject, eventdata, handles)

genericCallback(hObject);

updatePixelDisplay;

return;

% --- Executes on button press in enableButton.
function enableButton_Callback(hObject, eventdata, handles)
global state gh;

genericCallback(hObject);
state.init.eom.uncagingMapper.enabled(state.init.eom.uncagingMapper.beam) = ...
    state.init.eom.uncagingMapper.enable;

if state.init.eom.uncagingMapper.enable
    set(gh.uncagingMapper.enableButton, 'String', 'Disable');
    set(gh.uncagingMapper.enableButton, 'ForeGround', [1 0 0]);
else
    set(gh.uncagingMapper.enableButton, 'String', 'Enable');
    set(gh.uncagingMapper.enableButton, 'ForeGround', [0 .6 0]);
end

return;

% --- Executes during object creation, after setting all properties.
function beamSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to beamSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function beamSlider_Callback(hObject, eventdata, handles)
global state;

genericCallback(hObject);

if state.init.eom.uncagingMapper.beamSliderPosition > state.init.eom.uncagingMapper.beamSliderLast | ...
        state.init.eom.uncagingMapper.beamSliderPosition == 1
    
    if state.init.eom.uncagingMapper.beam > 1
        %Increment here, since the popup menu is reverse ordered.
        updateGUIByGlobal('state.init.eom.uncagingMapper.beam', 'Callback', 0, 'Value', state.init.eom.uncagingMapper.beam - 1);
    end
    
elseif state.init.eom.uncagingMapper.beamSliderPosition < state.init.eom.uncagingMapper.beamSliderLast | ...
        state.init.eom.uncagingMapper.beamSliderPosition == 0

    if state.init.eom.uncagingMapper.beam < state.init.eom.numberOfBeams
        %Decrement here, since the popup menu is reverse ordered.
        updateGUIByGlobal('state.init.eom.uncagingMapper.beam', 'Callback', 0, 'Value', state.init.eom.uncagingMapper.beam + 1);
    end
end

state.init.eom.uncagingMapper.beamSliderLast = state.init.eom.uncagingMapper.beamSliderPosition;

% updateGUIByGlobal('state.init.eom.uncagingMapper.pixelText', 'Value', 1, 'Callback', 1);
updatePixelDisplay;

return;

% --- Executes on button press in addPixel.
function addPixel_Callback(hObject, eventdata, handles)
global state;

lastValid = findLastValidPixel;
if isempty(state.init.eom.uncagingMapper.pixels)
    state.init.eom.uncagingMapper.pixels = -1 * ones(state.init.eom.numberOfBeams, ...
        1, 4);
elseif state.init.eom.uncagingMapper.pixel == lastValid
    state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, lastValid + 1, :) = -1;
end

%X
state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, lastValid, 1) = 0;
%Y
state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, lastValid, 2) = 0;
%Duration
state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, lastValid, 3) = state.init.eom.uncagingMapper.autoDuration;
%Power
state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, lastValid, 4) = state.init.eom.uncagingMapper.autoPower;

%Update the display.
updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Callback', 1, 'Value', lastValid);

return;

%--------------------------------------------
function pixel = findLastValidPixel
global state;

%TO051507B
if isempty(state.init.eom.uncagingMapper.pixels) || prod(size(state.init.eom.uncagingMapper.pixels)) == 1
    pixel = 1;
    state.init.eom.uncagingMapper.pixels = -1 * ones(state.init.eom.numberOfBeams, 1, 4);
    return;
end

%Find the earliest pixel with a -1 value.
[beam, pixel, field] = ind2sub(size(state.init.eom.uncagingMapper.pixels), find(state.init.eom.uncagingMapper.pixels(:, :, :) == -1));
pixel = min(pixel(find(beam == state.init.eom.uncagingMapper.beam)));

if ~any(find(state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, :, :) == -1))
    pixel = size(state.init.eom.uncagingMapper.pixels, 2);
end

if isempty(pixel)
    pixel = size(state.init.eom.uncagingMapper.pixels, 2) + 1;
end

return;

% --- Executes on button press in deletePixel.
function deletePixel_Callback(hObject, eventdata, handles)
global state;

if isempty(state.init.eom.uncagingMapper.pixels)
    return;
end

state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, :) = -1;
state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel : end - 1, :) = ...
    state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel + 1 : end, :);

%Find the earliest pixel with a -1 value.
pixel = findLastValidPixel;
if isempty(pixel)
    pixel = size(state.init.eom.uncagingMapper.pixels, 2) + 1;
end

if state.init.eom.uncagingMapper.pixel < pixel
    state.init.eom.uncagingMapper.pixel = state.init.eom.uncagingMapper.pixel + 1;
elseif state.init.eom.uncagingMapper.pixel > 1
    state.init.eom.uncagingMapper.pixel = state.init.eom.uncagingMapper.pixel - 1;
else
    state.init.eom.uncagingMapper.pixel = 1;
end

updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Callback', 1);

return;

% --- Executes on button press in drawButton.
function drawButton_Callback(hObject, eventdata, handles)
global state;

%Create a new pixel?
addPixel_Callback(hObject, eventdata, handles);

[x y] = getline;

updateGUIByGlobal('state.init.eom.uncagingMapper.x', 'Value', x(1) / state.acq.pixelsPerLine, 'Callback', 1);
updateGUIByGlobal('state.init.eom.uncagingMapper.y', 'Value', y(1) / state.acq.linesPerFrame, 'Callback', 1);
updateGUIByGlobal('state.init.eom.uncagingMapper.duration', 'Value', ...
    (x(2) - x(1)) / state.acq.pixelsPerLine * (state.acq.msPerLine * state.acq.fillFraction), 'Callback', 1); %VI012109A
% updateGUIByGlobal('state.init.eom.uncagingMapper.power', 'Value', state.init.eom.uncagingMapper.autoPower, 'Callback', 1);

return;

%-------------------------------------------------------------------
function updatePixelDisplay
global state gh;

if isempty(state.init.eom.uncagingMapper.pixels)
    updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', 1);
    updateGUIByGlobal('state.init.eom.uncagingMapper.x', 'Value', 0);
    updateGUIByGlobal('state.init.eom.uncagingMapper.y', 'Value', 0);
    updateGUIByGlobal('state.init.eom.uncagingMapper.duration', 'Value', 0.5);
    updateGUIByGlobal('state.init.eom.uncagingMapper.power', 'Value', 0);
    
    enablePixelEditor(0);

    return;
end

if size(state.init.eom.uncagingMapper.pixels, 1) < state.init.eom.uncagingMapper.beam  ...
    updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', 1);
    updateGUIByGlobal('state.init.eom.uncagingMapper.x', 'Value', 0);
    updateGUIByGlobal('state.init.eom.uncagingMapper.y', 'Value', 0);
    updateGUIByGlobal('state.init.eom.uncagingMapper.duration', 'Value', 0.5);
    updateGUIByGlobal('state.init.eom.uncagingMapper.power', 'Value', 0);
    
    enablePixelEditor(0);
    return;
end

lastValid = findLastValidPixel;
if state.init.eom.uncagingMapper.pixel > lastValid
    updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', lastValid, 'Callback', 1);
    return;
end

if state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel) < 0
    if state.init.eom.uncagingMapper.pixel > 1
        updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', state.init.eom.uncagingMapper.pixel - 1, 'Callback', 1);
    else
        updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', 1);
        updateGUIByGlobal('state.init.eom.uncagingMapper.x', 'Value', 0);
        updateGUIByGlobal('state.init.eom.uncagingMapper.y', 'Value', 0);
        updateGUIByGlobal('state.init.eom.uncagingMapper.duration', 'Value', 0.5);
        updateGUIByGlobal('state.init.eom.uncagingMapper.power', 'Value', 0);
        
        enablePixelEditor(0);
    end

    return;
end

if state.init.eom.uncagingMapper.pixel < 1
    fprintf(2, 'ERROR (UncagingMapper): Attempting to display invalid pixel - %s\n', ...
        num2str(state.init.eom.uncagingMapper.pixel));

    return;
elseif state.init.eom.uncagingMapper.pixel > size(state.init.eom.uncagingMapper.pixels, 2)
    state.init.eom.uncagingMapper.pixel = size(state.init.eom.uncagingMapper.pixels, 2);
elseif state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, :) == -1
    %Find the highest valid pixel.
    state.init.eom.uncagingMapper.pixel = min(...
        [ find(state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, :, 1) ~= -1), ...
        find(state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, :, 2) ~= -1), ...
        find(state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, :, 3) ~= -1), ...
        find(state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, :, 4) ~= -1) ]);
    
    if isempty(state.init.eom.uncagingMapper.pixel)
        state.init.eom.uncagingMapper.pixel = 1;
    end
end

updateGUIByGlobal('state.init.eom.uncagingMapper.pixel');

updateGUIByGlobal('state.init.eom.uncagingMapper.x', 'Value', ...
    state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, 1));

updateGUIByGlobal('state.init.eom.uncagingMapper.y', 'Value', ...
    state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, 2));

updateGUIByGlobal('state.init.eom.uncagingMapper.duration', 'Value', ...
    state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, 3));

state.init.eom.uncagingMapper.power = state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, 4);
if state.init.eom.powerInMw
    conversion = (getfield(state.init.eom, ['powerConversion' num2str(state.init.eom.uncagingMapper.beam)]) * ...
        state.init.eom.maxPhotodiodeVoltage(state.init.eom.uncagingMapper.beam) * .01);
else
    conversion = 1;
end

updateHeaderString('state.init.eom.uncagingMapper.pixels');
updateHeaderString('state.init.eom.uncagingMapper.pixel');
updateHeaderString('state.init.eom.uncagingMapper.power');
updateHeaderString('state.init.eom.uncagingMapper.x');
updateHeaderString('state.init.eom.uncagingMapper.y');
updateHeaderString('state.init.eom.uncagingMapper.duration');
updateHeaderString('state.init.eom.uncagingMapper.position');

set(gh.uncagingMapper.powerText, 'String', num2str(round(conversion * ...
    state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, state.init.eom.uncagingMapper.pixel, 4))));

enablePixelEditor(1);

if state.init.eom.uncagingMapper.showPixels
    if ~isempty(state.init.eom.uncagingMapper.pixelLineHandles)
        set(state.init.eom.uncagingMapper.pixelLineHandles, 'Visible', 'Off');
        delete(state.init.eom.uncagingMapper.pixelLineHandles);
    end
    
    for i = 1 : state.init.maximumNumberOfInputChannels
        state.init.eom.uncagingMapper.pixelLineHandles(i) = ...
            line([state.init.eom.uncagingMapper.x (state.init.eom.uncagingMapper.x + state.init.eom.uncagingMapper.duration / ...
               (state.acq.msPerLine * state.acq.fillFraction))] .* state.acq.pixelsPerLine, ... %VI012109A
            [state.init.eom.uncagingMapper.y state.init.eom.uncagingMapper.y] .* state.acq.linesPerFrame);
            set(state.init.eom.uncagingMapper.pixelLineHandles(i), 'Color', [0 .6 .2], 'LineWidth', 3, ...
                'Parent', state.internal.axis(i), 'Tag', 'uncagingMapperPixelDisplayButtonDownFcn', ...
                'ButtonDownFcn', 'uncagingMapperPixelDisplayButtonDownFcn');
    end
elseif ~isempty(state.init.eom.uncagingMapper.pixelLineHandles)
    set(state.init.eom.uncagingMapper.pixelLineHandles, 'Visible', 'Off');
    delete(state.init.eom.uncagingMapper.pixelLineHandles);
    state.init.eom.uncagingMapper.pixelLineHandles = [];
end

return;

%-------------------------------------------------------------------
function enablePixelEditor(yesOrNo)
global state gh;

val = 'Off';
if yesOrNo
    val = 'On';
end

% set(gh.uncagingMapper.enableButton, 'Enable', val);
% set(gh.uncagingMapper.xText, 'Enable', val);
% set(gh.uncagingMapper.yText, 'Enable', val);
% set(gh.uncagingMapper.durationText, 'Enable', val);
% set(gh.uncagingMapper.powerText, 'Enable', val);
% set(gh.uncagingMapper.pixelSlider, 'Enable', val);
set(gh.uncagingMapper.deletePixel, 'Enable', val);
set(gh.uncagingMapper.deleteAllPixels, 'Enable', val);
set(gh.uncagingMapper.enableButton, 'Enable', val);
if ~state.init.eom.uncagingMapper.perFrame
    set(gh.uncagingMapper.loop, 'Enable', val);
end

if ~isempty(state.init.eom.uncagingMapper.pixelLineHandles)
    set(state.init.eom.uncagingMapper.pixelLineHandles, 'Visible', 'Off');
    delete(state.init.eom.uncagingMapper.pixelLineHandles);
    state.init.eom.uncagingMapper.pixelLineHandles = [];
end

return;


% --- Executes on button press in resetPosition.
function resetPosition_Callback(hObject, eventdata, handles)
% hObject    handle to resetPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state;

state.init.eom.uncagingMapper.position = 1;
updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', 1, 'Callback', 1);


% --- Executes on button press in deleteAllPixels.
function deleteAllPixels_Callback(hObject, eventdata, handles)
% hObject    handle to deleteAllPixels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state gh;

state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, :, :) = -1;

updatePixelDisplay;
enablePixelEditor(0);

return;


% --- Executes on button press in plotPixels.
function plotPixels_Callback(hObject, eventdata, handles)
global state;

x = state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, :, 1);
y = -1 * state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, :, 2) + 1;
w = state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, :, 3) / sqrt(length(x));

f = figure('Color', 'White');

for i = 1 : length(x)
    line([x(i) (x(i) + w(i))], [y(i) y(i)], 'Color', 'Black');
    text(x(i), y(i), num2str(i), 'FontWeight', 'Normal');
end

line([0 0], [0 1], 'LineStyle', '--', 'Color', 'Black');
line([1 1], [0 1], 'LineStyle', '--', 'Color', 'Black');
line([0 1], [0 0], 'LineStyle', '--', 'Color', 'Black');
line([0 1], [1 1], 'LineStyle', '--', 'Color', 'Black');

title('Uncaging Map Excitation Points', 'FontWeight', 'Bold', 'FontSize', 12);
xlim([-0.05 1.05]);
ylim([-0.05 1.05]);
xlabel('X Coordinate [normalized]', 'FontSize', 10);
ylabel('Y Coordinate [normalized]', 'FontSize', 10);

% text(1, 1, 'Image Boundary');
state.internal.figHandles = [f state.internal.figHandles]; %VI110708A

return;

% --- Executes on button press in loop.
function loop_Callback(hObject, eventdata, handles)
global state gh;

if strcmpi(get(gh.uncagingMapper.loop, 'String'), 'loop')
    %Force auto-generation.
    if isempty(state.init.eom.uncagingMapper.pixels)
        autoButton_Callback(gh.uncagingMapper.autoButton);
    end
    
    if state.init.eom.uncagingMapper.position > size(state.init.eom.uncagingMapper.pixels, 2)
        state.init.eom.uncagingMapper.position = 1;
        updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', state.init.eom.uncagingMapper.position, ...
            'Callback', 1);
    end

    state.init.eom.uncagingMapper.tempEnable = state.init.eom.uncagingMapper.enable;
    updateGUIByGlobal('state.init.eom.uncagingMapper.enable', 'Value', 0, 'Callback', 1);
    
    if ~any(state.init.eom.uncagingMapper.enabled)
        state.init.eom.uncagingMapper.enabled(state.init.eom.uncagingMapper.beam) = 1;
    end
    state.init.eom.uncagingMapper.quitLoop = 0;

    set(gh.uncagingMapper.loop, 'String', 'Abort');
    set(gh.uncagingMapper.loop, 'ForegroundColor', [1 0 0]);

    if state.init.eom.uncagingMapper.position > size(state.init.eom.uncagingMapper.pixels, 2)
        updateGUIByGlobal('state.init.eom.uncagingMapper.position', 'Value', 1, 'Callback', 1);
        updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', 1, 'Callback', 1);
    end
    state.init.eom.changed(:);

    set(gh.mainControls.grabOneButton, 'Enable', 'Off');
    set(gh.uncagingMapper.enableButton, 'Enable', 'Off');
    set(gh.mainControls.focusButton, 'Enable', 'Off');
    set(gh.mainControls.startLoopButton, 'Enable', 'Off');
    set(gh.uncagingPulseImporter.enableToggleButton, 'Enable', 'Off');
    
    state.init.eom.uncagingMapper.tempFrames = state.acq.numberOfFrames;
    state.init.eom.uncagingMapper.tempFrames2 = state.standardMode.numberOfFrames;
    set(gh.mainControls.grabOneButton, 'Enable', 'Off');    
    if state.init.eom.uncagingMapper.perGrab
        
        updateGUIByGlobal('state.acq.numberOfFrames', 'Value', 1);
        updateGUIByGlobal('state.standardMode.numberOfFrames', 'Value', 1, 'Callback', 1);
        
        updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', ...
            state.init.eom.uncagingMapper.position, 'Callback', 1);
        if state.init.eom.uncagingMapper.position > size(state.init.eom.uncagingMapper.pixels, 2)
            state.init.eom.uncagingMapper.position = 1;
        end
        
        while ~state.init.eom.uncagingMapper.quitLoop & ...
                state.init.eom.uncagingMapper.position <= size(state.init.eom.uncagingMapper.pixels, 2)
            
            for i = 1 : state.init.eom.numberOfBeams
                if state.init.eom.uncagingMapper.enabled(i)
                    state.init.eom.changed(i) = 1;
                    putDaqData(state.acq.dm, state.init.eom.pockelsCellNames{i}, ...
                        makePockelsCellDataOutput(i));
                end
            end
            
            if ~state.init.eom.uncagingMapper.quitLoop
                executeGrabOneCallback(gh.mainControls.grabOneButton);
            end
            
            %Check before the pause, just in case.
            if state.init.eom.uncagingMapper.quitLoop | ...
                    state.init.eom.uncagingMapper.position > size(state.init.eom.uncagingMapper.pixels, 2)
                
                updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', ...
                    state.init.eom.uncagingMapper.position, 'Callback', 1);
                
                break;
            end
            pause(state.standardMode.repeatPeriod);
            
            updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', ...
                state.init.eom.uncagingMapper.position, 'Callback', 1);
            
            if state.init.eom.uncagingMapper.position == size(state.init.eom.uncagingMapper.pixels, 2) & ...
                ~state.init.eom.uncagingMapper.quitLoop
%                 updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', ...
%                     state.init.eom.uncagingMapper.position, 'Callback', 1);
                executeGrabOneCallback(gh.mainControls.grabOneButton);
                state.init.eom.uncagingMapper.quitLoop = 1;
            end
        end
    else
        updateGUIByGlobal('state.acq.numberOfFrames', 'Value', size(state.init.eom.uncagingMapper.pixels, 2));
        updateGUIByGlobal('state.standardMode.numberOfFrames', 'Value', size(state.init.eom.uncagingMapper.pixels, 2), 'Callback', 1);
        state.init.eom.changed(:) = 1;
        executeGrabOneCallback(gh.mainControls.grabOneButton);
    end

    pause(2)%Wait for Grab to catch up.
    updateGUIByGlobal('state.acq.numberOfFrames', 'Value', state.init.eom.uncagingMapper.tempFrames);
    updateGUIByGlobal('state.standardMode.numberOfFrames', 'Value', state.init.eom.uncagingMapper.tempFrames2, 'Callback', 1);
else
    updateGUIByGlobal('state.init.eom.uncagingMapper.enable', 'Value', state.init.eom.uncagingMapper.tempEnable, 'Callback', 1);
    state.init.eom.uncagingMapper.quitLoop = 1;
    if ~strcmpi(get(gh.mainControls.grabOneButton, 'String'), 'Grab')
        executeGrabOneCallback(gh.mainControls.grabOneButton);
    end
    pause(2)%Wait for Grab to catch up.
    updateGUIByGlobal('state.acq.numberOfFrames', 'Value', state.init.eom.uncagingMapper.tempFrames);
    updateGUIByGlobal('state.standardMode.numberOfFrames', 'Value', state.init.eom.uncagingMapper.tempFrames2, 'Callback', 1);
end

set(gh.mainControls.grabOneButton, 'Enable', 'On');
set(gh.uncagingMapper.enableButton, 'Enable', 'On');
set(gh.mainControls.focusButton, 'Enable', 'On');
set(gh.mainControls.startLoopButton, 'Enable', 'On');
set(gh.uncagingPulseImporter.enableToggleButton, 'Enable', 'On');

set(gh.uncagingMapper.loop, 'String', 'Loop');
set(gh.uncagingMapper.loop, 'ForegroundColor', [0 0 1]);

if state.init.eom.uncagingMapper.position > size(state.init.eom.uncagingMapper.pixels, 2)
    state.init.eom.uncagingMapper.position = 1;
    updateGUIByGlobal('state.init.eom.uncagingMapper.pixel', 'Value', state.init.eom.uncagingMapper.position, ...
        'Callback', 1);
end

return;


% --- Executes on button press in showPixels.
function showPixels_Callback(hObject, eventdata, handles)

genericCallback(hObject);
updatePixelDisplay;

return;


% --- Executes on button press in shutterBlank.
function shutterBlank_Callback(hObject, eventdata, handles)
global gh;

genericCallback(hObject);

set(gh.uncagingMapper.autoButton, 'ForeGround', [1 0 0]);

return;


% --- Executes on button press in singleFrame.
function singleFrame_Callback(hObject, eventdata, handles)
global gh;

genericCallback(hObject);
set(hObject, 'Enable', 'Inactive');
set(gh.uncagingMapper.perGrabRadioButton, 'Enable', 'On');
updateGUIByGlobal('state.init.eom.uncagingMapper.perGrab', 'Value', 0);
set(gh.uncagingMapper.perGrabRadioButton, 'Value', 0);
set(gh.uncagingMapper.loop, 'Enable', 'Off');

%TO051507A
set(gh.uncagingMapper.perFrameRadioButton, 'Enable', 'On');
updateGUIByGlobal('state.init.eom.uncagingMapper.perFrame', 'Value', 0);
set(gh.uncagingMapper.perFrameRadioButton, 'Value', 0);

return;