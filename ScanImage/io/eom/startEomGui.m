
function fig = startEomGui(varargin)
%% function fig = startEomGui(varargin);
% Setup the environment for using Pockels cell(s).
%
%% NOTES
%   Completely rewritten to use new DAQmx interface. MOLD file contains earlier version.  -- Vijay Iyer 9/5/09
%
%% CHANGES
%    10/17/08 Vijay Iyer (VI101708A) - Determine the number of 'beams' from the INI file, rather than having it separately specified
%    10/20/08 Vijay Iyer (VI102008A) - Eliminate use of scanLaserBeam state variable
%    1/17/09 Vijay Iyer (VI011709A) - Initialize new PowerBox GUI as needed
%    2/10/09 Vijay Iyer (VI021009A) - Allow beams to be named from the INI file
%    3/26/09 Vijay Iyer (VI032609A) - Initialize the state.init.eom.calibrated variable containing flags indicated beams that have been calibrated
%    5/13/09 Vijay Iyer (VI051309A) - Handle power vs Z feature
%    8/26/09 Vijay Iyer (VI082609A) - Moved Pockels/photodiode AO/AI board&channel configuration and related code to setupAOObjects_Common/setupAIObjects_Common
%    9/9/09 Vijay Iyer (VI090909A) - Initialize state.init.eom.cancel, to avoid error with newly written calibrateEOM()
%    1/8/10 Vijay Iyer (VI010810A) - Must initialize state.init.eom.powerLzStoredArray now as well to avoid error when multiple beams
%    1/12/10 Vijay Iyer (VI011210A) - Handle control visibility only in enableEomGui() now
%    VI032311A: Initialize new photodiodeInvert state var based on new model INI file vars -- Vijay Iyer 3/23/11
%
%% CREDITS
%   Created 9/5/09, by Vijay Iyer
%   Based heavily on earlier version by Tom Pologruto/Tim O'Connor
%% *************************************************************

global state gh

%Setup the initial variables (these were originally in the standard.ini
state.init.eom.cancel = 0; %VI090909A
state.init.eom.started = 0;
state.init.eom.lut = [];
state.init.eom.min = 1; % This will change once calibrated.
state.init.eom.calibrated = zeros(state.init.eom.maxNumberOfBeams,1); %VI032609A

state.init.eom.changed = zeros(1,state.init.eom.numberOfBeams); %VI102008A

%%%VI032311A%%%%
state.init.eom.photodiodeInvert = false(state.init.eom.numberOfBeams,1);
for i=1:state.init.eom.numberOfBeams
    fieldName = sprintf('photodiodeInputNegative%d',i);    
    state.init.eom.photodiodeInvert(i) = isfield(state.init.eom,fieldName) && state.init.eom.(fieldName);
end        

%%%Set up power vs Z (VI051309A)%%%%%%
%state.init.eom.powerVsZEnableArray = zeros(1,state.init.eom.numberOfBeams);
state.init.eom.powerLzArray = repmat(inf,[1 state.init.eom.numberOfBeams]);
state.init.eom.powerLzStoredArray = state.init.eom.powerLzArray; %VI010810A
state.init.eom.powerVsZActive = state.motor.motorOn; %Ideally would also validate that Z dimension is available
%%VI011210A: Relocated%%%
% if ~state.init.eom.powerVsZActive
%    set(get(gh.powerControl.pnlPowerVsZ,'Children'),'Enable','off');
% end
%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

beams = cell(state.init.eom.numberOfBeams, 1);
[state.init.eom.maxPower, state.init.eom.maxLimit] = deal(zeros(1, state.init.eom.numberOfBeams));

for i = 1:state.init.eom.numberOfBeams
    state.init.eom.pockelsCellNames{i} = strcat('PockelsCell-', num2str(i));
    beams{i} = state.init.eom.(['beamName' num2str(i)]); %VI021009A
    
    state.init.eom.maxPower(i) = state.init.eom.(['maxPower' num2str(i)]);
    state.init.eom.maxLimit(i) = state.init.eom.(['maxLimit' num2str(i)]);
 
    state.init.eom.constrainBoxToLine(i) = 0;    
end

set(gh.powerControl.beamMenu, 'String', beams);
set(gh.powerControl.beamMenuSlider, 'Min', 1);
set(gh.powerControl.beamMenuSlider, 'Max', state.init.eom.numberOfBeams + 1);
step = 1 / state.init.eom.numberOfBeams;
set(gh.powerControl.beamMenuSlider, 'SliderStep', [step step]);
set(gh.powerControl.beamMenuSlider, 'Val', 1);

%Set Power Box beam menu
set(gh.powerBox.pmBeamMenu, 'String', beams); %VI011709A

%Initialize Laser Function Panel
try
    feval(state.init.eom.laserFunctionPanel.updateDisplay);
catch
    warning('Failed to execute: %s\n  %s', func2str(state.init.eom.laserFunctionPanel.updateDisplay), lasterr);
end

%Initialize arrays for PowerBox feature
arrayNames = {'showBoxArray' 'boxPowerArray' 'startFrameArray' 'endFrameArray'};
for i=1:length(arrayNames)
    baseName = regexpi(arrayNames{i},'(.*)Array','tokens','once');
    baseName = baseName{1}; 
    state.init.eom.(arrayNames{i}) =  ones(1,state.init.eom.numberOfBeams) * state.init.eom.(baseName);    
end

%Other initialization
powerControl('beamMenu_Callback',gh.powerControl.beamMenu);
state.init.eom.boxcolors=hsv(state.init.eom.numberOfBeams);

%Make sure everything plays nice.
ensureEomGuiStates;
