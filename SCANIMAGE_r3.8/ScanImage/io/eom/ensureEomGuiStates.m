%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Checks the state of all the Pockels cell related variables,
%% and makes sure that they are correct, relative to one another.
%% Takes no arguments, returns no results.
%%
%% pre - Calibrated.
%% post - 0 < eom.min <= eom.maxPower <= eom.maxLimit <= 100
%%        eom.min < eom.maxLimit
%%        gh.powerControl.maxPower_Slider.Min = eom.min + 1
%%        gh.powerControl.maxPower_Slider.Max = eom.maxLimit
%%        gh.powerControl.maxLimit_Slider.Min = eom.min + 1
%% CHANGES
% Updated - Tim O'Connor 9/19/03 :: 'Call updatePowerReadout at end.'
% Updated - Tim O'Connor 2/18/04 TO21804a :: Allow power box to work in mW.
% Updated - Tim O'Connor 2/18/04 TO21804c :: Add options to control interaction between powerControl and uncagingPulseImporter GUIs.
%   TO22704a Tim O'Connor 2/27/04 - Created uncagingMapper.
%   TO042304b Tim O'Connor 4/23/04 - Created laserFuntionPanel.
%   VI070808A Vijay Iyer 7/08/08 - Corrected apparent indexing error in setting slider limits
%   VI070808B Vijay Iyer 7/08/08 - Ensure that state.init.eom.min remains an integer value
%   VI011709A Vijay Iyer 1/17/09 - Handle linkage to new Power Box GUI
%   VI051309A Vijay Iyer 5/13/09 - Add handling for Power vs Z feature
%   VI083109A Vijay Iyer 8/31/09 - Handle changes to new DAQmx interface
%   VI112309A Vijay Iyer 11/23/09 - Only update power upon power level change if 'direct mode' or 'verify power' is ON; reset power to minimum if direct mode is OFF
%   VI122309A Vijay Iyer 12/23/09 - Actually implement VI112309A (merged from 3.6) now in DAQmx-compliant manner, using newly separated park Tasks for each Pockels beam
%   VI122909A Vijay Iyer 12/29/09 - Implement VI112309A/VI122309A without using scim_parkLaser() which causes unneeded call to park mirrors, creating DAQmx resrouce conflict 
%   VI122909B Vijay Iyer 12/29/09 - BUGFIX -- use 'maxPower' and 'min' values correctly as indices into beam's LUT
%   VI010810A Vijay Iyer 01/08/10 - Enable P vs z is now a scalar, rather than a vector, and applies to all beams
%
%% CREDITS
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute
%% *****************************************************************************
function ensureEomGuiStates(beams);
global gh;

displayedBeam = get(gh.powerControl.beamMenu, 'Value');
if nargin == 0
    ensure(displayedBeam);
else
    
    assert(ismember(displayedBeam,beams));
    
    %Ensure displayedBeam is handled last, so display is handled correctly
    beams(beams==displayedBeam) = [];
    beams(end+1) = displayedBeam;
    
    for i = 1 : length(beams)
        ensure(beams(i));
    end
end

return;

% --------------------------------------------------------------------
function ensure(beam)
global gh state
oldPower=state.init.eom.maxPower(beam);

%Make sure maxLimit is okay.
if  state.init.eom.maxLimit(beam) <= state.init.eom.min(beam)
    state.init.eom.maxLimit(beam) = state.init.eom.min(beam) + 1;
end
if state.init.eom.maxLimit(beam) > 100
    state.init.eom.maxLimit(beam) = 100;        
end

%Make sure maxPower is okay.
if state.init.eom.maxPower(beam) < state.init.eom.min(beam)
    state.init.eom.maxPower(beam) = state.init.eom.min(beam);
end
if state.init.eom.maxPower(beam) > state.init.eom.maxLimit(beam)
    state.init.eom.maxPower(beam) = state.init.eom.maxLimit(beam);
end

maxPowerStep(1) = 1;
maxPowerStep(2) = 1;
if (state.init.eom.maxLimit(beam) - state.init.eom.min(beam)) == 0
    fprintf(2, 'WARNING: The pockels cell extinction ratio may have degraded.\n');
    if state.init.eom.maxLimit(beam) < 100
        state.init.eom.maxLimit(beam) = state.init.eom.maxLimit(beam) + 1;
    else
        %state.init.eom.min(beam) = state.init.eom.maxLimit(beam) - .0001; %VI070808B
        state.init.eom.min(beam) = state.init.eom.maxLimit(beam) - 1; %VI070808B
    end
end

   
maxPowerStep(1) = 1 / (state.init.eom.maxLimit(beam) - state.init.eom.min(beam));
maxPowerStep(2) = 10 / (state.init.eom.maxLimit(beam) - state.init.eom.min(beam));

if maxPowerStep(1) == Inf | maxPowerStep(1) == NaN
    maxPowerStep(1) = 1;
end
if maxPowerStep(2) == Inf | maxPowerStep(2) == NaN
    maxPowerStep(2) = 1;
end

%These settings should gaurantee no stupid warnings, and will get replaced immediately after this.
set(gh.powerControl.maxPower_Slider, 'Max', 101);

%Update the maxPower_Slider
if any(maxPowerStep < 0) | any(maxPowerStep > 1)
    maxPowerStep = [.1 .5];
end
if state.init.eom.min(beam) < 100
    set(gh.powerControl.maxPower_Slider, 'SliderStep', maxPowerStep);
    set(gh.powerControl.maxPower_Slider, 'Min', state.init.eom.min(beam));
    set(gh.powerControl.maxPower_Slider, 'Max', state.init.eom.maxLimit(beam));
else
    setDummyValues;
end

%maxLimitStep(beam) = 10 / (100 - state.init.eom.min(beam)); %VI070808A
maxLimitStep(1) = 10 / (100 - state.init.eom.min(beam)); %VI070808A
if maxLimitStep(1) == Inf | maxLimitStep(1) == NaN | max(maxLimitStep(1)) > 1
    maxLimitStep(1) = 1;
end
maxLimitStep(2) = maxLimitStep(1);

%Update the maxLimit_Slider
if (state.init.eom.min(beam) + 1) <  100
    set(gh.powerControl.maxLimit_Slider, 'SliderStep', maxLimitStep);
    set(gh.powerControl.maxLimit_Slider, 'Min', state.init.eom.min(beam) + 1);
    set(gh.powerControl.maxLimit_Slider, 'Max', 100);
    set(gh.powerControl.maxLimit_Slider, 'Value', state.init.eom.maxLimit(beam));
else
    setDummyValues;
end

%Update the text readout(s).
set(gh.powerControl.maxLimit, 'String', num2str(state.init.eom.maxLimit(beam)));
state.init.eom.maxPowerDisplaySlider = state.init.eom.maxPower(beam);
updateGUIByGlobal('state.init.eom.maxPowerDisplaySlider');

%Convert display to mW or %
conversion = 1;
if get(gh.powerControl.mW_radioButton, 'Value') == get(gh.powerControl.mW_radioButton, 'Max')
    conversion = getfield(state.init.eom, ['powerConversion' num2str(beam)]) * ...
        state.init.eom.maxPhotodiodeVoltage(beam) * .01;

    %TO22704a
    set(gh.uncagingMapper.powerText, 'TooltipString', 'Power per pulse in mW.');
    set(gh.uncagingMapper.autoPowerText, 'TooltipString', 'Power per pulse in mW.');
    
    set(gh.uncagingMapper.autoPowerLabel, 'String', 'Power [mW]');
    set(gh.uncagingMapper.powerLabel, 'String', 'Power [mW]');
    
    %%%VI011709A%%%%%%
    %sset(gh.powerControl.powerBoxText, 'String', 'Power [mW]');
    set(gh.powerBox.pnlPowerLevels, 'Title', 'Power Levels [mW]');
    %%%%%%%%%%%%%%%%%%
else    
    %TO22704a
    set(gh.uncagingMapper.powerText, 'TooltipString', 'Power per pulse in % of maximum.');
    set(gh.uncagingMapper.autoPowerText, 'TooltipString', 'Power per pulse in % of maximum.');
    
    
    set(gh.uncagingMapper.autoPowerLabel, 'String', 'Power [%]');
    set(gh.uncagingMapper.powerLabel, 'String', 'Power [%]');
    
    %%%VI011709A%%%%%%
    %set(gh.powerControl.powerBoxText, 'String', 'Power [%]');
    set(gh.powerBox.pnlPowerLevels, 'Title', 'Power Levels [%]');
    %%%%%%%%%%%%%%%%%%
    
    conversion2 = getfield(state.init.eom, ['powerConversion' num2str(beam)]) * ...
        state.init.eom.maxPhotodiodeVoltage(beam) * .01;
end

%%%VI051309A: Update Power vs Z settings
%set(gh.powerControl.cbEnablePvsZ, 'Value', state.init.eom.powerVsZEnableArray(beam)); %VI010810A
if numel(state.init.eom.powerLzStoredArray) >= beam 
    state.init.eom.powerLz = state.init.eom.powerLzStoredArray(beam);
else
    state.init.eom.powerLz = inf;
end
updateGUIByGlobal('state.init.eom.powerLz');
%%%%%%%%%%%%%%%%%%%%%%%

if size(state.init.eom.uncagingMapper.pixels, 3) == 4 & ...
        size(state.init.eom.uncagingMapper.pixels, 2) >= state.init.eom.uncagingMapper.pixel & ...
        size(state.init.eom.uncagingMapper.pixels, 1) >= state.init.eom.uncagingMapper.beam
    set(gh.uncagingMapper.powerText, 'String', num2Str(round(state.init.eom.uncagingMapper.pixels(state.init.eom.uncagingMapper.beam, ...
        state.init.eom.uncagingMapper.pixel, 4) * conversion)));
end
set(gh.uncagingMapper.autoPowerText, 'String', num2str(round(state.init.eom.uncagingMapper.autoPower * conversion)));
set(gh.uncagingMapper.powerText, 'String', num2str(round(state.init.eom.uncagingMapper.power * conversion)));

%TO22704a
if state.init.eom.uncagingMapper.enable
    set(gh.uncagingMapper.enableButton, 'String', 'Disable');
    set(gh.uncagingMapper.enableButton, 'ForegroundColor', [1 0 0]);
else
    set(gh.uncagingMapper.enableButton, 'String', 'Enable');
    set(gh.uncagingMapper.enableButton, 'ForegroundColor', [0 .6 0]);
end

%Max power text box.
state.init.eom.maxPowerDisplay = round(conversion * state.init.eom.maxPower(beam));
updateGUIByGlobal('state.init.eom.maxPowerDisplay');

%%%VI011709B: Removed %%%%%%
% %TO21804c - Added user preferences to control interactions between powerControl and uncagingPulseImporter.
% if state.init.eom.linkMaxAndBoxPower
%     state.init.eom.boxPowerArray(beam) = state.init.eom.maxPower(beam);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI011709B%%%%%%%%%%%%%%%%%
updateGUIByGlobal('state.init.eom.lockBoxOnToMax','Callback',1);
updateGUIByGlobal('state.init.eom.lockBoxOffToMin','Callback',1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Watch out for the dreaded "index exceeds matrix dimensions".
if length(state.init.eom.boxPowerArray) >= state.init.eom.beamMenu
    %Power box text box.
    %Added to allow power box to work in mW. -- Tim O'Connor TO21804a
    state.init.eom.boxPower = round(conversion * state.init.eom.boxPowerArray(state.init.eom.beamMenu));
    updateGUIByGlobal('state.init.eom.boxPower');
end

%changed something?
if state.init.eom.maxPower(beam) ~= oldPower
    state.init.eom.changed(beam)=1; %If this is getting called, something must've changed.
end

if get(gh.powerControl.maxLimit_Slider, 'Min') >= get(gh.powerControl.maxLimit_Slider, 'Max')
    setDummyValues;
end
if get(gh.powerControl.maxPower_Slider, 'Min') >= get(gh.powerControl.maxPower_Slider, 'Max')
    setDummyValues;
end

%Maybe try setting the Power now?
if state.init.eom.changed(beam)

    val = get(gh.mainControls.focusButton, 'String');

    if strcmpi(val, 'FOCUS') && (state.init.eom.directMode || state.init.eom.updatePowerContinuously) %VI112309A % not focusing now....
        %setPockelsVoltage(beam, state.init.eom.lut(beam, state.init.eom.maxPower(beam))); %VI083109A
        state.init.eom.(['hAOPark' num2str(beam)]).writeAnalogData(state.init.eom.lut(beam,state.init.eom.maxPower(beam)),1,true); %VI122909B %VI122309A 
    end

    %Make sure the power is really updated properly.
     updatePowerReadout(beam);
     
    %%%VI112309A: Set power to newly-calibrated minimum, if not in direct mode
    if ~state.init.eom.directMode && strcmpi(val,'FOCUS') 
        %scim_parkLaser(); %VI122909A
        state.init.eom.(['hAOPark' num2str(beam)]).writeAnalogData(state.init.eom.lut(beam,state.init.eom.min(beam)),1,true); %VI122909A\B
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%
end

%Added. -- Tim O'Connor 4/23/04 TO042304b
try
    feval(state.init.eom.laserFunctionPanel.updateDisplay);
catch
    fprintf(2,'Failed to execute: %s\n  %s', func2str(state.init.eom.laserFunctionPanel.updateDisplay), lasterr);
end

return;

%-------------------------------------------------------------------------
function setDummyValues
global gh;

    set(gh.powerControl.maxPower_Slider, 'Max', 100);
    set(gh.powerControl.maxPower_Slider, 'Min',  99);
    set(gh.powerControl.maxPower_Slider, 'Value', 99);
    fprintf(2, 'WARNING: Pockels cell calibration is invalid. Setting dummy values in the PowerControl gui.\n');
    
return;