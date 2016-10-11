function power = updatePowerReadout(varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%  Measures and returns the current power from the 
%  photodiode and updates the display appropriately.
%
%  Takes the beamline(s) as an argument. If none is supplied
%  it analyzes the current beam being displayed in the GUI.
%
%  The return value is the current power(s).
%
%  This may be enabled/disabled with the state.init.eom.updatePowerContinuously variable:
%    0 == 'disabled'
%    1 == 'enabled'
%
%  Note: It does not, as of now, update the power setting. 
%        Instead, it just updates the display.
%
%
%%  CHANGES
%     Tim O'Connor 11/24/03 - Allow this feature to be disabled in the standard.ini
%     Tim O'Connor 12/23/03 - Make sure we're not in the middle of an acquisition.
%     TPMOD_1: Modified 12/31/03 Tom Pologruto - checks to see if we want
%     to update continuously or not.  LEaving this off greatly increases
%     the speed.
%     VI103108A Vijay Iyer 10/31/08 - state.init.eom.ai is now a cell array
%     VI103108B Vijay Iyer 10/31/08 - Handle case where particular beam does not have a photodiode
%     VI032311A Vijay Iyer 3/23/11 - Handle case where photodiode voltage signal is inverted
%
%% CREDITS
%  Created - Tim O'Connor 9/19/03
%  Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute
%% ****************************************************************
global state gh;
% start TPMOD_1 12/31/03
if  ~state.init.eom.updatePowerContinuously
    return
end
% end TPMOD_1 12/31/03

%Quit if an acquisition is in progress. -- Tim O'Connor 12/23/03
if strcmpi(get(gh.mainControls.grabOneButton, 'String'), 'ABORT') | ...
        strcmpi(get(gh.mainControls.startLoopButton, 'String'), 'ABORT') | ...
        strcmpi(get(gh.mainControls.focusButton, 'String'), 'ABORT')
    return;
end

%Do all beams in the list, one at a time.
if length(varargin) > 1
    for i = 1:length(varargin)
        power(i) = updatePowerReadout(varargin(i));
    end
    
    return;
end

%No beams were specified, so just use the current one.
if length(varargin) == 0
    beam = state.init.eom.beamMenu;
else
    beam = varargin{1};
end

%If this feature is disabled, just return the power that has been set.
if ~state.init.eom.updatePowerContinuously
    power = state.init.eom.maxPower(beam);
    return;
end

%%%VI103108B: Deal with beams lacking photodiode
if isempty(state.init.eom.ai{beam})
    power = state.init.eom.maxPower(beam);
    return;    
end
%%%%%%%%%%%%%%%%

%Setup the photodiode.
set(state.init.eom.ai{beam}, 'SampleRate', state.internal.eom.calibrationSampleRate);
set(state.init.eom.ai{beam}, 'SamplesPerTrigger', round(state.internal.eom.calibrationSampleRate / 2) + 1);%Take a positive integer number of samples (>= 1 second).
tType = get(state.init.eom.ai{beam}, 'TriggerType');
set(state.init.eom.ai{beam}, 'TriggerType', 'Manual');

%Get the voltage.
start(state.init.eom.ai{beam});
trigger(state.init.eom.ai{beam});
power = mean(getdata(state.init.eom.ai{beam}));
stop(state.init.eom.ai{beam});

%Put the photodiode back the way it was.
set(state.init.eom.ai{beam}, 'TriggerType', tType);

%Watch out for wierdness, such as the power being higher than the
%maximum seen during calibration.

%%%VI032311A
if state.init.eom.photodiodeInvert(beam)
    power = -power;
end

if power > state.init.eom.maxPhotodiodeVoltage(beam) & abs(power - state.init.eom.maxPhotodiodeVoltage(beam)) > .015

    beep;
    fprintf(2, '\nWARNING: Current photodiode voltage for beam %s is greater than the maximum at the time of Pockels cell calibration.\n @Calibration: %s [V]\n @Now: %s [V]\n', ...
         num2str(beam), num2str(state.init.eom.maxPhotodiodeVoltage(beam)), num2str(power));
    fprintf(2, 'Recalibration of the Pockels cell is recommended.\n');
    fprintf(2, 'If the problem persists, possible causes may be:\n  improperly setup optics\n  electronic noise (in amplifiers, etc)\n  laser is out of mode-lock\n\n');

    return;
end

%Convert it, into a percentage of max.
power = power / state.init.eom.maxPhotodiodeVoltage(beam);

%Bump the limit, if necessary.
if state.init.eom.maxLimit(beam) < power
    state.init.eom.maxLimit(beam) = power;

    %Update the GUI, if necessary.
    if beam == state.init.eom.beamMenu
        set(gh.powerControl.maxLimit, 'String', num2str(power));
        set(gh.powerControl.maxLimit_Slider, 'Value', power);
    end
end

%Sync the variable.
%NOTE: Should this be done, since it could lead to a continuous change in power, given a constant offset on the diode?
%      I'll leave it out for now, but it may be worth putting back after some careful thought.
%state.init.eom.maxPower(beam) = power;

%Update the GUI, if necessary.
if beam == state.init.eom.beamMenu
    
    %Make sure the units are correct.
    if get(gh.powerControl.mW_radioButton, 'Value') == get(gh.powerControl.mW_radioButton, 'Max')%in mW 
        conversion = getfield(state.init.eom, ['powerConversion' num2str(beam)]) * state.init.eom.maxPhotodiodeVoltage(beam) * .01;
        state.init.eom.maxPowerDisplay = round(1 / conversion * state.init.eom.maxPowerDisplay);
    else
        state.init.maxPowerDisplay = power;
    end
    
    updateGUIByGlobal('state.init.maxPowerDisplay');
end

return;