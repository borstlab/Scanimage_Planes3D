function updateMaxPowerDisplay(beam,maxPowerValue)
%% function power = updateMaxPowerDisplay(beam,maxPowerValuegin)
%
%% CREDITS
%  Created by Vijay Iyer, 1/6/2010
%% ****************************************************************

global state gh;

if beam == state.init.eom.beamMenu
    
    if nargin < 2 || isempty(maxPowerValue)
        maxPowerValue = state.init.eom.maxPower(beam);
    end
    
    %Make sure the units are correct.
    if get(gh.powerControl.mW_radioButton, 'Value') == get(gh.powerControl.mW_radioButton, 'Max')
        conversion = state.init.eom.(['powerConversion' num2str(beam)]) * state.init.eom.maxPhotodiodeVoltage(beam) * .01;
        state.init.eom.maxPowerDisplay = round(1 / conversion * maxPowerValue);
    else
        state.init.eom.maxPowerDisplay = maxPowerValue;
    end
    
    updateGUIByGlobal('state.init.eom.maxPowerDisplay');
end

