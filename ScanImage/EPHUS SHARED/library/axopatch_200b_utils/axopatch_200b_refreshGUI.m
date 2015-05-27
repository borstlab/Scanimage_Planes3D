% AMPLIFIER/axopatch_200b_refreshGUI - Update a GUI to match the amplifier.
%
% SYNTAX
%  axopatch_200b_refreshGUI(hObject)
%
% USAGE
%
% NOTES
%
% CHANGES
%
% Created 3/28/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function axopatch_200b_refreshGUI(hObject)

amp = getLocal(progmanager, hObject, 'amplifier');
if isempty(amp)
    setLocal(progmanager, hObject, 'gain_daq_board_id', 1);
    setLocal(progmanager, hObject, 'mode_daq_board_id', 1);
    setLocal(progmanager, hObject, 'v_hold_daq_board_id', 1);
    setLocal(progmanager, hObject, 'gain_channel', 0);
    setLocal(progmanager, hObject, 'mode_channel', 0);
    setLocal(progmanager, hObject, 'v_hold_channel', 0);
    setLocal(progmanager, hObject, 'i_clamp_input_factor', 1);
    setLocal(progmanager, hObject, 'v_clamp_input_factor', 1);
    setLocal(progmanager, hObject, 'i_clamp_output_factor', 1);
    setLocal(progmanager, hObject, 'v_clamp_output_factor', 1);
    setLocal(progmanager, hObject, 'scaledOutputBoardID', 0);
    setLocal(progmanager, hObject, 'scaledOutputChannelID', 1);
    setLocal(progmanager, hObject, 'vComBoardID', 0);
    setLocal(progmanager, hObject, 'vComChannelID', 1);
    return;
end

setLocal(progmanager, hObject, 'gainBoard', get(amp, 'gain_daq_board_id'));
setLocal(progmanager, hObject, 'modeBoard', get(amp, 'mode_daq_board_id'));
setLocal(progmanager, hObject, 'vHoldBoard', get(amp, 'v_hold_daq_board_id'));
setLocal(progmanager, hObject, 'gainChannel', get(amp, 'gain_channel'));
setLocal(progmanager, hObject, 'modeChannel', get(amp, 'mode_channel'));
setLocal(progmanager, hObject, 'vHoldChannel', get(amp, 'v_hold_channel'));
setLocal(progmanager, hObject, 'iClampInputFactor', get(amp, 'i_clamp_input_factor'));
setLocal(progmanager, hObject, 'vClampInputFactor', get(amp, 'v_clamp_input_factor'));
setLocal(progmanager, hObject, 'iClampOutputFactor', get(amp, 'i_clamp_output_factor'));
setLocal(progmanager, hObject, 'vClampOutputFactor', get(amp, 'v_clamp_output_factor'));
setLocal(progmanager, hObject, 'scaledOutputBoardID', get(amp, 'scaledOutputBoardID'));
setLocal(progmanager, hObject, 'scaledOutputChannelID', get(amp, 'scaledOutputChannelID'));
setLocal(progmanager, hObject, 'vComBoardID', get(amp, 'vComBoardID'));
setLocal(progmanager, hObject, 'vComChannelID', get(amp, 'vComChannelID'));

setLocal(progmanager, hObject, 'changesMade', 0);
setLocalGh(progmanager, hObject, 'applyChanges', 'Enable', 'Off');

return;