% AMPLIFIER/axopatch_200b_refreshGUI - Bind an amplifier.
%
% SYNTAX
%  axopatch_200b_setAmplifier(hObject, AMPLIFIER)
%
% USAGE
%  Bind an AXOPATCH200B amplifier to an associated configuration GUI.
%
% NOTES
%
% CHANGES
%
% Created 3/28/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function axopatch_200b_setAmplifier(hObject, amp)

setLocal(progmanager, hObject, 'amplifier', amp);
axopatch_200b_refreshGUI(hObject);

return;