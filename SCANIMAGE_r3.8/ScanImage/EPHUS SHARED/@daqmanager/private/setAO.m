%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Put an AO object into place.
%%
%%  OBJ = putAO(OBJ, ao)
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%           1/17/06 Tim O'Connor TO011706D: Take into account NIDAQmx labelling scheme.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dm = addOutputChannel(dm, ao)
global gdm;

info = daqhwinfo(ao);

boardId = getNumericSuffix(info.ID);%TO011706D - Take into account NIDAQmx labelling scheme. -- Tim O'Connor 1/17/06
gdm(dm.ptr).aos{boardId} = ao;

return;