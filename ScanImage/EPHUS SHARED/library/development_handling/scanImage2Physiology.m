% This is a very big kluge, just to get old ScanImage and new Physiology working together.
%
% USAGE
%  scanImage2Physiology
%
% NOTES
%  This should be called immediately prior to `dioTrigger` in ScanImage.
%
% CHANGES
%
% Created 1/26/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function scanImage2Physiology
global loopManagers state;

this.ptr = 1;%Hope there's only one instance.
fireEvent(loopManagers(this.ptr).callbackManager, 'objectUpdate');%TO100705E - Do this before the actual iteration. -- Tim O'Connor 10/7/05
fireEvent(loopManagers(this.ptr).callbackManager, 'loopIterate', eventdata);

pulseNameArray = getGlobal(progmanager, 'pulseNameArray', 'ephys', 'ephys');
state.physiology.pulseNumber = getNumericSuffix(pulseNameArray{1});

%Initiate
trigger(startmanager('acquisition'));

return;