% daqmanager/saveobj - Produce a saveable form of this object.
%
%  This function is for Matlab to call when saving objects to m-files. It is not
%  intended to be used at any other time or for any other purpose.
%
% Created 5/4/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function this = saveobj(this)
global gdm;

this.serialized = gdm(this.ptr);

%Don't save figures.
figurenames = fieldnames(this.serialized.figures)
for i = 1 : length(figurenames)
    this.serialized.(figurenames{i}) = [];
end

%These can't be saved, because they might cause recursive loops.
%If the daqmanager is a variable in an object that is registered as a listener, 
%it'll be saving itself forever.
this.serialized.channelCreationListeners = {};
this.serialized.channelRemovalListeners = {};
this.serialized.channelStartListener = {};
this.serialized.channelStopListener = {};

%Don't save the @daqdevice instances.
rmfield(this.serialized, 'aos');
rmfield(this.serialized, 'ais');

%Don't save the pointer.
this.ptr = -1;

return;