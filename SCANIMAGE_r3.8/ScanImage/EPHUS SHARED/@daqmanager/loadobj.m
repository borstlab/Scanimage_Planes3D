% daqmanager/loadobj - Retrieve a saved form of this object.
%
%  This function is for Matlab to call when loading objects from m-files. It is not
%  intended to be used at any other time or for any other purpose.
%
% Created 5/4/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function this = loadobj(this)
global gdm;

this.serialized
dm = daqmanager(this.serialized.adaptor);
this.ptr = dm.ptr;

fields = fieldnames(this.serialized);
for i = 1 : length(fields)
    gdm(this.ptr).(fields{i}) = this.serialized.(fields{i});
end
this.serialized = [];

return;