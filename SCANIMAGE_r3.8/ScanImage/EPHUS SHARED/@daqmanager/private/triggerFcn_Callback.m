% @daqmanager/triggerFcn_Callback - A daqtoolbox callback event handler.
%
% SYNTAX
%  startFcn_Callback(obj, eventdata, dm)
%   obj - The daqobject that initiated the event.
%   eventdata - The eventdata supplied by the daqtoolbox.
%   dm - The @daqmanager instance for the event initiating daqobject.
%
% USAGE
%
% NOTES
%  Moved from a subfunction in startChannel to a private function in the flass.
%
% CHANGES
%  TO080606A: See @daqmanager/startChannel for changes prior to refactoring. -- Tim O'Connor 8/6/06
%
% Created 8/6/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function triggerFcn_Callback(obj, eventdata, dm)
global gdm;

for i = 1 : length(obj.Channel)
    fireEvent(gdm(dm.ptr).cbm, [obj.Channel(i).ChannelName 'Trigger'], obj.Channel(i).ChannelName, obj, eventdata);
end

return;