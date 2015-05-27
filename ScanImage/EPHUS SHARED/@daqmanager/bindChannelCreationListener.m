% DAQMANAGER/bindChannelCreationListener - Register a channel creation listener.
%
% SYNTAX
%   bindChannelCreationListener(DAQMANAGER, listenerFunction, listenerFunctionID)
%     DAQMANAGER - object
%     listenerFunction - Function to get executed on channel start.
%     listenerFunctionID - A unique identifier (number, string, or object with an eq method) to label the callback.
%
% NOTES
%
% CHANGES
%  TO112205C - Implement lifecycle listeners using the @callbackManager. -- Tim O'Connor 11/22/05
%
% Created 1/20/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function bindChannelCreationListener(this, listenerFunction, listenerFunctionID)
global gdm;

%TO112205C
addCallback(gdm(this.ptr).cbm, 'channelCreation', listenerFunction, listenerFunctionID);

% if ~isnumeric(listenerFunctionID)
%     if ~strcmpi(class(listenerFunctionID), 'char')
%         if ~ismethod(listenerFunctionID, 'eq')
%             error('A listener function ID must be a unique identifier (number, string, or object with an eq method).');
%         end
%     end
% end
% 
% index = size(gdm(this.ptr).channelCreationListeners, 1) + 1;
% gdm(this.ptr).channelCreationListeners{index, 1} = listenerFunctionID; 
% gdm(this.ptr).channelCreationListeners{index, 2} = listenerFunction;

return;