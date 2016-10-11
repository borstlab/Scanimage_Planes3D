% @callbackmanager/getDocumentation - Retrive any integrated documentation for a specific event.
%
%  SYNTAX
%   documentation - getDocumentation(cbm, eventName)
%    cbm - A @callbackmanager instance.
%    eventName - The event whose documentation to retrieve.
%    documentation - A string to be used to document this event.
%                    Specifically, this is meant to be viewable with the userFcns gui.
%
%  USAGE
%
%  CHANGES
%
% Created 5/30/08 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2008
function documentation = getDocumentation(this, eventName)
global callbackmanagers;

if ~isempty(callbackmanagers(this.ptr).callbacks)
    index = find(strcmpi({callbackmanagers(this.ptr).callbacks{:, 1}}, eventName));
    if isempty(index)
        warning('Event''%s'' not found.', event);
        return;
    end
end

documentation = callbackmanagers(this.ptr).callbacks{index, 3};

return;