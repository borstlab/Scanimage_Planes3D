% registerUserFcn - Binds a user function to an event.
%
% SYNTAX
%  registerUserFcn(eventname, callbackSpec, callbackID)
%   eventname - The name of the event of interest.
%               By convention, event names should start with the program name, followed by a colon (':'), followed by
%               the event name itself.
%   callbackSpec - A valid callback (see @callbackManager/addCallback for details).
%   callbackID - A unique identifier (see @callbackManager/addCallback for details).
%
% USAGE
%  Programs are free to pass arguments to the callbacks. The preferred semantics involve passing only the hObject for the
%  given program. No firm rules are (as of 12/5/05) in place regarding arguments, this may change at a later date.
%
% NOTES
%
% CHANGES
%
% Created 12/5/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function registerUserFcn(eventname, callbackSpec, callbackID)

addCallback(getUserFcnCBM, eventname, callbackSpec, callbackID);

return;