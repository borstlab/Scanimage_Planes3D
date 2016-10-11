% AIMUX/aimuxSamplesAcquiredFcnDispatcher - This function is set, by the @AIMUX object, as the SamplesAcquiredFcn for
%                                           bound analog input channels.
%
% SYNTAX
%   aimuxSamplesAcquiredFcnDispatcher(ai, strct, varargin)
%     ai - The analoginput object
%     strct - The struct passed in from the SamplesAcquiredFcn call.
%     varargin - Anything else possibly passed in by the callback.
%
% NOTE
%  There is no reason for this to be called directly. The only use for this function is when the
%  @AIMUX object sets it as a callback.
%
% CHANGES
%  TO062405F: Check to see if the ai is running/valid, before executing callbacks. -- Tim O'Connor 6/24/05
%  TO123005I: Trap errors generated inside the AIMUX/aimuxSamplesAcquiredFcn function. -- Tim O'Connor 12/30/05
%
% Created 11/18/04 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
function aimuxSamplesAcquiredFcnDispatcher(ai, strct, varargin)

%TO062405F: It might be best to just see that there are channels, and not worry about
%           if the device is running (to allow data to be flushed after a stop).
if isempty(ai.Channel)
    %Erroneous call, may be too annoying and unavoidable to issue an error/warning for.
% fprintf(1, 'aimuxSamplesAcquiredFcnDispatcher: no channels for ''%s''\n', get(ai, 'Tag'));
    return;
end

aim = [];

if ~isempty(varargin)
    %Look for a passed in argument.
    if strcmpi(class(varargin{1}), 'aimux')
        aim = varargin{1};
        if length(varargin) > 1
            varargin = varargin{2:end};
        end
    end

    %Look in the user data.
    if isempty(aim)
        udata = get(ai, 'UserData');
        if isempty(udata)
            error('No userdata attached to the analoginput object. It is not correctly configured for use with aimuxSamplesAcquiredFcnDispatcher.');
        end
        if ~isfield(udata, 'aimux')
            error('No aimux to the analoginput object. It is not correctly configured for use with aimuxSamplesAcquiredFcnDispatcher.');
        end
        
        aim = udata.aimux;
    end
end

%TO123005I
% try
    aimuxSamplesAcquiredFcn(aim, ai, strct, varargin);
% catch
%     warning('%s aimuxSamplesAcquiredFcnDispatcher - Failed to properly execute .m: %s', datestr(now), lasterr);
% end

return;