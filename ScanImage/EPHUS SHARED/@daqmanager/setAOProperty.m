%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Call-through to set properties on analogoutput objects.
%%
%%  PROPERTIES = setAOProperty(OBJ, boardId)
%%  PROPERTIES = setAOProperty(OBJ, 'channelName')
%%
%%  PROPERTIES = setAOProperty(OBJ, boardId, 'PROPERTY_NAME', 'PROPERTY_VALUE', ...)
%%  PROPERTIES = setAOProperty(OBJ, 'channelName', 'PROPERTY_NAME', 'PROPERTY_VALUE', ...)
%%
%%  Works like the standard 'set' function, except it takes a boardId as well as an object
%%  as the first arguments.
%%
%%  The first two forms of the function (2 args) call directly through to the analog output object.
%%  They do not reflect the state of the property table in the daqmanager. Therefore, they serve only
%%  as a list of settable properties, the values should be ignored.
%%
%%  Both forms of the function using boardId act directly on the analog output object. That is to say,
%%  they do not alter properties for named channels.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%           1/29/04 Tim O'Connor TO12904c: Use "pointers". See daqmanager.m for details.
%%    Tim O'Connor 4/21/05 TO042105A - Make set commands fail-fast by hitting the underlying object right away.
%%    Tim O'Connor 11/22/05 TO112205C - Implement Start/Stop/Trigger listeners using the @callbackManager.
%%                                      Warn of the overwrite implementation in @daqmanager/startChannel, suggest alternatives.
%%    Tim O'Connor 2/8/06 TO020806A - Allow compatibility with the allowMultistart property. See TO012706F.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function val = setAOProperty(dm, identifier, varargin)
global gdm;

%Check the args.
if mod(length(varargin), 2) ~= 0
    error('Wrong number of arguments.');
end

ao = getAO(dm, identifier);

if isempty(ao)
    if isnumeric(identifier)
        identifier = num2str(identifier);
    end

    error(sprintf('No analog output found with identifier: %s', identifier));
end

if nargin == 2
    val = set(ao);
% 
%     if ~isnumeric(identifier)
%         val = putAOProperty(dm, identifier);
%     end
else
    %TO112205C
    propNames = lower({varargin{1:2:end-1}});
    if ismember('startfcn', propNames) | ismember('stopfcn', propNames) | ismember('triggerfcn', propNames)
        error('Values set for the StartFcn, StopFcn, and TriggerFcn properties will get overwritten.\nUse setChannelStartListener, setChannelStopListener, setChannelTriggerListener instead.');
    end
    if isnumeric(identifier)
        %TO020806A
        if ~strcmpi(get(ao, 'Running'), 'On') & gdm(dm.ptr).allowMultistart
            set(ao, varargin{:});
        end
        val = set(ao);
    else
        %TO020806A
        if ~strcmpi(get(ao, 'Running'), 'On') & gdm(dm.ptr).allowMultistart
            set(ao, varargin{:});%TO042105A - Make this fail-fast.
        end
        val = putAOProperty(dm, identifier, varargin);
    end
end

return;