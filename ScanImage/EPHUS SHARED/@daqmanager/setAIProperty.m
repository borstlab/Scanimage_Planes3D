%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Call-through to set properties on analoginput objects.
%%
%%  PROPERTIES = setAIProperty(OBJ, boardId)
%%  PROPERTIES = setAIProperty(OBJ, 'channelName')
%%
%%  PROPERTIES = setAIProperty(OBJ, boardId, 'PROPERTY_NAME', 'PROPERTY_VALUE', ...)
%%  PROPERTIES = setAIProperty(OBJ, 'channelName', 'PROPERTY_NAME', 'PROPERTY_VALUE', ...)
%%
%%  Works like the standard 'set' function, except it takes a boardId as well as an object
%%  as the first arguments.
%%
%%  The first two forms of the function (2 args) call directly through to the analog input object.
%%  They do not reflect the state of the property table in the daqmanager. Therefore, they serve only
%%  as a list of settable properties, the values should be ignored.
%%
%%  Both forms of the function using boardId act directly on the analog output object. That is to say,
%%  they do not alter properties for named channels.
%%
%%  Created - Tim O'Connor 11/29/04
%%
%%  Changed:
%%    Tim O'Connor 4/21/05 TO042105A - Make set commands fail-fast by hitting the underlying object right away.
%%    Tim O'Connor 11/22/05 TO112205C - Implement Start/Stop/Trigger listeners using the @callbackManager.
%%                                      Warn of the overwrite implementation in @daqmanager/startChannel, suggest alternatives.
%%    Tim O'Connor 2/8/06 TO020806A - Allow compatibility with the allowMultistart property. See TO012706F.
%%    Tim O'Connor 2/27/06 TO022706D - Optimization(s). Try batching up the `ismember` calls.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function val = setAIProperty(dm, identifier, varargin)
global gdm;

%Check the args.
if mod(length(varargin), 2) ~= 0
    error('Wrong number of arguments.');
end

ai = getAI(dm, identifier);

if isempty(ai)
    if isnumeric(identifier)
        identifier = num2str(identifier);
    end
    
    error(sprintf('No analog output found with identifier: %s', identifier));
end

if nargin == 2
    val = set(ai);
% 
%     if ~isnumeric(identifier)
%         val = putAOProperty(dm, identifier);
%     end
else
    
    %TO112205C
    propNames = lower({varargin{1:2:end-1}});
    %TO022706D: Batch these up, it should be faster. -- Tim O'Connor 2/27/06
%     if ismember('startfcn', propNames) | ismember('stopfcn', propNames) | ismember('triggerfcn', propNames)
    if any(ismember({'startfcn', 'stopfcn', 'triggerfcn'}, propNames))
        error('Values set for the StartFcn, StopFcn, and TriggerFcn properties will get overwritten.\nUse setChannelStartListener, setChannelStopListener, setChannelTriggerListener instead.');
    end
    if isnumeric(identifier)
        %TO020806A
        if ~strcmpi(get(ai, 'Running'), 'On') & gdm(dm.ptr).allowMultistart
            set(ai, varargin{:});
        end
        val = set(ai);
    else
        %TO020806A
        if ~strcmpi(get(ai, 'Running'), 'On') & gdm(dm.ptr).allowMultistart
            set(ai, varargin{:});
        end
        val = putAIProperty(dm, identifier, varargin);
    end
end

return;