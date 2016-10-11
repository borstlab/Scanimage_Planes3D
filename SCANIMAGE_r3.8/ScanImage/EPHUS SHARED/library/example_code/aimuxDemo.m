% SYNTAX
%  aimuxDemo
%  aimuxDemo(boardId, channelId) - Specify a specific hardware channel to use.
%
% NOTES
%  This is a small demonstration of the basic functionality and usage of the @aimux object.
%  In order for this to work, the @aimux class, @daqmanager class, and library are necessary.
%
% USAGE
%  This will:
%   - add a channel to the general daqmanager instance
%   - connect a new aimux instance to the daqmanager
%   - bind a listener function to the aimux
%   - create and bind a scope to the aimux
%   - configure specific properties for the new channel on the daqmanager
%   - start an acquisition
%   - clean up after itself
%
%  Hardware and naming conflicts are possible. It is best to not run this
%  simultaneously with other data acquisition software.
%
% Created - Tim O'Connor 4/5/05
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function aimuxDemo(varargin)

if isempty(varargin)
    boardId = 1;  
    channelId = 1;
else
    boardId = varargin{1};
    channelId = varargin{2};
end
channelName = ['TestIn' num2str(boardId) num2str(channelId)];

%Get a daqmanager instance.
dm = getDaqmanager;

%Create an input channel: nameInputChannel(@daqmanager, boardID, channelID, channelName)
nameInputChannel(dm, boardId, channelId, channelName);

%Enable the channel.
enableChannel(dm, channelName);

%Create an analog input multiplexer, attached to this @daqmanager.
aim = aimux(dm);

%Make a listener function definition, including a custom defined argument.
listenerFcn = {@exampleListenerFcn, channelName};

%Register to receive data from the multiplexer: bind(AIMUX, channelName, listenerFunction, listenerFunctionID)
bind(aim, channelName, listenerFcn, 'aimuxDemo');
sc = scopeObject;
bindAimuxChannel(sc, channelName, aim);

sampleRate = 5000;

%Set acquisition specific properties, see Matlab's analoginput documentation for details.
%NOTES: The 'SamplesAcquiredFcnCount' property defines how often the aimux object will distribute data.
%       The 'StopFcn' will clean up, removing the test channel(s) from the @daqmanager.
setAIProperty(dm, channelName, 'SamplesPerTrigger', sampleRate * 100, 'TriggerType', 'Immediate', ...
    'SamplesAcquiredFcnCount', 1024, 'StopFcn', {@exampleStopFcn, dm, channelName}, ...
    'StartFcn', @exampleStartFcn, 'TriggerFcn', @exampleTriggerFcn, 'SampleRate', sampleRate);

%Start the acquisition.
startChannel(dm, channelName);

return;

%-----------------------------------------------------------------------------
function exampleListenerFcn(channelName, data, ai, strct, varargin)

%Do something application specific with the data.
fprintf(1, 'aimuxDemo: %s - received %s samples on channel ''%s''.\n', datestr(now), num2str(length(data)), channelName);

return;

%-----------------------------------------------------------------------------
function exampleStartFcn(varargin)

fprintf(1, 'aimuxDemo: %s - Acquisition started.\n', datestr(now));

return;

%-----------------------------------------------------------------------------
function exampleTriggerFcn(varargin)

fprintf(1, 'aimuxDemo: %s - Acquisition triggered.\n', datestr(now));

return;

%-----------------------------------------------------------------------------
function exampleStopFcn(ai, eventdata, dm, channelName)

fprintf(1, 'aimuxDemo: %s - Acquisition stopped.\n', datestr(now));

%This may generate a warning, because we're trying to stop an object from within it's stop callback.
%The @daqmanager will resolve the problem, but will issue a warning, because it's bad practice to do things
%this way.
denameInputChannel(dm, channelName);

return;