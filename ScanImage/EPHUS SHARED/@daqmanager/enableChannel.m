function dm = enableChannel(dm, varargin)
% ENABLECHANNEL - Activate a channel for sending/recieving data.
%
% A warning will be generated if the channel is already enabled.
%
% Returns the number of channels that have been enabled.
%
% count = enableChannel(OBJ, 'name', ...)
%
% Created - Tim O'Connor 11/11/03
%
% Changed:
%         1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%         6/14/06 Vijay Iyer VI061406a: "1" used to be "i"
%
% Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%
% See also DAQMANAGER/DISABLECHANNEL
global gdm;

count = 0;

if nargin == 2
    name = varargin{1};
else
    for i = 1 : length(varargin) %VI061406a
        enableChannel(dm, varargin{i});
    end
    
    return;
end

index = getChannelIndex(dm, varargin{1});
if index < 1
    errMsg = sprintf('Could not find channel ''%s''.', varargin{1});
    error(errMsg);
end

if gdm(dm.ptr).channels(index).state == 1
    warnmsg = sprintf('Channel %s is already enabled.', name);
    warning(warnmsg);
elseif gdm(dm.ptr).channels(index).state == 2
    warnmsg = sprintf('Channel %s is already started.', name);
    warning(warnmsg);
else
    %Toggle the flag.
    gdm(dm.ptr).channels(index).state = 1;
    
    count = count + 1;
end

return;