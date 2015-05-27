%  daqmanager/flushAllInputChannels(dm, channelName) - Calls any waiting samplesAcquireFcn for a given channel, to flush all data.
%
% USAGE
%  flushAllInputChannels(dm)
%    dm - A @daqmanager instance
%
% NOTES
%
% CHANGES
%
% Created 12/30/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function flushAllInputChannels(this, samples)
global gdm;

channelNames = {};
for i = 1 : length(gdm(this.ptr).channels)
    if gdm(this.ptr).channels(i).ioFlag == 1
        % drawnow;%This will let interrupting calls get executed, such as other SamplesAcquiredFcn calls.
        channelNames{length(channelNames) + 1} = gdm(this.ptr).channels(i).name;
    end
end

if ~isempty(channelNames)
    flushInputChannel(this, channelNames);%gdm(this.ptr).channels(i).name);
end

return;