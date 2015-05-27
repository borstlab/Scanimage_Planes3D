%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  getInputData(OBJ, channelName)
%%
%%  Check hardware buffers and refill daqmanager buffers, for
%%  analog input objects.
%%
%%  Was originally a subfunction of getDaqData.
%%
%%  Created - Tim O'Connor 11/29/04
%%
%%  Changed:
%%   TO022105b - Specify number of samples to retrieve, due to a "More samples requested than available." error . -- Tim O'Connor 2/21/05
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function getInputData(dm, channelName)
global gdm;

ai = getAI(dm, channelName);

numberOfSamples = get(ai, 'SamplesAvailable');
if (exist('ai') ~= 1) | isempty(ai) | isempty(ai.Channel) | (numberOfSamples < 1)
    return;%Nothing's running, so there's no buffer to move.
end

%Get the data for all channels.
data = getData(ai, numberOfSamples);

%Sort the data into the individual channel buffers.
for i = 1 : length(ai.Channel)
    index = getChannelIndex(dm, ai.Channel(i).ChannelName);
    gdm(dm.ptr).channels(index).data = cat(1, gdm(dm.ptr).channels(index).data, data(:, i));
end

return;