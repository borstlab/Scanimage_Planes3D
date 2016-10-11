% @daqmanager/putDaqDataChunked - Queue new data to set of channels without stopping their ongoing acquisition. 
%
% SYNTAX
%  putDaqDataChunked(dm, channelName, data)
%    dm - The @daqmanager instance.
%    channelNames - A cell array of channel names to queue data out to.
%    data - A matrix of data to be put out, each column represents a channel.
%
% USAGE
%
% NOTES
%  No properties are changed, no events are processed. For any other channels
%  shared on analogoutput objects containing named channels, data
%  corresponding to the "DefaultChannelValue" property of those channels
%  are output.
%
% CHANGES
%
% Created 8/29/06 Vijay Iyer
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function putDaqDataChunked(this, channelNames, data)

%Determine each of the AOs associated with list of channel Names
aos = [];
aochannels = {};
for i=1:length(channelNames)
    ao = getAO(this, channelNames{i});
    if ~ismember(ao,aos)
        aos = [aos ao];
        aochannels = {aochannels{:},[]}; %add one new element to cell array
    end
    aoindex = find(ao==aos);
    aochannels{aoindex} = [aochannels{aoindex} i]; %Store cell array of channels for each AO object
end
if isempty(ao)
    error('No AOs found for specified channels');
end

% fprintf(1, '%s - @daqmanager/putDaqDataChunked: Restarting object...\n', datestr(now));

%Put data to each of the AO objects
datalength = size(data,1);

for i=1:length(aos)
    chans = aos(i).Channel.Index;
    
    outdata = zeros(datalength,length(chans));
    for j=1:length(chans)
        dataindex = find(chans{j}==aochannels{i});
        if isempty(dataindex)
            outdata(:,j) = ones(datalength,1)*aos(i).Channel(chans{j}).DefaultChannelValue;
        else
            outdata(:,j) = data(:,dataindex);
        end
    end
    
    putdata(aos(i),outdata);
end     
 

return