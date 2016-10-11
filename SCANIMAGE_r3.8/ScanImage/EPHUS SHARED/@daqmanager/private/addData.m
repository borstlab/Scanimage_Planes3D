% @daqmanager/addData - Put output data on the correct analogoutput objects.
%
% SYNTAX
%  addData(this, name, ...)
%  addData(this, nameArray)
%   name - A valid channel name.
%   nameArray - A cell array of valid channel names.
%
% USAGE
%  Put output data based on a list of channel names (the list may be a cell array).
%
% NOTES
%  Moved from a subfunction in startChannel to a private function in the flass.
%
% CHANGES
%  TO080606A: See @daqmanager/startChannel for changes prior to refactoring. -- Tim O'Connor 8/6/06
%
% Created 8/4/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function addData(dm, varargin)
global gdm;

if length(varargin) == 1 & strcmpi(class(varargin{1}), 'cell')
    varargin = varargin{1};
end

done = {};

%Iterate over all analog outputs and active channels.
for i = 1 : length(varargin)

    data = [];

    ao = getAO(dm, varargin{i});

    %If we've already done this board's object, skip to the next.
    if isempty(ao) | isempty(ao.Channel) | inList(ao, done)
        continue;
    end

    done{length(done) + 1} = ao;
    for j = 1 : length(ao.Channel)

        %Look up the channel.
        info = daqhwinfo(ao);
        boardId = getNumericSuffix(info.ID);%TO011706D - Take into account NIDAQmx labelling scheme. -- Tim O'Connor 1/17/06
        index = getChannelIndex(dm, boardId, ao.Channel(j).HwChannel, 0);%TO12704d: 0 == output
        if index > 0
            %Pick up the channel's data.
            chData = gdm(dm.ptr).channels(index).data;

            %Pad arrays as needed.
            if isempty(chData)
                %No data for this channel...
                chData = gdm(dm.ptr).channels(index).lastData;
                warnMsg = sprintf('Channel ''%s'' has no output data. The output is being padded...', gdm(dm.ptr).channels(index).name);
                warning(warnMsg);
            end

            % start TPMOD_1 1/8/04
            % cache the size of the rows in data because it may change
            rows_of_data = size(data, 1);

            % Put the column of data into the array.  This could change
            % size(data)
            data(1 : length(chData), j) = chData;%NOTE: At this point, the array is resized, and padded with zeros.

            % This if/else handles the 2 cases where the new data is
            % smaller than the existing data, or larger.
            if ~isempty(data) & rows_of_data > length(chData)

                %The data for this channel needs to be padded.
                data(length(chData) : rows_of_data, 1) = data(length(chData));

                warnMsg = sprintf('Padding output data for channel ''%s''.', gdm(dm.ptr).channels(index).name);
                warning(warnMsg);
                
            elseif j ~= 1 & length(chData) > rows_of_data

                %TO11904a - Instead of just saying 'Padding output data for multiple channels', it should list the channels.
                paddedChannelList = 'Padding output data for channel(s) ';

                for paddedChannels = 1 : j - 1

                    chDx = getChannelIndex(dm, boardId, ao.Channel(paddedChannels).HwChannel, 0);%TO12704d: 0 == output

                    if  paddedChannels < (j - 1)
                        paddedChannelList = sprintf('%s''%s'', ', paddedChannelList, gdm(dm.ptr).channels(chDx).name);
                    else
                        paddedChannelList = sprintf('%s''%s''', paddedChannelList, gdm(dm.ptr).channels(chDx).name);    
                    end

                end
                warning(paddedChannelList);

                %The other channels need to be padded.
                for all_other_channels = 1 : j
                    data(rows_of_data : length(chData), all_other_channels) = data(rows_of_data, all_other_channels);
                end

            end
            % end TPMOD_1 1/8/04

        end
    end

    %Added the ability to display data sent to the hardware buffer. --Tim O'Connor 3/31/04: TO033104b
    if ~isempty(ao.Channel) & gdm(dm.ptr).displayHardwareBuffer
        try
            plotData(dm, ao, data);
        catch
            warning(sprintf('Error trying to plot data from hardware data acquisition buffer: %s', lasterr));
        end
    end
    if ~isempty(ao.Channel) & gdm(dm.ptr).displayAOStatus
        try
            fprintf(1, '\n---------------------\n');
            get(ao);
            fprintf(1, '\n---------------------  %s\n---------------------\n\n', getStackTraceString(1));
        catch
            warning(sprintf('Error trying to plot data from hardware data acquisition buffer: %s', lasterr));
        end
    end
    
    %Put the data onto this analog output.
    if ~isempty(ao) & ~isempty(ao.Channel)
        putdata(ao, data);
    end

    %Update the object.
    info = daqhwinfo(ao);
    gdm(dm.ptr).aos{boardId} = ao;

    %Clear the data array.
    data = [];
end

return;