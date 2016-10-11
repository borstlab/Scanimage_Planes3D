%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  channelData ... = getDaqData(OBJ, channelName, <pointsToGet>, ...)
%%
%%  Returns a cell array of the 'data' field for each channel specified. The cell array
%%  is in the same order as the channelNames.
%%
%%  In the case of an input channel, calling this function causes all input channels on the same
%%  board to have their (hardware/Matlab) buffers read. The buffers are appended onto all the channels' 
%%  buffers in the daqmanager object. The daqmanager's buffer for the channel being read is 
%%  emptied and the data in it is returned.
%%
%%  The pointsToGet field is optional (with each channel specified). If pointsToGet is larger than the available
%%  data, all the data will be returned. Otherwise, the buffer, from index 1 to pointsToGet (inclusive) is returned.
%%  Setting pointsToGet less than 0 is the same as not specifying the option.
%%
%%  For other output options, unique to analog inputs, use getDaqDataSpecial.
%%
%%  NOTE
%%   The number of samples retrieved must be the same for all channels requested in a single call to this function.
%%
%%  Created - Tim O'Connor 11/7/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%           1/27/04 Tim O'Connor TO12704d: Add support for input channels.
%%           1/27/04 Tim O'Connor TO12704e: Don't pad this data, return a cell array instead of a numeric array.
%%           11/11/04 Tim O'Connor TO111104a: Track when put/get calls for data, for debugging purposes.
%%           8/29/05 Tim O'Connor TO082905B: Fix problems when requesting a specific number of samples.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function varargout = getDaqData(dm, varargin)
global gdm;

if length(varargin) == 1 & strcmpi(class(varargin{1}), 'cell')
    varargin = varargin{1};
end

data = [];
pointsToGet = -1;

%TO082905B - Switch to a while loop, since the argument indexing may change midstream.
% for i = 1 : length(varargin)
i = 1;
while i <= length(varargin)
    indexOffset = 0;%TO082905B - Use this to augment i, at the end of the loop, if necessary.
    index = getChannelIndex(dm, varargin{i});
    
    if index < 1
        errMsg = sprintf('No channel found with name ''%s''.', varargin{i});
        error(errMsg);
    end
    
    %Only get some of the data...
    if i < length(varargin) & isnumeric(varargin{i + 1}) %TO082905B
        indexOffset = 1;
        pointsToGet = varargin{i + 1};
    end

    %Pick up the channel's data.
    if gdm(dm.ptr).channels(index).ioFlag == 0
        if length(gdm(dm.ptr).channels(index).data) == 0
            %No data.
            chData = [];
        elseif pointsToGet < 0 | length(gdm(dm.ptr).channels(index).data) < pointsToGet
            %Just return the buffer.
            chData = gdm(dm.ptr).channels(index).data;
        else
            %Return a subset of the buffer.
            chData = gdm(dm.ptr).channels(index).data(1 : pointsToGet);
        end
    elseif gdm(dm.ptr).channels(index).ioFlag == 1
        %Fill buffer(s).
        getInputData(dm, varargin{i});
        
        if pointsToGet == 0
            %Nothing to get.
            chData = [];
            
        elseif pointsToGet < 0 | length(gdm(dm.ptr).channels(index).data) < pointsToGet
            %TO082905B
            if length(gdm(dm.ptr).channels(index).data) < pointsToGet
                error('Requested number of samples (%s) is greater than available number of samples (%s) for channel ''%s''', ...
                    num2str(pointsToGet), num2str(length(gdm(dm.ptr).channels(index).data)), varargin{i});
            end
            
            %Pick up the correct buffer, to be returned.
            chData = gdm(dm.ptr).channels(index).data;
        
            %Now, clear that buffer.
            gdm(dm.ptr).channels(index).data = [];
        else
            %Pick up the correct buffer, to be returned, grab only some of the data.
            chData = gdm(dm.ptr).channels(index).data(1 : pointsToGet);
            
            %Now, clear part of that buffer.
            gdm(dm.ptr).channels(index).data = gdm(dm.ptr).channels(index).data(pointsToGet + 1 : length(gdm(dm.ptr).channels(index).data));
        end
    elseif gdm(dm.ptr).channels(index).ioFlag == 2
        %What should we do with digitalio types?
    else
        %Uh-oh!!!
        error(sprintf('ioFlag field for daqmanager:%s channel:%s is corrupted: %s', num2str(dm.ptr), num2str(index), num2str(gdm(dm.ptr).channels(index).ioFlag)));
    end

    %TO111104a - Track when put/get calls for data, for debugging purposes. -- Tim O'Connor 11/11/04
    %Should this come before or after all the other work (and possible errors)?
    cr = sprintf('\n');
    lastSetEventString = sprintf( '''%s'' last getDaqData event - %s', gdm(dm.ptr).channels(index).name, getStackTraceString);
    gdm(dm.ptr).channels(index).lastPutEventString = strrep(lastSetEventString, cr, [cr '   ']);
    
    %TO12704e - Removed padding and array concatenation code.
    %Fill the cell array values.
    varargout{i} = chData;
    
    i = i + 1 + indexOffset;%TO082905B
end

return;