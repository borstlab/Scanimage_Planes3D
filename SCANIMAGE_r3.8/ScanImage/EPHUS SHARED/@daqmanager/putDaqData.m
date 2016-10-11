%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  OBJ = putDaqData(OBJ, channels{}, data{})
%%
%%  Created - Tim O'Connor 11/7/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%           11/11/04 Tim O'Connor TO111104a: Track when put/get calls for data, for debugging purposes.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dm = putDaqData(dm, channels, data)
global gdm;
% fprintf(1, '@daqmanager/putDaqData(this, %s, %s)\n%s\n', channels, num2str(length(data)), getStackTraceString);
%Transpose, if ncessary.
if size(data, 1) > size(channels, 1) & size(data, 2) > size(channels, 1)
    error('Too many columns of data for the number of channels listed.');
else
    if strcmpi(class(data), 'cell')
        for i = 1 : length(data)
            if size(data{i}, 2) > size(data{i}, 1)
                data{i} = data{i}';
            end
        end
    elseif size(data, 2) > size(data, 1)
        data = data';
    end
end

if ~strcmp(class(channels), 'cell')
    channels = cellstr(channels);
end

if ~strcmp(class(data), 'cell')
    cdata = {};
    columns = size(data, 2);
    
    for i = 1 : columns
        cdata{i} = data(:, i);
    end
    
    data = cdata;
end

%Prescan for errors...
indices = [];
for i = 1 : length(channels)
    indices(i) = getChannelIndex(dm, channels{i});
    if indices(i) < 1
        error('Failed to find channel ''%s''. No data has been written to any channel.', channels{i});
    end
end

cr = sprintf('\n');
%Put the data out to the channels, sequentially.
for i = 1 : length(channels)
% fprintf(1, '@daqmanager/putDaqData: Putting %s samples for channel ''%s''...\n', mat2str(size(data{i})), gdm(dm.ptr).channels(indices(i)).name);
   gdm(dm.ptr).channels(indices(i)).data = data{i};
   gdm(dm.ptr).channels(indices(i)).lastData = gdm(dm.ptr).channels(indices(i)).data(length(gdm(dm.ptr).channels(indices(i)).data));
   
   %TO111104a - Track when put/get calls for data, for debugging purposes. -- Tim O'Connor 11/11/04
   lastSetEventString = sprintf( '''%s'' last putDaqData event - %s', gdm(dm.ptr).channels(indices(i)).name, getStackTraceString);
   gdm(dm.ptr).channels(indices(i)).lastPutEventString = strrep(lastSetEventString, cr, [cr '   ']);
end

return;