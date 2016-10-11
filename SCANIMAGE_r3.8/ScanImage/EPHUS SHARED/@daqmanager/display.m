function display(dm)
% DISPLAY The display method for the daqmanager class.
%
% display(dm)
%  
% Created - Tim O'Connor 11/13/03
%
% Changed:
%         1/26/04 Tim O'Connor TO12604b: Use "pointers". See daqmanager.m for details.
%         1/29/04 Tim O'Connor TO12904a: Put an upper bound on string lengths.
%
% Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%
% See also DAQMANAGER/DAQMANAGER

global gdm;

dmString = sprintf('Daqmanager v0.1\nObjectPointer: %s\nChannels:\n', num2str(dm.ptr));

dmArr{1, 1} = '   Name';
dmArr{1, 2} = '   BoardId';
dmArr{1, 3} = '   ChannelId';
dmArr{1, 4} = '     State    ';
dmArr{1, 5} = '   Samples';
dmArr{1, 6} = '   Sample Rate\n';

dmArr{2, 1} = '   ----';
dmArr{2, 2} = '   -------';
dmArr{2, 3} = '   ---------';
dmArr{2, 4} = '     -----    ';
dmArr{2, 5} = '   -------';
dmArr{2, 6} = '   -----------\n';

%Collect lengths.
lengths = [];
for i = 1 : size(dmArr, 1)
    for j = 1 : size(dmArr, 2)
        lengths(i, j) = length(dmArr{i, j});
    end
end

%Assume that the biggest length differential will not exceed 40 characters.
spaces = char(ones(1, 40) * ' ');

%Construct display data, keep a running assesment of lengths.
for i = 1 : length(gdm(dm.ptr).channels)
    
    if ~isempty(gdm(dm.ptr).channels(i).name)
        
        %TO12904a Tim O'Connor 1/28/04 - Shorten any overly long strings.
        if length(gdm(dm.ptr).channels(i).name) <= 40
            dmArr{i + 2, 1} = strcat('   ', gdm(dm.ptr).channels(i).name);
        else
            dmArr{i + 2, 1} = strcat('   ', strcat(gdm(dm.ptr).channels(i).name(1 : 17), '...'));
        end
        lengths(i + 2, 1) = length(dmArr{i + 2, 1});
        
        dmArr{i + 2, 2} = strcat('   ', num2str(gdm(dm.ptr).channels(i).boardId));
        lengths(i + 2, 2) = length(dmArr{i + 2, 2});
        
        dmArr{i + 2, 3} = strcat('   ', num2str(gdm(dm.ptr).channels(i).channelId));
        lengths(i + 2, 3) = length(dmArr{i + 2, 3});
        
        if gdm(dm.ptr).channels(i).state == 0
            dmArr{i + 2, 4} = '    Disabled  ';
        elseif gdm(dm.ptr).channels(i).state == 1
            dmArr{i + 2, 4} = '     Enabled  ';
        elseif gdm(dm.ptr).channels(i).state == 2
            dmArr{i + 2, 4} = '     Started  ';
        else
            dmArr{i + 2, 4} = '       ???    ';
        end
        lengths(i + 2, 4) = length(dmArr{i + 2, 4});
        
        if gdm(dm.ptr).channels(i).ioFlag == 1
            %Update buffers, if necessary.
            getInputData(dm, gdm(dm.ptr).channels(i).name);
        end
        dmArr{i + 2, 5} = strcat('   ', num2str(length(gdm(dm.ptr).channels(i).data)));
        lengths(i + 2, 5) = length(dmArr{i + 2, 5});
        
        if gdm(dm.ptr).channels(i).ioFlag == 0
            dmArr{i + 2, 6} = strcat('   ', num2str(takeAOProperty(dm, gdm(dm.ptr).channels(i).name, 'SampleRate')), ' Hz\n');
        elseif gdm(dm.ptr).channels(i).ioFlag == 1
            dmArr{i + 2, 6} = strcat('   ', num2str(takeAIProperty(dm, gdm(dm.ptr).channels(i).name, 'SampleRate')), ' Hz\n');
        end
        lengths(i + 2, 6) = length(dmArr{i + 2, 6});
    end
end

%Build display string. Pad strings for justification, as needed.
for i = 1 : size(dmArr, 1)

    for j = 1 : size(dmArr, 2)

        len = max(lengths(:, j));

        if lengths(i, j) == len
            dmString = sprintf('%s%s', dmString, dmArr{i, j});
        else
            dmString = sprintf('%s%s%s', dmString, spaces(1 : len - lengths(i, j)), dmArr{i, j});
        end

    end
end

fprintf(1, dmString);

return;