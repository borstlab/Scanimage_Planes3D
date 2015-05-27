% @daqmanager/plotData - Plot the buffered data for a selected analogoutput object.
%
% SYNTAX
%  plotData(this, ao, data)
%   this - The daqmanager instance.
%   ao - The analogoutput object.
%   data - The data buffer.
%
% USAGE
%
% NOTES
%  Moved from a subfunction in startChannel to a private function in the flass.
%
% CHANGES
%  TO080606A: See @daqmanager/startChannel for changes prior to refactoring. -- Tim O'Connor 8/6/06
%
% Created 8/6/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function plotData(dm, ao, data)
global gdm;

if ~gdm(dm.ptr).displayHardwareBuffer
    return;
end

info = daqhwinfo(ao);

%Put together the X coordinates and an associated legend.
domain = 1000 .* (1:length(data)) ./ get(ao, 'SampleRate');
legendString = '';
for i = 1 : length(ao.Channel)
    domain(:, i) = domain(:, 1);
    %Tim O'Connor 7/21/04 TO072104a: Print the actual hardware channel # instead of the index.
    legendString{i} = sprintf('Channel-%s (''%s'')', num2str(ao.Channel(i).HwChannel), ao.Channel(i).ChannelName);
end

%Make a new figure, if necessary.
% if isempty(gdm(dm.ptr).figures.hardwareBuffers)
%     gdm(dm.ptr).figures.hardwareBuffers(info.ID) = figure('NumberTitle', 'Off', 'Name', '@daqmanager/startchannel');
% elseif length(gdm(dm.ptr).figures.hardwareBuffers) < info.ID
%     gdm(dm.ptr).figures.hardwareBuffers(info.ID) = figure('NumberTitle', 'Off', 'Name', '@daqmanager/startchannel');
% elseif gdm(dm.ptr).figures.hardwareBuffers(info.ID) == 0 | ~ishandle(gdm(dm.ptr).figures.hardwareBuffers(info.ID))
%     gdm(dm.ptr).figures.hardwareBuffers(info.ID) = figure('NumberTitle', 'Off', 'Name', '@daqmanager/startchannel');
% end
figure('NumberTitle', 'Off', 'Name', '@daqmanager/startchannel')

%It needs multiple axes. One for the data and one for the stack trace.
h = axes('Position', [0 0 1 1], 'Visible', 'Off');
axes('Position', [.1 .3 .85 .6]);

%Plot the data.
plot(domain, data, '-o', 'MarkerSize', 5);

%Make it purty.
title(sprintf('DaqManager: Data being written to NIDAQ buffer.\nBoardID: %s', num2str(info.ID)));
xlabel('Time [ms]');
ylabel('Voltage [V]');

%Add a legend, if nothing went wrong.
if ~isempty(legendString)
    legend(legendString{:});
end

%Tim O'Connor TO072104b: Factored out the stack trace construction.
%Add the stack trace to the plot.
set(gcf, 'CurrentAxes', h);
strace = texSafe(getStackTraceString(2));%TO021605a
if isempty(strace)
    strace = sprintf('No stack trace information available.\n  Most likely called from the Matlab command line.');
end
text(.05, .1, strace, 'FontSize', 8);

return;