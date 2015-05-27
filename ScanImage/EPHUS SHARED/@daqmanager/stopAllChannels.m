% @daqmanager/stopAllChannels - Stop all the running channels.
%
% SYNTAX
%  stopAllChannels(DAQMANAGER)
%
% USAGE
%
% NOTES
%
% CHANGES
%
% Created 4/7/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function stopAllChannels(this)
global gdm;

warnMsg = '';

try
    %Stop the output boards.
    for i = 1 : length(gdm(this.ptr).aos)
        if ~isempty(gdm(this.ptr).aos{i})
            stop(gdm(this.ptr).aos{i});
        end
    end

    %Mark them as stopped.
    for i = 1 : length(gdm(this.ptr).channels)
        if gdm(this.ptr).channels(i).ioFlag == 0 & gdm(this.ptr).channels(i).state == 2
            gdm(this.ptr).channels(i).state = 1;
        end
    end
catch
   warnMsg = addToWarnMsg(warnMsg, sprintf('Error stopping analog output objects: %s', lasterr));
end

try
    %Stop the input boards.
    for i = 1 : length(gdm(this.ptr).ais)
        if ~isempty(gdm(this.ptr).ais{i})
            stop(gdm(this.ptr).ais{i});
        end
    end
    %Mark them as stopped.
    for i = 1 : length(gdm(this.ptr).channels)
        if gdm(this.ptr).channels(i).ioFlag == 1 & gdm(this.ptr).channels(i).state == 2
            gdm(this.ptr).channels(i).state = 1;
        end
    end
catch
  warnMsg = addToWarnMsg(warnMsg, sprintf('Error stopping analoginput objects: %s', lasterr));
end

try
    deleteChannels(this, gdm(this.ptr).aos);
catch
    warnMsg = addToWarnMsg(warnMsg, sprintf('Error deleting analogoutput channels: %s', lasterr));
end

try
    deleteChannels(this, gdm(this.ptr).ais);
catch
    warnMsg = addToWarnMsg(warnMsg, sprintf('Error deleting analoginput channels: %s', lasterr));
end

if ~isempty(warnMsg)
    warning(warnMsg);
end

return;

%----------------------------------------------------------------------------
function warnMsg = addToWarnMsg(warnMsg, newMsg)

separator = '';
if ~isempty(warnMsg)
    separator = sprintf('\n         ');
end
warnMsg = sprintf('%s%s%s', warnMsg, separator, newMsg);

return