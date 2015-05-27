function info = getHardwareInfo(dm)
% GETHARDWAREINFO Returns a structure that concisely describes the available hardware.
%
% getHardwareInfo(dm)
%
% The structure has the following fields:
%  out - an array of all the daqhwinfo for output objects.
%  in - an array of all the daqhwinfo for input objects.
%  channels.in - an array of structures containing name, boardId, and channelId.
%  channels.out - an array of structures containing name, boardId, and channelId.
%  
% Created - Tim O'Connor 5/26/04
%
% Changed:
%
% Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2004
%
% See also DAQMANAGER/DAQMANAGER
global gdm;

for i = 1 : length(gdm(dm.ptr).aos)
    info.out(i) = daqhwinfo(gdm(dm.ptr).aos(i));
end

for i = 1 : length(gdm(dm.ptr).ais)
    info.in(i) = daqhwinfo(gdm(dm.ptr).ais(i));
end

info.channels.in = [];
info.channels.out = [];
for i = 1 : length(gdm(dm.ptr).channels)
    if gdm(dm.ptr).channels(i).ioFlag
        info.channels.in(length(info.channels.in) + 1).name = gdm(dm.ptr).channels(i).name;
        info.channels.in(length(info.channels.in) + 1).boardId = gdm(dm.ptr).channels(i).boardId;
        info.channels.in(length(info.channels.in) + 1).channelId = gdm(dm.ptr).channels(i).channelId;
    else
        info.channels.out(length(info.channels.out) + 1).name = gdm(dm.ptr).channels(i).name;
        info.channels.out(length(info.channels.out) + 1).boardId = gdm(dm.ptr).channels(i).boardId;
        info.channels.out(length(info.channels.out) + 1).channelId = gdm(dm.ptr).channels(i).channelId;
    end
end

return;