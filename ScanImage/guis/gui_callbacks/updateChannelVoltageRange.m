function updateChannelVoltageRange(handle)
%% function updateChannelVoltageRange(handle)
% Callback function that handles update to voltage range value for a channel
%
%% NOTES
%   Being an INI-named callback allows this to be called during either a GUI control or CFG file loading event
%
%% CHANGES
%   VI010611A: Support voltage ranges of 0.2, 0.5, and 20V -- Vijay Iyer 1/6/11
%
%% CREDITS
%   Created 1/11/09, by Vijay Iyer
%% ******************************************************************
global state gh

inGlobalName = get(handle,'Tag');
chanNum = str2num(inGlobalName(end));

val = get(handle,'Value'); %VI110209A
switch val %VI110209A
    case {1,8} %VI110209A
        voltageRange = 0.2;
    case {2,9} %VI010611A
        voltageRange = 0.5;
    case {3,10} %VI110209A
        voltageRange = 1;        
    case {4,11} %VI110209A
        voltageRange = 2;
    case {5,12} %VI110209A
        voltageRange = 5;             
    case {6,13} %VI110209A
        voltageRange = 10;
    case {7,14} %VI010611A
        voltageRange = 20;
end
invert = double(val > 7); %VI010611A %VI110209A

%eval(['state.acq.inputVoltageRange' num2str(chanNum) '=' num2str(voltageRange) ';']); %VI110209A
chanStr = num2str(chanNum);
state.acq.(['inputVoltageRange' chanStr]) = voltageRange;
state.acq.(['inputVoltageInvert' chanStr]) = invert;



        
        