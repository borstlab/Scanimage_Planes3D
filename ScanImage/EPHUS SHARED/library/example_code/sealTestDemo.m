% SYNTAX
%  sealTestDemo
%
% NOTES
%  This is a small demonstration of the basic functionality of how to execute a
%  seal test, using the @aimux, @aomux, @signal, @daqmanager, @amplifier, and 
%  @axopatch_200b classes.
%
% USAGE
%  This will:
%
%  Hardware and naming conflicts are possible. It is best to not run this
%  simultaneously with other data acquisition software.
%
% Created - Tim O'Connor 4/8/05
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function sealTestDemo
global dm amp aim aom scaledOutputChannelName inputChannelNames channelNames sc dio;

%Set up some configuration type variables.
sampleRate = 5000;%Hz
pulseDuration = .001;%Seconds
repetitions = 1;

%Get a @daqmanager instance.
dm = getDaqmanager;

%Set up the VCom channel.
nameOutputChannel(dm, 1, 0, 'V-Com');
enableChannel(dm, 'V-Com');

%Create and configure a patch clamp.
amp = axopatch_200B;
set(amp, 'name', 'sealTestDemo', 'gain_daq_board_id', 2, 'mode_daq_board_id', 2, 'v_hold_daq_board_id', 2, 'gain_channel', 0, ...
    'mode_channel', 2, 'v_hold_channel', 1, 'commandInputChannel', 'V-Com', 'scaledOutputBoardID', 1, 'scaledOutputChannelID', 1);

%Create an analog input multiplexer and attach the amplifier to it.
aim = aimux(dm);
bindToAIMUX(amp, aim);

%Create an analog output multiplexer, then a signal, and bind the signal to the multiplexer.
aom = aomux(dm);
set(aom, 'outputTime', 2 * pulseDuration);
s = signalobject;
squarePulse(s, .01, 0, 0.25 * pulseDuration, 0.5 * pulseDuration);%Make it a square pulse.
bind(aom, 'V-Com', s);

scaledOutputChannelName = getScaledOutputChannelName(amp);

%Set up a scope display.
sc = scopeObject;
set(sc, 'yUnitsPerDiv', 2 * pulseDuration / 11);
bindAimuxChannel(sc, getScaledOutputChannelName(amp), aim);%Attach the scope to the inbound signal.

%Gather up a list of channel names.
channelNames = cat(2, {'V-Com'}, getInputChannelNames(amp));

%Set the AO and AI properties, as needed.
setAOProperty(dm, 'V-Com', 'RepeatOutput', 1000);
inputChannelNames = getInputChannelNames(amp);

for i = 1 : length(inputChannelNames)
%     setAIProperty(dm, inputChannelNames{i}, 'SamplesPerTrigger', sampleRate * 100, 'TriggerType', 'HwDigital', ...
%         'SamplesAcquiredFcnCount', round(sampleRate * pulseDuration), 'StopFcn', {@sealTestStopFcn, amp}, ...
%         'StartFcn', @sealTestStartFcn, 'TriggerFcn', @sealTestTriggerFcn, 'SampleRate', sampleRate);
    setAIProperty(dm, inputChannelNames{i}, 'SamplesPerTrigger', sampleRate * 100, 'TriggerType', 'HwDigital', ...
        'SamplesAcquiredFcnCount', round(sampleRate * pulseDuration), 'SampleRate', sampleRate);
end

%Configure a digital trigger.
dio = digitalio('nidaq', 1);
addline(dio, 0, 'out');
putvalue(dio, 0);

channelNames = {'sealTestDemo_scaledOutput'};%The other board is having issues, so leave it out for now. 4/15/05

%Run through this a few times.
for i = 1 : repetitions
    fprintf(1, 'sealTestDemo: Starting...\n');
% channelNames
% return;
    startChannel(dm, channelNames);

    fprintf(1, 'Triggering...\n');    
    putvalue(dio, 1);
    putvalue(dio, 0);
    putvalue(dio, 1);

    counter = 0;
    ao = getAO(dm, 'V-Com');
    while strcmpi(get(ao, 'Running'), 'On') & counter < 40
        fprintf('sealTestDemo: Waiting...\n');
        pause(1);
        counter = counter + 1;
    end
    if counter > 40
        fprintf(1, 'sealTestDemo: Breaking...\n');
        stop(daqfind);
        return;
    end
    
    stopChannel(dm, channelNames);
end

return;

%-------------------------------------------------------
%Clean up and get ready for the next round of acquisitions.
function sealTestStartFcn(ai, eventdata, varargin)

fprintf(1, '%s - sealTestDemo/StartFcn: ''%s''\n', datestr(now), get(ai, 'name'));

return;

%-------------------------------------------------------
%Clean up and get ready for the next round of acquisitions.
function sealTestTriggerFcn(ai, eventdata, varargin)

fprintf(1, '%s - sealTestDemo/TriggerFcn: ''%s''\n', datestr(now), get(ai, 'name'));

return;

%-------------------------------------------------------
%Clean up and get ready for the next round of acquisitions.
function sealTestStopFcn(ai, eventdata, amp)

fprintf(1, '%s - sealTestDemo/StopFcn: ''%s''\n', datestr(now), get(ai, 'name'));
try
    stopChannel(getDaqmanager, cat(1, getOutputChannelNames(amp), getInputChannelNames(amp)));
catch
    warning(lasterr);
end

return;