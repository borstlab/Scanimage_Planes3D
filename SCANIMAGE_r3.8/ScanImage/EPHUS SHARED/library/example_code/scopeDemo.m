% SYNTAX
%  scopeDemo
%
% NOTES
%  This is a small demonstration of the basic functionality of how to use the @aimux, @daqmanager,
%  and @scopeobject classes.
%
% USAGE
%
%  Hardware and naming conflicts are possible. It is best to not run this
%  simultaneously with other data acquisition software.
%
% Created - Tim O'Connor 4/8/05
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function scopeDemo
global dm amp aim sc scg dio;

%Set up some configuration type variables.
sampleRate = 5000;%Hz
traceDuration = 5;%seconds
updateRate = 10;%Hz

%Get a @daqmanager instance.
dm = getDaqmanager;

%Set up the input channel.
nameInputChannel(dm, 1, 0, 'scopeDemoInput');
enableChannel(dm, 'scopeDemoInput');

%Create an analog input multiplexer.
aim = aimux(dm);
% bind(aim, 'scopeDemoInput', @scopeDemoSamplesAcquiredFcn, 'scopeDemoSamplesAcquiredFcn');%Comment this out to remove all profiling activities.

%Reset this counter, used to see if things are keeping up with the desired refresh rate.
global scopeDemoSamplesAcquiredFcnStrct;
scopeDemoSamplesAcquiredFcnStrct.samplesAcquired = 0;

%Set up a scope display.
scg = program('scopeDemo', 'scopeDemo', 'scopeGui');
openprogram(progmanager, scg);
sc = scg_getScope (scg);
% set(sc, ...);%Set specific scope properties here, if you like.
bindAimuxChannel(sc, 'scopeDemoInput', aim);%Attach the scope to the inbound signal.

setAIProperty(dm, 'scopeDemoInput', 'SamplesPerTrigger', sampleRate * traceDuration, 'TriggerType', 'HwDigital', ...
    'SamplesAcquiredFcnCount', round(sampleRate / updateRate), 'SampleRate', sampleRate, 'StartFcn', @scopeDemoStartFcn, ...
    'TriggerFcn', @scopeDemoTriggerFcn, 'StopFcn', @scopeDemoStopFcn);

%Configure a digital trigger.
dio = digitalio('nidaq', 1);
addline(dio, 0, 'out');
putvalue(dio, 0);

channelNames = {'scopeDemo_scaledOutput'};%The other board is having issues, so leave it out for now. 4/15/05

fprintf(1, 'scopeDemo: Starting...\n');

startChannel(dm, 'scopeDemoInput');

fprintf(1, 'Triggering...\n');
% tic
scopeDemoSamplesAcquiredFcnStrct.startTime = clock;
putvalue(dio, 1);
putvalue(dio, 0);
putvalue(dio, 1);

% stopChannel(dm, channelNames);%It will stop itself, when the 'SamplesPerTrigger' has been achieved.

return;

%-------------------------------------------------------
%An acquisition has just been started (but a trigger may not have been issued.
function scopeDemoStartFcn(ai, eventdata, varargin)

fprintf(1, '%s - scopeDemo/StartFcn: ''%s''\n', datestr(now), get(ai, 'name'));

return;

%-------------------------------------------------------
%A Trigger was just executed.
function scopeDemoTriggerFcn(ai, eventdata, varargin)

fprintf(1, '%s - scopeDemo/TriggerFcn: ''%s''\n', datestr(now), get(ai, 'name'));

return;

%-------------------------------------------------------
%Clean up and get ready for the next round of acquisitions.
function scopeDemoStopFcn(ai, eventdata, varargin)

%Make sure any left over samples get processed.
flushInputChannel(getDaqmanager, 'scopeDemoInput');

fprintf(1, '%s - scopeDemo/StopFcn: ''%s''\n', datestr(now), get(ai, 'name'));

return;

%-------------------------------------------------------
%Some data has just been acquired.
function scopeDemoSamplesAcquiredFcn(data, ai, eventdata, varargin)
global scopeDemoSamplesAcquiredFcnStrct;
% %Include some assorted profiling kinda stuff (which will also serve to generate more CPU load, since the command-line is soooo slow.
% toc
s = get(ai, 'SamplesAcquired');
scopeDemoSamplesAcquiredFcnStrct.samplesAcquired = scopeDemoSamplesAcquiredFcnStrct.samplesAcquired + length(data);
t = etime(clock, scopeDemoSamplesAcquiredFcnStrct.startTime);
e = scopeDemoSamplesAcquiredFcnStrct.samplesAcquired / get(ai, 'SampleRate');
if t > 1.1 * e
    fprintf('Warning: Falling behind...\n  ExpectedTime: %s\n  ElapsedTime: %s\n', num2str(e), num2str(t));
else
    fprintf('Keeping up, to within 10% of desired sample throughput.\n  ExpectedTime: %s\n  ElapsedTime: %s\n\n', num2str(e), num2str(t)');
end
fprintf(1, '%s - scopeDemo/scopeDemoSamplesAcquiredFcn: ''%s''\n    SamplesAcquired: %s\n    SamplesProcessed: %s\n', datestr(now), get(ai, 'Name'), ...
    num2str(get(ai, 'SamplesAcquired')), num2str(scopeDemoSamplesAcquiredFcnStrct.samplesAcquired));
% tic
return;