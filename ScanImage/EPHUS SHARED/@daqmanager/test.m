%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  An everchanging test function, with privledges to act on the object's structure.
%%
%%  Ideally this should run through a bunch of test cases.
%%
%%  OBJ = test(OBJ, varargin)
%%
%%  Created - Tim O'Connor 11/13/03
%%
%%  Changed:
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dm = test(dm, varargin)
global gdm;

verbose = 0;
for i = 1 : length(varargin)
    if strcmpi(varargin{i}, 'verbose')
        verbose = 1;
    end
end

%-------------------------------------------------------
%START Logic tests.
fprintf(1, 'Logic tests -\n');
fprintf(1, ' Exist: %s\n', num2str(exist(dm)));
fprintf(1, ' Equals itself: %s\n', num2str(dm == dm));
fprintf(1, ' Does not equal itself: %s\n', num2str(dm ~= dm));
fprintf(1, ' Not: %s\n', num2str(~dm));
fprintf(1, ' Or itself: %s\n', num2str(dm | dm));

fprintf(1, 'Setting up hardware trigger...\n');
dio = digitalio('nidaq', 1);
triggerLine = addLine(dio, 0, 'out');
%START Logic tests.
%-------------------------------------------------------

%-------------------------------------------------------
%START Create output channels and set properties.

%Channel 1
data = ones(5000, 1);
data(2500 : 5000) = .5 * data(2500 : 5000);

fprintf(1, 'Creating output channel 1...\n');
nameOutputChannel(dm, 3, 1, 'o-ch1');

fprintf(1, 'Setting sample rate on output channel 1...\n');
setAOProperty(dm, 'o-ch1', 'SampleRate', 1000);

fprintf(1, 'Setting trigger type on output channel 1...\n');
setAOProperty(dm, 'o-ch1', 'TriggerType', 'HwDigital');

fprintf(1, 'Enabling output channel 1...\n');
enableChannel(dm, 'o-ch1');

fprintf(1, 'Putting data on output channel 1...\n');
putDaqData(dm, 'o-ch1', data);

if verbose
    dm%Display
end

%Channel 2
data(2500 : 5000) = zeros(2501, 1);

fprintf(1, 'Creating output channel 2...\n');
nameOutputChannel(dm, 3, 0, 'o-ch2');

fprintf(1, 'Setting sample rate on output channel 2...\n');
setAOProperty(dm, 'o-ch2', 'SampleRate', 1000);

fprintf(1, 'Setting trigger type on output channel 2...\n');
setAOProperty(dm, 'o-ch2', 'TriggerType', 'HwDigital');

fprintf(1, 'Enabling output channel 2...\n');
enableChannel(dm, 'o-ch2');

fprintf(1, 'Putting data on output channel 2...\n');
putDaqData(dm, 'o-ch2', data);

if verbose
    dm%Display
end

%ERROR CONDITIONS---------------------------------------
try
    fprintf(1, 'Creating ''duplicate'' output channel...\n');
    nameOutputChannel(dm, 3, 0, 'ch3');
catch
    fprintf(1, 'Recieved error, as expected: \n %s\n', lasterr);
end

%END Create output channels and set properties.
%-------------------------------------------------------

%-------------------------------------------------------
%START Start, trigger, stop output channels
fprintf(1, 'Starting both output channels...\n');

startChannel(dm, 'o-ch1', 'o-ch2');
if verbose
    dm%Display
end

try
    fprintf(1, 'Enabling output channel 1 (again)...\n');
    enableChannel(dm, 'o-ch1');
catch
    fprintf(1, 'Recieved error, as expected: \n %s\n', lasterr);
end

fprintf(1, '\n\n');

fprintf(1, 'Triggering both output channels...\n');
putvalue(triggerLine, 0);
putvalue(triggerLine, 1);
putvalue(triggerLine, 0);

fprintf(1, 'Stopping both output channels...\n');
% n = 1;
% % strcmp(getAOField(dm, 'o-ch1', 'Running'), dm.aos(1).Running)
% while strcmpi(getAOField(dm, 'o-ch1', 'Running'), 'On')
%     if n == 1
%         sendingStatus = getAOField(dm, 'o-ch1', 'Sending')
%         runningStatus = getAOField(dm, 'o-ch1', 'Running')
%         n = 0;
%     end
%         
% end

% stopChannelImmediate(dm, 'o-ch1', 'o-ch2');
stopChannel(dm, 'o-ch1', 'o-ch2');
if verbose
    dm%Display
end

%ERROR CONDITIONS---------------------------------------
fprintf(1, 'Starting both output channels...\n');
startChannel(dm, 'o-ch1', 'o-ch2');

fprintf(1, 'Enabling output channel 1 (again)...\n');
enableChannel(dm, 'o-ch1');

try
    fprintf(1, 'Starting both output channels...\n');
    startChannel(dm, 'o-ch1', 'o-ch2');
catch
    fprintf(1, 'Recieved error, as expected: \n %s\n', lasterr);
end

fprintf(1, 'Disable output channel 1...\n');
disableChannel(dm, 'o-ch1');

%END Start, trigger, stop output channels.
%-------------------------------------------------------

%-------------------------------------------------------
%START putsample
fprintf(1, 'Putting sample on output channel 1...\n');
putDaqSample(dm, 'o-ch1', 2);
if verbose
    dm%Display
end

fprintf(1, 'Putting sample on output channel 2...\n');
putDaqSample(dm, 'o-ch2', 2);
if verbose
    dm%Display
end

try
    fprintf(1, 'Put sample on non-existent output channel...\n');
    putDaqSample(dm, 'ch3', 2);
catch
    fprintf(1, 'Recieved error, as expected: \n %s\n', lasterr);
end

%ERROR CONDITIONS---------------------------------------
%END putsample
%-------------------------------------------------------

%-------------------------------------------------------
%START Remove output channels.
fprintf(1, 'Removing output channel 1...\n');
denameOutputChannel(dm, 'o-ch1');
if verbose
    dm%Display
end

fprintf(1, 'Removing output channel 2...\n');
denameOutputChannel(dm, 'o-ch2');
if verbose
    dm%Display
end
%ERROR CONDITIONS---------------------------------------
%END Remove output channels.
%-------------------------------------------------------

%-------------------------------------------------------
%START Create input channels and set properties.
fprintf(1, 'Creating input channel 0...\n');
nameInputChannel(dm, 1, 0, 'i-ch0');

fprintf(1, 'Setting trigger type on input channel 0...\n');
setAIProperty(dm, 'i-ch0', 'TriggerType', 'HwDigital');

fprintf(1, 'Enabling input channel 0...\n');
enableChannel(dm, 'i-ch0');

if verbose
    dm%Display
end

fprintf(1, 'Creating input channel 1...\n');
nameInputChannel(dm, 1, 1, 'i-ch1');

fprintf(1, 'Setting trigger type on input channel 1...\n');
setAIProperty(dm, 'i-ch1', 'TriggerType', 'HwDigital');

fprintf(1, 'Enabling input channel 1...\n');
enableChannel(dm, 'i-ch1');

if verbose
    dm%Display
end

%END Create input channels and set properties.
%-------------------------------------------------------

%-------------------------------------------------------
%START Start, trigger, stop output channels
fprintf(1, 'Starting both input channels...\n');
startChannel(dm, 'i-ch0', 'i-ch1');
if verbose
    dm%Display
end

fprintf(1, 'Triggering both input channels...\n');
putvalue(triggerLine, 0);
putvalue(triggerLine, 1);
putvalue(triggerLine, 0);

fprintf(1, 'Stopping both input channels...\n');
stopChannel(dm, 'i-ch0', 'i-ch1');
if verbose
    dm%Display
end

%ERROR CONDITIONS---------------------------------------
fprintf(1, 'Starting both input channels...\n');
startChannel(dm, 'i-ch0', 'i-ch1');

fprintf(1, 'Enabling input channel 1 (again)...\n');
enableChannel(dm, 'i-ch1');

try
    fprintf(1, 'Starting both input channels...\n');
    startChannel(dm, 'i-ch0', 'i-ch1');
catch
    fprintf(1, 'Recieved error, as expected: \n %s\n', lasterr);
end

fprintf(1, 'Disable input channel 1...\n');
disableChannel(dm, 'i-ch1');
fprintf(1, 'Stopping both input channels...\n');
stopChannel(dm, 'i-ch0', 'i-ch1');
if verbose
    dm%Display
end

%END Start, trigger, stop output channels
%-------------------------------------------------------

%-------------------------------------------------------
%START Get data.

fprintf(1, 'Starting both input channels...\n');
startChannel(dm, 'i-ch0', 'i-ch1');
if verbose
    dm%Display
end

fprintf(1, 'Triggering both input channels...\n');
putvalue(triggerLine, 0);
putvalue(triggerLine, 1);
putvalue(triggerLine, 0);

fprintf(1, 'Getting data from both input channels...\n');
if verbose
    dm%Display
end
[data0 data1] = getDaqData(dm, 'i-ch0', 'i-ch1');
fprintf(1, 'Found %s datapoints on input channel 0 and %s datapoints on input channel 1.\n', num2str(length(data0)), num2str(length(data1)));

fprintf(1, 'Stopping both input channels...\n');
stopChannel(dm, 'i-ch0', 'i-ch1');
if verbose
    dm%Display
end

%END Get data.
%-------------------------------------------------------

%-------------------------------------------------------
%START Remove output channels.
fprintf(1, 'Removing input channel 0...\n');
denameOutputChannel(dm, 'i-ch0');
if verbose
    dm%Display
end

fprintf(1, 'Removing input channel 1...\n');
denameOutputChannel(dm, 'i-ch1');
if verbose
    dm%Display
end
%ERROR CONDITIONS---------------------------------------
%END Remove output channels.
%-------------------------------------------------------

%-------------------------------------------------------
%START Delete object.
fprintf(1, 'Deleting object...\n');
delete(dm);
%ERROR CONDITIONS---------------------------------------
try
    fprintf(1, 'Attempting to add output channel to deleted object...\n');
    addChannel(dm, 'ch3', 1, 1);
catch
    fprintf(1, 'Recieved error, as expected: \n %s\n', lasterr);
end
%END Delete object.
%-------------------------------------------------------

fprintf(1, '\n\nTest complete.\n\n');