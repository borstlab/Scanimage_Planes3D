% getStackTraceString - Returns a formatted string showing the current execution stack.
%
%  ARGS 
%      NONE - Returns the entire current execution stack.
%      level - Takes a single integer, telling the function to discard the highest 'level' 
%              stack frames in the trace. This is useful for only looking at calling functions.
%
%  DESCRIPTION
%      getStackTraceString(1) - returns a stack trace, ignoring the function that called `getStackTraceString`
%                               and only showing the functions that lead up to it.
%
%      getStackTraceString(5) - returns a stack trace, ignoring the 5 functions that called `getStackTraceString`
%                               and only shows previously called functions.
% CHANGES
%   TO021605a: Pulled out the TeX safety in the string, and inserted it into a new function 'texSafe', which may be called separately. -- Tim O'Connor 2/16/05
%   TO010606E: Optimization(s). `datestr(now)` works just as well as `datestr(datevec(now))`. -- Tim O'Connor 1/6/06
%   TO022706D: Optimization(s). Cache the length of the `dbstack` array. The `i ~= length(dbstack)` statement was eating lots of time (50% of the total function's time). -- Tim O'Connor 2/27/06
%   TO031006H: Always return something, at minimum the timestamp, even if there is no calling function context on the stack. Small optimization too. -- Tim O'Connor 3/10/06
%   TO071906C: Updated for Matlab 7.2's new `dbstack` options, while maintaining backwards compatibility. -- Tim O'Connor 7/19/06
%   TO101807F: Allow the user to provide a stack, to be formatted, as per the lasterror function. -- Tim O'Connor 10/18/07

%------------------------------------------------------------------
%Tim O'Connor TO072104b: Factored out the stack trace construction.
function stackTraceString = getStackTraceString(varargin)

stackTraceStruct = [];
if isempty(varargin) || length(varargin) > 1
    primaryLevel = 2;
elseif length(varargin) == 1
    if isstruct(varargin{1})
        stackTraceStruct = varargin{1};
        primaryLevel = 0;
    elseif varargin{1} >= 0
        primaryLevel = 2 + varargin{1};
    end
else
    primaryLevel = 2;
end

% %Construct a stack trace. Start from calls outside of startChannel.m and indent it nicely.
%Construct a stack trace. Start from calls outside of this function and any number of requested callers and indent it nicely.
%TO071906C - Updated for Matlab 7.2 - Note: passing a number into `dbstack` doesn't work, because Matlab sucks.
verstring = version;
if str2double(verstring(1:3)) >= 7.2
    if isempty(stackTraceStruct)
        stackTraceStruct = dbstack('-completenames');
    end
    for i = 1 : length(stackTraceStruct)
        stackTraceStruct(i).name = [stackTraceStruct(i).file '/' stackTraceStruct(i).name ];
    end
else
    if isempty(stackTraceStruct)
        stackTraceStruct = dbstack;
    end
end

stackTraceString = '';
len = length(stackTraceStruct);%TO022706D: Cache the length. %TO031006H: Use the struct here, don't call `dbstack` again. -- Tim O'Connor 3/10/06
if primaryLevel < len
    for i = 1 : len
        if i == primaryLevel
            stackTraceString = sprintf('%s - %s called:\n', datestr(now, 0), stackTraceStruct(i).name);%TO010606E
        elseif i == primaryLevel + 1
            stackTraceString = sprintf('%s  In %s at line %s...\n',  stackTraceString, stackTraceStruct(i).name, num2str(stackTraceStruct(i).line));
        elseif i ~= len
            stackTraceString = sprintf('%s   in %s at line %s...\n', stackTraceString, stackTraceStruct(i).name, num2str(stackTraceStruct(i).line));
        elseif i > primaryLevel + 1
            stackTraceString = sprintf('%s   in %s at line %s.\n', stackTraceString, stackTraceStruct(i).name, num2str(stackTraceStruct(i).line));    
        end
    end
else
    stackTraceString = sprintf('%s - '''' called: NO_STACK_AVAILABLE\n', datestr(now, 0));%TO031006H
end

return;