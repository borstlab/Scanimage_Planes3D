% displayErrorStack - Displays the stack of functions leading up to last error
%
%  ARGS 
%      NONE
%   
%  DESCRIPTION
%       
%
%  NOTES
%   Largely a cut & paste job from `getStackTraceString.m`
%
%  CHANGES
%  vi08312006a -- Added compatability with Matlab V6 by directly calling getStackTraceString()--not as much info as in V7.
%
%  CREATED 6/5/06
%   Vijay Iyer & Tim O'Connor
%   Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
%
%------------------------------------------------------------------

function [] = displayErrorStack()
% Displays stack associated of last error


matlabVersionString = version;
if str2num(matlabVersionString(1:3)) < 7  %vi08312006a
    fprintf(1, '%s', getStackTraceString(1));
else
    errorstruct = lasterror;
    stackTraceStruct = errorstruct.stack;
    
    stackTraceString = '';
    len = length(stackTraceStruct);%TO022706D: Cache the length. %TO031006H: Use the struct here, don't call `dbstack` again. -- Tim O'Connor 3/10/06
    if len > 0
        for i = 1 : len
            if i == 1
                stackTraceString = sprintf('%s - Error ''%s'' in function ''%s'':\n', datestr(now, 0), lasterr, stackTraceStruct(i).name);%TO010606E
                stackTraceString = sprintf('%s  In %s at line %s...\n',  stackTraceString, stackTraceStruct(i).name, num2str(stackTraceStruct(i).line));
            elseif i ~= len
                stackTraceString = sprintf('%s   in %s at line %s...\n', stackTraceString, stackTraceStruct(i).name, num2str(stackTraceStruct(i).line));
            elseif i == len
                stackTraceString = sprintf('%s   in %s at line %s.\n', stackTraceString, stackTraceStruct(i).name, num2str(stackTraceStruct(i).line));    
            end
        end
    else
        stackTraceString = sprintf('%s - Error ''%s'': NO_STACK_AVAILABLE\n', datestr(now, 0), lasterr);%TO031006H
    end
    
    disp(stackTraceString);
end

return;