% @daqmanager/captureValue - Write a specific daq property onto a file handle.
%
% SYNTAX
%  captureValue(f, val)
%   f - A valid file handle.
%   val - The daqdevice value to be written (with translation into a text format).
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
function captureValue(f, val)

if isempty(val)
    if isnumeric(val)
        fprintf(f, '[]\r\n');
    else
        fprintf(f, '{}\r\n');
    end
elseif isnumeric(val)
    fprintf(f, '%s\r\n', mat2str(val));
else
    switch lower(class(val))
        case 'char'
            fprintf(f, '''%s''\r\n', val);
        case 'function_handle'
            fprintf(f, '@%s\r\n', func2str(val));
        case 'cell'
            fprintf(f, '{');
            for k = 1 : length(val) - 1
                if isnumeric(val{k})
                    fprintf(f, '%s, ', mat2str(val{k}));
                else
                    switch lower(class(val{k}))
                        case 'char'
                            fprintf(f, '''%s'', ', val{k});
                        case 'function_handle'
                            fprintf(f, '@%s, ', func2str(val{k}));
                        otherwise
                            fprintf(f, 'UNSUPPORTED_TYPE: %s, ', class(val{k}));
                    end
                end
            end
            if isnumeric(val{end})
                fprintf(f, '%s', mat2str(val{end}));
            else
                switch lower(class(val{end}))
                    case 'char'
                        fprintf(f, '''%s''', val{end});
                    case 'function_handle'
                        fprintf(f, '@%s', func2str(val{end}));
                    otherwise
                        fprintf(f, 'UNSUPPORTED_TYPE: %s', class(val{end}));
                end
            end
            fprintf(f, '}\r\n');
        otherwise
            fprintf(f, 'UNSUPPORTED_TYPE: %s\r\n', class(val));
    end
end

return;