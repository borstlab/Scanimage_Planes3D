% validateCallbackSpec(callbackSpec)
%
% Checks that the callbackSpec is either a:
%   string
%   function_handle
%   cell array whose first element is a function_handle
%
% If callbackSpec does not meet the above criteria, an error, with an informative
% message is thrown.
%
% NOTES:
%  No assertions are made about the correctness of functions/arguments. Strings are not eagerly evaluated.
%
% Created 1/20/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function validateCallbackSpec(callbackSpec)

if ~strcmpi(class(callbackSpec), 'function_handle') & ~strcmpi(class(callbackSpec), 'char') & ...
        ~strcmpi(class(callbackSpec), 'cell')
    error('Invalid callback specification. Must be a string, function_handle, or cell array: %s', class(callbackSpec));
elseif strcmpi(class(callbackSpec), 'cell')
    if ~strcmpi(class(callbackSpec{1}), 'function_handle')
        error('Invalid cell array callback specification. The cell array''s first element must be a function_handle: %s', class(callbackSpec{1}));
    end
end

return;