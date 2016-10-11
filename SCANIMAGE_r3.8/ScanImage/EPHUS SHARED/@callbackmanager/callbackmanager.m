% callbackmanager - An object used to track callbacks bound to strings, with unique identifiers.
%
%  SYNTAX
%   cbm = callbackmanager
%   cbm = callbackmanager(index)
%   cbm = callbackmanager(instance)
%    cbm - A @callbackmanager instance.
%    index - A pointer index to an existing instance.
%    instance - An existing callbackmanager instance, simply returns the same instance.
%
%  STRUCTURE
%   callbacks - A 2xN cell array. The first column is a string (representing the event), the second column contains 
%               a structure array.
%       nested structure:
%         id - A unique identifier for the callback, to facilitate deletion.
%         callbackSpec - The actual callback (a function_handle, a cell array whose first element is a function_handle, or a string).
%         priority - A priority specifier, to help in ordering callbacks during execution.
%   handlesOnly - A flag that disallows string based callbacks to be added.
%   enable - When set to 0 fireEvent calls have no effect.
%            Default: 1
%
%  USAGE
%
%  CHANGES
%   TO010506D: Added the 'enable' field. This is to simplify the UserFcn implementation. -- Tim O'Connor 1/5/06
%
% CREDITS
% Created 5/12/05 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function this = callbackmanager(varargin)
global callbackmanagers;

if length(varargin) == 1
    if isempty(varargin{1})
        error('[] and {} are not a valid constructor arguments.');
    elseif isnumeric(varargin{1})
        if 0 < varargin{1} && varargin{1} < length(callbackmanagers)
            this.ptr = varargin{1};
        else
            error('Pointer index out of range: %s', num2str(varargin{1}));
        end
    elseif strcmpi(class(varargin{1}), 'callbackmanager')
        this = varargin{1};
        return;
    else
        this.ptr = length(callbackmanagers) + 1;
        callbackmanagers(this.ptr).callbacks = {};
        callbackmanagers(this.ptr).handlesOnly = 0;
        callbackmanagers(this.ptr).enable = 1;%TO010506D
        callbackmanagers(this.ptr).readOnlyFields = {'readOnlyFields'};%TO010506D
    end
elseif isempty(varargin)
    this.ptr = length(callbackmanagers) + 1;
    callbackmanagers(this.ptr).callbacks = {};
    callbackmanagers(this.ptr).handlesOnly = 0;
    callbackmanagers(this.ptr).enable = 1;
    callbackmanagers(this.ptr).readOnlyFields = {'readOnlyFields'};%TO010506D
else
    error('Too many input arguments.');
end

this = class(this, 'callbackmanager');

return;
