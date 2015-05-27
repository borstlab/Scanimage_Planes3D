function set(obj,varargin)
%SET   - Overloaed method for @progmanager class.
%   SET will set properties for the object.  Varargin are param/value
%   pairs to be set for the program.
%
%   See also PROGMANAGER

if nargin < 3 
   error('@progmanager/set: too few inputs.');
end

% parse the param value pair inputs.
for param_counter=1:2:length(varargin)
    if ischar(varargin{param_counter}) & isfield(obj,(varargin{param_counter}))
        obj.(varargin{param_counter})=varargin{param_counter+1};
    else
        error(['@progmanager/set: invalid property ' varargin{param_counter} ' for progmanager object.']);
    end
end