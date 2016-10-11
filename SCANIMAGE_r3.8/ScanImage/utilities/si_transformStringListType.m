function out = si_transformStringListType(in)
%SI_TRANSFORMSTRINGLISTTYPE Converts between cell array and pipe-delimited string formats for a list of strings
%
%% NOTES
%   This is useful for list-type UIControls whose 'String' property can take either of these formats, each of which is useful for different purposes
%   E.g cell array is useful while processing, but string can be more convenient for saving/loading from file
%
%% CREDITS
%   Created 12/1/09, by Vijay Iyer
%% ****************************************

if iscellstr(in)
    out = '';
    if ~isempty(in)
        out = in{1};    
        for i=2:length(in)
            out = [out '|' in{i}];
        end
    end      
elseif ischar(in)
    out = {};
    while ~isempty(in)
        [tok,in] = strtok(in,'|');
        out{end+1} = tok;                      
    end         
else
    error('Input must be a cell string array or a pipe-delimited string');
end
    
    
