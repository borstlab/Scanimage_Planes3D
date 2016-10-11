function m = str2map(s)
%STR2MAP Convert a string to a containers.Map object
% m = str2map(s)
% 
% The input string s should be in the format generated by map2str.

if isempty(s)
    m = containers.Map;
    return;
else
    s = [s ' | ']; % add last divider to make all key-val pairs identical
end

pat = '([^:|])+:([^|])+|';
toks = regexp(s,pat,'tokens');

keys = cellfun(@(x)strtrim(x{1}),toks,'UniformOutput',false);

% treat keys as numeric if they can all be converted successfully to numeric
numkeys = cellfun(@str2double,keys);
if ~any(isnan(numkeys))
    keys = numkeys;
end 

vals = cellfun(@(x)strtrim(x{2}),toks,'UniformOutput',false);
tfUnencodeable = strcmp(vals,'<unencodeable value>');
vals(tfUnencodeable) = {'[]'};
vals = cellfun(@eval,vals,'UniformOutput',false);

m = containers.Map(keys,vals);

end


%--------------------------------------------------------------------------%
% str2map.m                                                                %
% Copyright � 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
