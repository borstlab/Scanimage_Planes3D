% USAGE
%  endsWith(parentString, childString)
%
% Returns true if, and only if, childString is both a substring of parentString
% and the childString matches a contiguous sequence at the end of parentString.
%
% CHANGES
%  TO123005N: Also operate on cell arrays of strings. -- Tim O'Connor 12/30/05
%  TO091310C: Allow for cell arrays of endings, which is true if any endings match. -- Tim O'Connor 9/13/10
%
%Created Timothy O'Connor 8/26/04
%Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
function boolean = endsWith(string, ending)

%TO122005N %TO091310C
if iscell(string) || iscell(ending)
    if ~iscell(string)
        string = {string};
    end
    if ~iscell(ending)
        ending = {ending};
    end

    boolean = zeros(size(string));
    for i = 1 : length(string)
        for j = 1 : length(ending)
            boolean(i) = boolean(i) || endsWith(string{i}, ending{j});
        end
    end

    return;
end

boolean = 0;

if length(string) < length(ending)
    return;
end

boolean = strcmp(string(length(string) - length(ending) + 1: end), ending);

return;