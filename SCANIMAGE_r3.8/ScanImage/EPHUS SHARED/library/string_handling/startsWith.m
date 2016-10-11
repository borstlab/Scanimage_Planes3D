% USAGE
%  startsWith(parentString, childString)
%
% Returns true if, and only if, childString is both a substring of parentString
% and the childString matches a contiguous sequence at the beginning of parentString.
%
% CHANGES
%  TO123005N: Also operate on cell arrays of strings. -- Tim O'Connor 12/30/05
%  TO091310C: Allow for cell arrays of endings, which is true if any endings match. -- Tim O'Connor 9/13/10
%
%Created Timothy O'Connor 8/26/04
%Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
function boolean = startsWith(string, beginning)

%TO122005N %TO091310C
if iscell(string) || iscell(beginning)
    if ~iscell(string)
        string = {string};
    end
    if ~iscell(beginning)
        beginning = {beginning};
    end

    boolean = zeros(size(string));
    for i = 1 : length(string)
        for j = 1 : length(beginning)
            boolean(i) = boolean(i) || startsWith(string{i}, beginning{j});
        end
    end

    return;
end

boolean = 0;

if length(string) < length(beginning)
    return;
end

boolean = strcmp(string(1 : length(beginning)), beginning);

return;