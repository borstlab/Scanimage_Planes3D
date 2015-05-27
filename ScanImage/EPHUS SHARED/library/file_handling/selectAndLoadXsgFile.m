% selectAndLoadXsgFile - Browse for and load an xsg file.
%
%  SYNTAX
%   xsgData = selectAndLoadXsgFile
%   [xsgData, fname] = selectAndLoadXsgFile
%    xsgData - The structure stored in the selected xsg file (empty if cancelled).
%    fname - The full path of the file that was loaded.
%
%  NOTES
%   Relies on getStackTraceString and The Mathworks's (new as of version 7) lasterror function.
%
%  CHANGES
%
% Created 11/30/07 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2007
function [xsgData, fname] = selectAndLoadXsgFile

xsgData = [];

qvDir = getDefaultCacheDirectory(progmanager, 'qvDir');
[f, p] = uigetfile(fullfile(qvDir, '*.xsg'));
if length(f) == 1
    if f == 0
        return;
    end
elseif length(p) == 1
    if p == 0
        return;
    end
end

fname = fullfile(p, f);
setDefaultCacheValue(progmanager, 'qvDir', p);
xsgData = load(fname, '-mat');

return;