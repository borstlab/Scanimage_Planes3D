% @daqmanager/inList - Check for existence of a daqobject in a cell array of daqobjects.
%
% SYNTAX
%  isInList = inList(obj, list)
%   obj - A daq object.
%   list - A cell array of daqobjects, cells may be empty.
%   isInList - A boolean, true if the object is in the list.
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
function isInList = inList(obj, list)
%TO022706D: Optimization. Rewritten to take advantage of `ismember`. -- Tim O'Connor 2/27/06
isInList = any(ismember(obj, [list{:}]));

return;

% isInList = 0;
% 
% for i = 1 : length(list)
%     
%     if strcmpi(class(list), 'cell')
%         if obj == list{i}
%             isInList = 1;
%             return;
%         end
%     else
%         if obj == list(i)
%             isInList = 1;
%             return;
%         end
%     end
% end
% 
% return;