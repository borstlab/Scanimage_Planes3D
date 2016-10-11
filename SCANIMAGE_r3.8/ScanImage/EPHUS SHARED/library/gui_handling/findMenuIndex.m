function out=findMenuIndex(handle, entry)
% FINDMENUINDEX   - Listbox/Popupmenu Name to index converter.
%   FINDMENUINDEX will take the gui handle (class uitool) and a text entry
%   (char) and out put the index that corresponds to that index in the
%   GUI. 
%
%   This function is very useful when dealing with popupmenus and listboxes
%   in GUIs.
%
% See also GETMENUENTRY

% Changes:
% 	TPMOD1 (2/4/04) - Rewritten and Commented.

str=get(hObject,'String');
if ~iscellstr(str)
	str={str};
end
labelBinaryArray=strcmpi(str,entry);
out=find(labelBinaryArray);
if isempty(out)
	out=0;
end


% 
% val=get(hObject,'Value');
% out=[];
% if ~iscellstr(str)
% 	out=str;
% % elseif index >= 1 & index <= length(str)
% % 	out=str{val};
% % end
% % 
% 
% 	out=0;
% 	
% 	if strcmp(get(handle, 'Style'), 'popupmenu')==0
% 		disp(['findMenuIndex: called with handle to non-popmenu.  ' get(handle, 'Tag') ' is of style ' get(handle, 'Style')]);
% 		return
% 	end
% 	
% 	if ~isnumeric(entry)
% 		val=str2num(entry);
% 		if isnumeric(val) & length(val)>0
% 			entry=val;
% 		end
% 	end
% 		
% 	menuItems=get(handle, 'String');
% 	
% 	for i=1:length(menuItems)
% 		label=menuItems{i};
% 		val=str2num(label);
% 		if isnumeric(val) & length(val)>0
% 			label=val;
% 			if isnumeric(entry)
% 				if entry==label
% 					out=i;
% 					return
% 				end
% 			end
% 		else
% 			if ~isnumeric(entry)
% 				if strcmp(label,entry)
% 					out=i;
% 					return
% 				end
% 			end
% 		end
% 	end
% 	