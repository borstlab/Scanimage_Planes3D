function varargout=switchString(hObject,str1,str2)
%SWITCHSTRING   - Exchanges string names of a GUI's uicontrol object.
% 	SWITCHSTRING will replace the current string in a handle
% 	(hObject) with either str1 or str2, depending on if the current string is
% 	str1 or str2.  It will effectively toggle the object's string between the
% 	2 strings str1 and str2.
%
%   Output is the string that was switched to.
%
%   See also 

out='';
current=get(hObject,'String');
if strcmpi(current,str1)
	out=str2;
	set(hObject,'String',str2);
elseif strcmpi(current,str2)
	out=str1;
	set(hObject,'String',str1);
end
if nargout==1
	varargout{1}=out;
end
