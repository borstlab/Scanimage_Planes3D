function scaleXData(handle,factor)
% SCALEXDATA   - Changes 'XData' for current axes.
%   SCALEXDATA rescales the xdata on an axes by a factor by getting all
%   the xdatas from the plots and multiplying them.  This funcion will not
%   work if the XDatas are of different lengths.
%
%   See also getDataFromHandle

[xdata,handles]=getDataFromHandle(handle,'dataToGetFromObject','XData');
set(handles,{'XData'},mat2cell(xdata*factor,ones(1,size(xdata,1)),size(xdata,2)));
axis(handle,'tight');
