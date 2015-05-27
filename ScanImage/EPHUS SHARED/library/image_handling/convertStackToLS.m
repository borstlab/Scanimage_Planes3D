function out=convertStackToLS(array)
% CONVERTSTACKTOLS   - Converts a 3D array into a 2D array preserving row order.
%   CONVERTSTACKTOLS will take a 2 or 3 dimensional stack of data of any class
%   (double, uint16, etc..) and reshape it such that it is 2D.  If the input
%   array has size [m n l] the output array will have size [m*l n].
%
%   See also RESHAPE, PERMUTE

if size(array,3) > 1
    array=permute(array,[2 1 3]);
    out=reshape(array,size(array,1),size(array,2)*size(array,3))';
else
	out=array;
end

