function [xs,ys] = xformPointsXY(xs,ys,T)
    % T is a 2D affine
    % x is [m x n] array of x points
    % y is [m x n] array of y points
    r=[xs(:),ys(:),ones(size(xs(:)))];
    r=r*T';
    xs=reshape(r(:,1),size(xs));
    ys=reshape(r(:,2),size(ys));
end


%--------------------------------------------------------------------------%
% xformPointsXY.m                                                          %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
