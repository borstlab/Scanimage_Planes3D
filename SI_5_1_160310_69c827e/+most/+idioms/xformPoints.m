function pts = xformPoints(pts,T)
    % T is a 2D affine
    % pts is [N x 2] array of xy points
    pts(:,3) = 1;
    pts = pts * T';
    pts(:,3) = [];
end


%--------------------------------------------------------------------------%
% xformPoints.m                                                            %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
