function ro = xformRect(ri,T)
    % T is a 2D affine
    % ri is [left top width height]
    r = T * [ri(3) 0 ri(1); 0 ri(4) ri(2); 0 0 1];
    ro = [r(7) r(8) r(1) r(5)];
end


%--------------------------------------------------------------------------%
% xformRect.m                                                              %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
