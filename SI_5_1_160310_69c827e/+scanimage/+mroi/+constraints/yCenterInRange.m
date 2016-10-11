function yCenterInRange(roigroup,scannerset,~)

for roi=roigroup.rois
    for s=roi.scanfields
        if isprop(s,'rect')
            if strcmp(roigroup.coordSys,scannerset.name)
                tmpr = s.rect;
            elseif strcmp(roigroup.coordSys,'Reference')
                tmpr = most.idioms.xformRect(s.rect,scannerset.refToScannerTransform);
            else
                assert(false, 'RoiGroup is in an unexpected coordinate system.');
            end
            
            tmpr(2) = correctTop(tmpr(2), tmpr(4));
            
            if strcmp(roigroup.coordSys,scannerset.name)
                s.rect = tmpr;
            else
                s.rect = most.idioms.xformRect(tmpr,scannerset.scannerToRefTransform);
            end
        end
    end
end

function top = correctTop(top, height)
% function correctTop - Adjusts top, taking into account that height might be negative
% Corrects top to have the rectangle between the ranges of 0 to 1
% preserving the sign of the height

    [minTopVal] = min([top, top+height]);
    if minTopVal < 0
        if minTopVal == top
            top = 0;
        else
            top = -height;
        end
    end

    [maxTopVal] = max([top, top+height]);
    if maxTopVal > 1
        if maxTopVal == top
            top = 1;
        else
            top = 1-height;
        end
    end
end
end

%NOTE:  We are assuming that rect widths and heights can be negative
%       The current roi mechanism ensures MROI mode doesn't do this, 
%       but stimulus rois currently require it.
%       After some testing, incorrect flipping issues couldn't be reproduced, but 
%       if any are observed, they might be related to this.
%TODO:  Consider adding a "flipXY" property to simplify constraint calculations.


%--------------------------------------------------------------------------%
% yCenterInRange.m                                                         %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
