function xCenterInRange(roigroup,scannerset,~)

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
            
            checkByBounds = true;
            
            if isa(scannerset, 'scanimage.mroi.scannerset.ResonantGalvoGalvo')
                if scannerset.resonantLimitedFovMode
                    %leave checkByBounds true. When in this mode we must check both that the sf is within galvo range
                    %and that the entire rectangle is in range due to possible combinations of angular ranges
                    xGalvoRg = scannerset.scanners{2}.fullAngleDegrees / scannerset.scanners{1}.fullAngleDegrees;
                else
                    checkByBounds = false; %the combination of max width and center in range makes the rectangle bound check unnecessary
                    xGalvoRg = scannerset.scanners{2}.fullAngleDegrees / (scannerset.scanners{1}.fullAngleDegrees + scannerset.scanners{2}.fullAngleDegrees);
                end
                
                roiMddl = tmpr(1) + tmpr(3)/2 - .5; %in -.5 to .5 coords
                
                if (abs(roiMddl) - xGalvoRg/2) > 0.000001
                    tmpr(1) = .5 + xGalvoRg/2 * sign(roiMddl) - tmpr(3)/2;
                    
                    if strcmp(roigroup.coordSys,scannerset.name)
                        s.rect = tmpr;
                    else
                        s.rect = most.idioms.xformRect(tmpr,scannerset.scannerToRefTransform);
                    end
                end
            end

            
            if checkByBounds

                tmpr(1) = correctLeft(tmpr(1), tmpr(3), scannerset.fillFractionSpatial);

                if strcmp(roigroup.coordSys,scannerset.name)
                    s.rect = tmpr;
                else
                    s.rect = most.idioms.xformRect(tmpr,scannerset.scannerToRefTransform);
                end
            end
        end
    end
end

function left = correctLeft(left, width, fillFrac)
% function correctLeft - Adjusts left, taking into account that width might be negative
% Corrects left to have the rectangle between the ranges of 0 to 1
% preserving the sign of the width
    
    widthF = width/fillFrac;
    leftF = left - abs(widthF-width)/2;
    
    [minVal] = min([leftF, leftF+widthF]);
    if minVal < 0
        if minVal == leftF
            leftF = 0;
        else
            leftF = -widthF;
        end
    end

    [maxVal] = max([leftF, leftF+widthF]);
    if maxVal > 1
        if maxVal == leftF
            leftF = 1;
        else
            leftF = 1-widthF;
        end
    end
    
    left = leftF + abs(widthF-width)/2;
end

end

%NOTE:  We are assuming that rect widths and heights can be negative
%       The current roi mechanism ensures MROI mode doesn't do this, 
%       but stimulus rois currently require it.
%       After some testing, incorrect flipping issues couldn't be reproduced, but 
%       if any are observed, they might be related to this.
%TODO:  Consider adding a "flipXY" property to simplify constraint calculations.


%--------------------------------------------------------------------------%
% xCenterInRange.m                                                         %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
