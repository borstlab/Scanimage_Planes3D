function maxWidth(roigroup,scannerset,~)

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
            
            if isa(scannerset, 'scanimage.mroi.scannerset.ResonantGalvoGalvo') && ~scannerset.resonantLimitedFovMode
                resW = 0.00000001 + scannerset.fillFractionSpatial * scannerset.scanners{1}.fullAngleDegrees / (scannerset.scanners{1}.fullAngleDegrees + scannerset.scanners{2}.fullAngleDegrees);
                if tmpr(3) > resW
                    tmpr(1) = tmpr(1) - (resW - tmpr(3)) / 2; %maintain center
                    tmpr(3) = resW;
                    
                    if strcmp(roigroup.coordSys,scannerset.name)
                        s.rect = tmpr;
                    else
                        s.rect = most.idioms.xformRect(tmpr,scannerset.scannerToRefTransform);
                    end
                end
            else
                if abs(tmpr(3)) > 1.00000001 * scannerset.fillFractionSpatial
                    newW = 1 * scannerset.fillFractionSpatial * sign(tmpr(3));
                    tmpr(1) = tmpr(1) - (newW - tmpr(3)) / 2; %maintain center
                    tmpr(3) = newW;
                    
                    if strcmp(roigroup.coordSys,scannerset.name)
                        s.rect = tmpr;
                    else
                        s.rect = most.idioms.xformRect(tmpr,scannerset.scannerToRefTransform);
                    end
                end
            end
        end
    end
end

end


%--------------------------------------------------------------------------%
% maxWidth.m                                                               %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
