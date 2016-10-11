function maxHeight(roigroup,scannerset,~)

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
            
            if abs(tmpr(4)) > 1.00000001
                nwH = 1 * sign(tmpr(4));
                tmpr(2) = tmpr(2) - (nwH - tmpr(4)) / 2; %maintain center
                tmpr(4) = nwH;
                
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


%--------------------------------------------------------------------------%
% maxHeight.m                                                              %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
