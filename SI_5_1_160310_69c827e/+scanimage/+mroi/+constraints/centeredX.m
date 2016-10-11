function centeredX(roigroup,scannerset,~)

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
            
            w=tmpr(3);
            if abs(tmpr(1) - (1 - w)/2) > 0.0000001
                tmpr(1) = (1 - w)/2;
                
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
% centeredX.m                                                              %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
