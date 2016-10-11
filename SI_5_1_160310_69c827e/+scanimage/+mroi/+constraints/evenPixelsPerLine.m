function evenPixelsPerLine(roigroup,~,~)

for roi=roigroup.rois
    for s=roi.scanfields
        if isprop(s,'pixelResolution')
            if mod(s.pixelResolution(1),2)~=0
                s.pixelResolution(1) = s.pixelResolution(1)+1;
            end
        end
    end
end

end


%--------------------------------------------------------------------------%
% evenPixelsPerLine.m                                                      %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
