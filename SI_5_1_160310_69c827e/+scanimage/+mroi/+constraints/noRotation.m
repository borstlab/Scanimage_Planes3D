function noRotation(roigroup,~,~)

for roi=roigroup.rois
    for sf=roi.scanfields
        if isprop(sf,'degrees') && sf.degrees ~= 0
            sf.degrees = 0;            
        end
    end
end

end


%--------------------------------------------------------------------------%
% noRotation.m                                                             %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
