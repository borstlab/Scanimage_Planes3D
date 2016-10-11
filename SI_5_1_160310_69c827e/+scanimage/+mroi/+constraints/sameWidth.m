function sameWidth(roigroup,~,scanfield)
if nargin < 2 || isempty(scanfield)
    width = [];
else
    if isprop(scanfield,'rect')
        width=scanfield.rect(3);
    else
        width = [];
    end
end

for roi=roigroup.rois
    for s=roi.scanfields
        if isprop(s,'rect')
            if isempty(width)
                width = s.rect(3);
            elseif s.rect(3) ~= width
                dims = s.rect;
                dw = width - dims(3);
                dims(1) = dims(1) - dw/2;
                dims(3) = width;
                s.rect = dims;
            end
        end
    end
end

end


%--------------------------------------------------------------------------%
% sameWidth.m                                                              %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
