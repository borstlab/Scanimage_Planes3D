function [frameStringCell, numImg] = getStringCellFromTiff(tifObj)
% Returns a cell array of strings for each TIFF header
% If the number of images is desired one can call numel on frameStringCell or use the 
% second argument (the latter approach is preferrable)
%
    numImg = 0;

    % Before anything else, see if the tiff file has any image-data
    try
        %Parse SI from the first frame
        numImg = 1;
        while ~tifObj.lastDirectory()
            tifObj.nextDirectory();
            numImg = numImg + 1;
        end
    catch
        warning('The tiff file may be corrupt.')
        % numImg will have the last valid value, so we can keep going and 
        % deliver as much data as we can
    end
    tifObj.setDirectory(1);

    %Make sure the tiff file's ImageDescription didn't go over the limit set in 
    %Acquisition.m:LOG_TIFF_HEADER_EXPANSION
    try
        if ~isempty(strfind(tifObj.getTag('ImageDescription'), '<output truncated>'))
            most.idioms.warn('Corrupt header data');
            return;
        end
    catch
        most.idioms.warn('Corrupt or incomplete tiff header');
        return
    end

    frameStringCell = cell(1,numImg);

    for idxImg = 1:numImg
        frameStringCell{1,idxImg} = tifObj.getTag('ImageDescription');
        if idxImg == numImg; break;end  % Handles last case
        tifObj.nextDirectory();
    end
end


%--------------------------------------------------------------------------%
% getStringCellFromTiff.m                                                  %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
