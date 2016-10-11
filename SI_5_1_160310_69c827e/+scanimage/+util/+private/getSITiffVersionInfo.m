function [verInfo] = getSITiffVersionInfo(frameString)
%   Analize a tiff-header frame-string to determine the scanimage version it came from
%   The tags provided by the ScanImage header are insufficient to keep track of released 
%   versions of ScanImage, hence we'll provide a structure called verInfo to help us simplify
%   version detection

    verInfo = struct();
    verInfo.infoFound = false;

    %TODO: Make sure this works for the case where this property doesn't exist?
    try
        verInfo.SI_MAJOR = num2str(scanimage.util.private.getHeaderProperty(frameString,'scanimage.SI.VERSION_MAJOR'));
        verInfo.SI_MINOR = num2str(scanimage.util.private.getHeaderProperty(frameString,'scanimage.SI.VERSION_MINOR'));
        verInfo.TIFF_FORMAT_VERSION = scanimage.util.private.getHeaderProperty(frameString,'scanimage.SI.TIFF_FORMAT_VERSION');
        
        verInfo.infoFound = true;
    catch
        most.idioms.dispError('Cannot find SI and/or Tiff version properties in Tiff header.\n');
        return;
    end

    %% Determine if the scanner is linear or resonant

    % The following call assumes later versions (from October onwards?) and the string is either 'Resonant' or 'Linear'
    verInfo.ImagingSystemType = scanimage.util.private.getHeaderProperty(frameString,'scanimage.SI.hScan2D.scannerType');
    if isempty(verInfo.ImagingSystemType)
        % If the call above was unsuccessful, the following will use the legacy approach to obtain the scanner type
        verInfo.ImagingSystemType = scanimage.util.private.getHeaderProperty(frameString,'scanimage.SI.imagingSystem');
        if isempty(verInfo.ImagingSystemType)
            most.idioms.dispError('Could not find imaging system! Assuming Resonant. Please contact support.');
            verInfo.ImagingSystemType = 'Resonant';
        end
    end
end


%--------------------------------------------------------------------------%
% getSITiffVersionInfo.m                                                   %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
