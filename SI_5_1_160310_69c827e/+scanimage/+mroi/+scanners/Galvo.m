classdef Galvo < handle

    properties
        sampleRateHz;
        fullAngleDegrees;
        voltsPerDegree;
        flytoTimeSeconds;
        flybackTimeSeconds;
        parkAngleDegrees;
    end

    properties(SetObservable)
        pixelCount;
    end
    
    properties(Dependent)
        degreesPerPixel;
    end

    methods(Static)
        function obj = default
            obj=scanimage.mroi.scanners.Galvo(27,20/27,27/128,1e-3,-27/2,200000);
        end
    end

    methods
        % See Note (1)
        function obj=Galvo(fullAngleDegrees,...
                           voltsPerDegree,...
                           flytoTimeSeconds,...
                           flybackTimeSeconds,...
                           parkAngleDegrees,...
                           sampleRateHz)
            obj.fullAngleDegrees   = fullAngleDegrees;
            obj.voltsPerDegree     = voltsPerDegree;
            obj.flytoTimeSeconds   = flytoTimeSeconds;
            obj.flybackTimeSeconds = flybackTimeSeconds;
            obj.parkAngleDegrees   = parkAngleDegrees;
            obj.sampleRateHz       = sampleRateHz;
        end

        function set.pixelCount(obj,val)
            obj.pixelCount=floor(val);
        end

        function val=get.degreesPerPixel(obj)
            val=obj.fullAngleDegrees/obj.pixelCount;
        end
    end
end

%% NOTES
%{

1. This constructor has too many arguments.
   
   Property-Value pairs should be used instead to make calling code more
   legible/flexible.

   This is just cosmetic though, so this can wait till later.

%}


%--------------------------------------------------------------------------%
% Galvo.m                                                                  %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
