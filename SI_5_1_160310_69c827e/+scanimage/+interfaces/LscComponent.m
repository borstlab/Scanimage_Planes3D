classdef LscComponent < scanimage.interfaces.Class
    
    %% Abstract properties
    properties (Abstract, Constant, Hidden)
        motorDimensions; %= 'xyz'
        motorRegistryInfo; %= struct('TwoStep', struct('Enable', false))
    end
    
    properties (Abstract, SetAccess=private)
        lscInitSuccessful;
    end
    
    
    %% Constants
    properties (Constant, Hidden)
       lscDontDelete = true; % This is to prevent StageController from deleting the object
    end
    
end


%--------------------------------------------------------------------------%
% LscComponent.m                                                           %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
