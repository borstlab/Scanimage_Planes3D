classdef RoiTree < matlab.mixin.Copyable
    %% Parent class of RoiGroup, Roi and ScanField
    % only purpose is to overload the addlistener method to prevent reference leaks
    methods
        function lh = addlistener(varargin)
            lh = most.idioms.addweaklistener(varargin{:});
        end
    end
end

%--------------------------------------------------------------------------%
% RoiTree.m                                                                %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
