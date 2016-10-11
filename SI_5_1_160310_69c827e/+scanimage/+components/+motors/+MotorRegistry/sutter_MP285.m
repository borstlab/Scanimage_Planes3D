function s = sutter_MP285

s = struct();
s.Names = {'mp285' 'sutter.mp285' 'sutter.MP285'};
s.Class = 'dabs.sutter.MP285';
s.SubType = '';
s.TwoStep.Enable = true; 
s.TwoStep.FastLSCPropVals = struct('resolutionMode','coarse');
s.TwoStep.SlowLSCPropVals = struct('resolutionMode','fine');
s.TwoStep.InitSlowLSCProps = true;
s.SafeReset = true;
s.NumDimensionsPreset = true;


%--------------------------------------------------------------------------%
% sutter_MP285.m                                                           %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
