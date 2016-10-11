function res = xcorr2(A, B)
  if nargin == 1
    res = conv2(A, rot90(A,2));
  elseif nargin == 2
    res = conv2(A, rot90(B,2));
  else
    disp('most.mimics.xcorr2: Unexpected number of arguments');
  end
end


%--------------------------------------------------------------------------%
% xcorr2.m                                                                 %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
