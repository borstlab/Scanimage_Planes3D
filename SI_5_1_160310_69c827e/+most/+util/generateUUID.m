function uuid = generateUUID()
    % generates a universally unique identifier
    % example: fc4d7383-839c-430b-9dc6-12757160b97f
    error(javachk('jvm'));
    uuid = char(java.util.UUID.randomUUID);
end


%--------------------------------------------------------------------------%
% generateUUID.m                                                           %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
