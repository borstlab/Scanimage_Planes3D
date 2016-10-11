function safeDeleteObj(obj)
    %SAFEDELETEOBJ Checks if the object handle is valid and deletes it if so.
    % Returns true if object was valid.
    
    %VI20141219: Don't see a reason to report any error if object isn't extant/valid    
    if most.idioms.isValidObj(obj)
        if isa(obj,'timer')
            stop(obj);
        end
        delete(obj);
    end
    
    %     try
    %         tf = false;
    %         if most.idioms.isValidObj(obj)
    %             delete(obj);
    %             tf = true;
    %         end
    %     catch ME
    %         most.idioms.reportError(ME);
    %         tf = [];
    %     end
end


%--------------------------------------------------------------------------%
% safeDeleteObj.m                                                          %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
