function disableAll(hFigOrPanel)
%DISABLEALL Disables all controls, as possible, on a figure or panel


arrayfun(@(h)disableIfPossible(h),findall(hFigOrPanel));

    function disableIfPossible(handle)
        if isprop(handle,'Enable')
            set(handle,'Enable','off')
        end
    end

end



%--------------------------------------------------------------------------%
% disableAll.m                                                             %
% Copyright � 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
