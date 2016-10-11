function scanimage(varargin)
    %SCANIMAGE Starts ScanImage application and its GUI(s)
    
    mdf = '';
    usr = '';
    
    for i = 1:nargin
        [~,~,ext] = fileparts(varargin{i});
        
        if strcmp(ext, '.m')
            mdf = varargin{i};
            assert(logical(exist(mdf,'file')), 'Specified machine data file not found.');
        end

        if strcmp(ext, '.usr')
            usr = varargin{i};
            if ~strcmp(usr, '.usr')
                assert(logical(exist(usr,'file')), 'Specified usr file not found.');
            end
        end
    end

    if evalin('base','~exist(''hSI'')')
        
        if isempty(mdf) || isempty(usr)
            [mdf,usr,runSI] = scanimage.StartupConfig.doModalConfigPrompt(mdf,usr);
            if ~runSI
                return;
            end
        end
        
        try
            hSI = scanimage.SI(mdf);
            assignin('base','hSI',hSI); % assign object in base as soon as it is constructed
            hSI.initialize();
            
            hSICtl = scanimage.SIController(hSI);
            assignin('base','hSICtl',hSI.hController{1}); % assign object in base as soon as it is constructed
            hSICtl.initialize(usr);

        catch ME
            if exist('hSI', 'var')
                most.idioms.safeDeleteObj(hSI);
            end

            evalin('base','clear hSI hSICtl MachineDataFile');
            
            if strcmp(ME.message, 'MachineDateFile: Operation canceled.')
                most.idioms.warn(ME.message);
            else
                ME.rethrow;
            end
        end
    else
        most.idioms.warn('ScanImage is already running.');
        evalin('base','hSICtl.raiseAllGUIs')
    end
end


%--------------------------------------------------------------------------%
% scanimage.m                                                              %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
