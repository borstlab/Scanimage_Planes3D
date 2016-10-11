function filename = generateNIMaxReport(filename)
if nargin < 1 || isempty(filename)
   [filename,pathname] = uiputfile('.zip','Choose path to save report','NIMaxReport.zip');
   if filename==0;return;end
   filename = fullfile(pathname,filename);
end

[fpath,fname,fext] = fileparts(filename);
if isempty(fpath)
    fpath = pwd;
end
filename = fullfile(fpath,[fname '.zip']); % extension has to be .zip, otherwise NISysCfgGenerateMAXReport will throw error

% initialize NI system configuration session
[~,~,~,experthandle,sessionhandle] = nisyscfgCall('NISysCfgInitializeSession','localhost','','',1033,false,100,libpointer,libpointer);

% create MAX report
nisyscfgCall('NISysCfgGenerateMAXReport',sessionhandle,filename,2,true);

% close session and expert handle
nisyscfgCall('NISysCfgCloseHandle',sessionhandle);
nisyscfgCall('NISysCfgCloseHandle',experthandle);
end

function varargout = nisyscfgCall(funcName,varargin)
if ~libisloaded('nisyscfg')
    loadnisyscfg
end

varargout = cell(nargout,1);
[status,varargout{:}] = calllib('nisyscfg',funcName,varargin{:});
assert(strcmpi(status,'NISysCfg_OK'),'Function call %s in nisyscfg.dll failed with code %s',funcName,status);
end

function loadnisyscfg()
    if libisloaded('nisyscfg');return;end
    
    if strcmp(computer('arch'),'win32')
        % for some reason this function crashes Matlab on Windows 32 bit
        error('generateNIMaxReport currently unsupported on Windows 32 bit');
    end
    
    binarypath = fullfile(fileparts(mfilename('fullpath')),'private',computer('arch'));
    
    oldpath = pwd();
    cd(binarypath);
    
    s = warning();
    warning off MATLAB:loadlibrary:TypeNotFound
    warning off MATLAB:loadlibrary:TypeNotFoundForStructure
    try
        [notfound,warnings] = loadlibrary('nisyscfg', @nisyscfg_proto);
    catch ME
        warning(s);
        cd(oldpath);
        rethrow(ME);
    end
    warning(s);
    cd(oldpath);
end


%--------------------------------------------------------------------------%
% generateNIMaxReport.m                                                    %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
