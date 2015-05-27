function out=openini(fileName)
%% function out=openini(fileName)
%   Opens Scanimage initialization file (i.e. 'standard.ini' file)
%
%% NOTES
%   Function now opens two files: 1) the specified file, which contains the Rig configuration info, and 2) the fixed 'internal.ini' file that now is part of the codebase -- Vijay Iyer 3/29/09
%
%% CHANGES
%   VI022608A Vijay Iyer 2/26/08 -- Display full path of the opened standard.ini file onto command line
%   VI030508A Vijay Iyer 3/5/08 -- Only use 'which' to find full path of .ini file in cases where its location was determined from the path ...
%        (in other cases, the fully specified filename should be given as an argument to this function)
%   VI032909A Vijay Iyer 3/29/09 -- Open internal.ini file in addition to the specified .ini file 
%   VI051911A Vijay Iyer 5/19/11 -- Load new defaults.ini file /before/ the standard INI file, to allow former to specify defaults in case not present in latter
%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

out=1;

[fid, message]=fopen(fileName);
if fid<0
    disp(['openini: Error opening ' fileName ': ' message]);
    out=1;
    return
end
[fileName,permission, machineormat] = fopen(fid);
fclose(fid);

%%%VI051911A%%%%%%%%%%%%
scanimagePath = fileparts(which('scanimage'));
defaultIniFile = fullfile(scanimagePath,'init_files','defaults.ini');
initGUIs(defaultIniFile);
%%%%%%%%%%%%%%%%%%%%%%%%

%VI030508A - get full path of .ini file for case where it was just found on the path
if ~isempty(which(fileName)) 
    fileName = which(fileName);
end
disp(['*** CURRENT INI FILE = ' fileName ' ***']); %VI022608A/VI030508A
initGUIs(fileName);

[path,name] = fileparts(fileName);

global state;
state.iniName=name;
state.iniPath=path;

%%%VI032909A%%%%%%%%%%%%
scanimagePath = fileparts(which('scanimage'));
internalIniFile = fullfile(scanimagePath,'init_files','internal.ini');
initGUIs(internalIniFile);
%%%%%%%%%%%%%%%%%%%%%%%%