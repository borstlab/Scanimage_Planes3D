function filenames=formatDirectoryForContents(directory,extension)
% FORMATDIRECTORYFORCONTENTS   - Outputs H1 Line from m-files in directory as a character array.
% 	FORMATDIRECTORYFORCONTENTS(directory,extension) is used to
% 	automatically generate the strings used for the Content.m file for a
% 	directory.
%
% 	See also FINDFILEWITHNAME, PROGMANAGER

if nargin < 1
    directory=pwd;
    extension='m';
elseif nargin < 2
    extension='m';
end

filenames={};
allfiles = dir([directory '\*.' extension]);
if isempty(allfiles)
    return
end
[filenames{1:length(allfiles),1}] = deal(allfiles.name);
inarray=zeros(1,length(filenames));
removeindex=[];
for counter=1:length(filenames)
    if strcmpi(filenames{counter},'contents.m')
        removeindex=[removeindex counter];
    else
%         filenames{counter}= lower(filenames{counter}(1:end-2));
        temphelp=help(filenames{counter});
        periods=findstr(temphelp,'.');
        if ~isempty(periods)
            filenames{counter}=temphelp(1:periods(1));
        end
    end
end
filenames(removeindex)=[];
filenames=char(filenames);
