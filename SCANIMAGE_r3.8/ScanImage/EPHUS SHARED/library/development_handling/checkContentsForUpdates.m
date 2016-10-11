function fileout=checkContentsForUpdates(directory,extension)
% CHECKCONTENTSFORUPDATES   - Outputs H1 lines for m files in directory that are not in Contents.m file.
% 	CHECKCONTENTSFORUPDATES(directory,extension) is used to
% 	automatically generate the strings used for the Content.m file for a
% 	directory, filtering ofr thos that are not already in the Contents.m
% 	file.
%
% 	See also FORMATDIRECTORYFORCONTENTS

if nargin < 1
    directory=pwd;
    extension='m';
elseif nargin < 2
    extension='m';
end

filenames={};
fileout={};
allfiles = dir([directory '\*.' extension]);
if isempty(allfiles)
    return
end
[filenames{1:length(allfiles),1}] = deal(allfiles.name);
if exist(fullfile(directory,'Contents.m'))==2
    contentsfile=textread('Contents.m','%s');
    filenames(strcmpi(filenames,'Contents.m'))=[];
    for filecounter=1:length(filenames)
        if ~any(strcmpi(filenames{filecounter}(1:end-2),contentsfile))
            helpout=help(filenames{filecounter}(1:end-2));
            periods=findstr(helpout,'.');
            if ~isempty(periods)
                helpout(1:periods(1));
                if ~isempty(strfind(helpout(1:periods(1)),'-'))
                    fileout{end+1}=helpout(1:periods(1));
                end
            end
        end
    end
end
fileout=char(fileout);