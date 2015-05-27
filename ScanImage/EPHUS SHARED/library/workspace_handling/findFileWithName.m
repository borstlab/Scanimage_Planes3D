function [filenames]=findFileWithName(string,extension,directory)
% FINDFILEWITHNAME   - Search for files filtering by name and extension.
%   FINDFILEWITHNAME searches through the names of all the files in the
%   directory looking for possible string matches.
%
%   Can also filter by the extension to only search for files with a given
%   extension.
%
%   See also FINDWITHTAG, FINDWITHNAME

filenames={};
if nargin < 2
	directory=pwd;
	extension='*';
elseif  nargin < 3
	directory=pwd;
end

allfiles = dir([directory '\*.' extension]);
if isempty(allfiles)
	return
end
[filenames{1:length(allfiles)}] = deal(allfiles.name);
inarray=zeros(1,length(filenames));
for counter=1:length(filenames)
	inarray(counter)=~isempty(findstr(filenames{counter},string));
end
filenames=filenames(logical(inarray));

