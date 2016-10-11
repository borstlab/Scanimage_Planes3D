function scim_PFileClear(verbose)
%% function scim_PFileClear(verbose)
% Clears pcode throughout ScanImage installation
%
%% SYNTAX
%   verbose: (OPTIONAL - Default: false) Logical indicating whether to display names of all P files removed
%
%% CREDITS
%   Created 3/31/10, by Vijay Iyer
%% **************************************************

if nargin < 1 || isempty(verbose)
    verbose = false;
end

siPath = fileparts(which('scanimage'));
removePFiles(siPath);
rehash path;

    function removePFiles(targetPath)
        pathStruct = what(targetPath);
        pFiles = pathStruct.p;
        if ~isempty(pFiles)
            cellfun(@(x)deleteNotify(x),pFiles);
        end               
        dirStruct = dir(targetPath);
        subDirs = dirStruct(arrayfun(@(x)x.isdir && ~ismember(x.name,{'.' '..'}),dirStruct));
        for i=1:length(subDirs)
            removePFiles(fullfile(targetPath,subDirs(i).name));
        end
        
    end

    function deleteNotify(pFileName)
        fullFileName = which(pFileName);
        delete(fullFileName);
        if verbose
            fprintf(1,'Removed P file: %s\n',fullFileName);
        end
    end
end

