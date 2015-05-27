% generateDocumentation - Scan entire package and generate documentation for all M-files.
%
% SYNTAX
%  generateDocumentation(codeRoot, docRoot)
%   codeRoot - The root path for the package's directory tree.
%              All M-files in the codeRoot and its subdirectories will be published.
%   docRoot - The root path for the generated documentation's destination.
%             The directory structure of codeRoot will be mirrored within docRoot.
%
% Created - Timothy O'Connor 4/23/09
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2009
function generateDocumentation(codeRoot, docRoot)

contents = dir(codeRoot);
if exist(docRoot, 'dir') ~= 7
    [path, name] = fileparts(docRoot);
    if exist(path, 'dir') ~= 7
        error('Path does not exist: ''%s''', docRoot);
    end
    mkdir(path, name);
end

%The documentation is wrong, it says to use 'true' or 'false', when boolean options should be numbers.
options.evalCode = 0;
options.showCode = 0;
options.catchError = 1;

for i = 1 : 4 %length(contents)
    if any(strcmp(contents(i).name, {'.', '..'}))
        continue;
    elseif contents(i).isdir
        if exist(fullfile(docRoot, contents(i).name), 'dir') ~= 7
            fprintf(1, 'Making documentation directory: ''%s''\n', fullfile(docRoot, contents(i).name));
            mkdir(docRoot, contents(i).name);
        end
        generateDocumentation(fullfile(codeRoot, contents(i).name), fullfile(docRoot, contents(i).name));
    else
        if endsWithIgnoreCase(contents(i).name, '.m')
            try
                fprintf(1, 'Publishing %s to %s\n', fullfile(codeRoot, contents(i).name), docRoot);
                options.outputDir = docRoot;
                publish(contents(i).name(1 : end - 2), options);
            catch
                fprintf(2, 'Failed to publish documentation for ''%s'': %s\n', fullfile(codeRoot, contents(i).name), lasterr);
            end
        end
        return;
    end
end

return;