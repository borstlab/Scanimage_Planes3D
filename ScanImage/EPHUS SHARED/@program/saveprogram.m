function saveprogram(obj,filename)
%SAVEPROGRAM   - @program save method.
%  SAVEPROGRAM(obj) will save a program object to a MAT file stored on
%  disk.  The default file extension is .PMP (Program Manager Program).  
%  With no input arguments, the default is to open the file browser.
%
%  SAVEPROGRAM(obj,filename) will save obj into the filename given.
%
%  See also LOADPROGRAM

% Select file from disk if none is supplied.
if nargin == 1 | isempty(filename)
    [fname,pname]=uiputfile('default.pmp','Save ' obj.program_name ' As..');
    if isnumeric(fname)
        return
    end
    filename=fullfile(pname,fname);
end
save(filename,'obj', '-mat');
