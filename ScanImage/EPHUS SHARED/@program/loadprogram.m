function [obj_out,filename] = loadprogram(obj,filename)
%LOADPROGRAM   - @program load method.
%  LOADPROGRAM(obj) will load a program object from a MAT file stored on
%  disk.  The possible file extensions are .MAT and .PMP, of which the .PMP
%  file is the default one saved using the SAVEPROGRAM function.  With no
%  input arguments, the default is to open the file browser.
%
%  LOADPROGRAM(obj,filename) will load the file specified by filename.
%
%  Note that if the mat file is used, and there are multiple variables on
%  teh file, only the first instance of the program will be used.  If no
%  program object exists on the file, it will return an empty [].
%
%  See also SAVEPROGRAM

% Select file from disk if none is supplied.
obj_out=[];
filename='';
if nargin == 1 | isempty(filename)
    extensionlist={'*.pmp';'*.mat'};
    [fname,pname,index]=uigetfile(extensionlist,'Select Program to Load...');
    if isnumeric(fname)
        return
    end
    filename=fullfile(pname,fname);
end
temp=struct2cell(load(filename,'-mat'));    % Read file and convert to cell array.
temp=temp(cellfun('isclass', temp, 'program')); % Filter for only program objects.
if isempty(temp)
    return
else
    obj_out=temp{1};
    obj_out.filename=filename;
end


