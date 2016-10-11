%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  opens and interprets and initialization file 
%%
%%  Created - Bernardo Sabatini 1/21/01
%%
%%  Changed:
%%          TPMOD_1_1/28/04: Updated to use initGUIsFromCellArray function
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [out,fname,pname,ext]=initGUIs(fileName)

out=0;
fname='';
pname='';
ext='';
% open file and read in by line ignoring comments
fid=fopen(fileName, 'r');
if fid==-1
	disp(['initGUIs: Error: Unable to open file ' fileName ]);
	return
else
	out=1;
	[fullName, per, mf]=fopen(fid);
	fclose(fid);
	[pname, fname, ext]=fileparts(fullName);
end
file = textread(fullName,'%s', 'commentstyle', 'matlab', 'delimiter', '\n');
% start TPMOD_1_1/28/04  Removed Many Lines of Code
initGUIsFromCellArray(file);
% end TPMOD_1_1/28/04  Removed Many Lines of Code

