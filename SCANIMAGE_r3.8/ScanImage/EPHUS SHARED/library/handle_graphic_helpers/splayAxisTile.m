function splayAxisTile(f,varargin)
% SPLAYAXISTILE   - Displays axes in a grid (like subplot(M,N)).
%   SPLAYAXISTILE displays all the axes that are children of the 
%   figure handle f in a grid whose size is closest to a square as possible.
%   with parameters specifed by varargin.
%
%   varargin are parameter (string) value (numeric) pairs that specify:
%
% 	    gapH: fractional horizontal space between axes (0 to 1)
%
% 	    startH: fractional horizontal space between left edge of figure and left corner of
% 	    1st axes.
%
% 	    heightH: fractional horizontal space occupied by all axes.
%
% 	    gapH: fractional horizontal space between axes (0 to 1)
%
% 	    startH: fractional horizontal space between left edge of figure and left corner of
% 	    1st axes.
%
% 	    heightH: fractional horizontal space occupied by all axes.
%
%	See also SPLAYAXISHORIZONTAL, SPLAYAXISVERTICAL, SPLAYFIGS, SUBPLOT

if nargin == 0
    f=gcf;  
else
    if mod(length(varargin),2)==1   % passed a f...
        varargin=[{f} varargin];
        f=gcf;   
   end    
end

allAx=findobj(f, 'Type', 'axes', 'Box', 'off');
nAx=length(allAx);
if nAx==0
	return
end

nSq=ceil(sqrt(nAx));
nV=ceil(nAx/nSq);

gapH=.1;
startH=.1;
heightH=.81;
gapV=.15;
startV=.1;
heightV=.81;

% Parse input parameter pairs and rewrite values.
counter=1;
while counter+1 <= length(varargin)
    eval([varargin{counter} '=' num2str(varargin{counter+1}) ';']);
    counter=counter+2;
end

deltaV=(heightV-gapV*(nV-1))/nV;
deltaH=(heightH-gapH*(nSq-1))/nSq;

for lineCounter=1:nV
	for rowCounter=1:nSq
		place=(lineCounter-1)*nSq+rowCounter;
		if place<=nAx
			set(allAx(nAx-place+1), 'Position', [startH+(rowCounter-1)*(deltaH+gapH) startV+(lineCounter-1)*(deltaV+gapV)  deltaH deltaV ]);
		end
	end
end
