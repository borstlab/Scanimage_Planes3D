function splayFigs(figs,varargin)
% SPLAYFIGS   - Displays figures in a grid.
%   SPLAYFIGS displays all the figures specified by the array of figure handles figs
%   in a grid whose size is closest to a square as possible.
%   with parameters specifed by varargin.
%
%   varargin are parameter (string) value (numeric) pairs that specify:
%
% 	    gapH: fractional horizontal space between figures (0 to 1)
%
% 	    startH: fractional horizontal space between left edge of figure and left corner of
% 	    1st figures.
%
% 	    heightH: fractional horizontal space occupied by all figures.
%
% 	    gapH: fractional horizontal space between figures (0 to 1)
%
% 	    startH: fractional horizontal space between left edge of figure and left corner of
% 	    1st figures.
%
% 	    heightH: fractional horizontal space occupied by all figures.
%
%	See also SPLAYAXISHORIZONTAL, SPLAYAXISVERTICAL, SPLAYAXISTILE

if nargin < 1
    %Get fig handles
    figs=findobj('type','figure','HandleVisibility','on','visible','on');
else
    figs=figs(ishandle(figs));
end

if isempty(figs)
    return
end

nfigs=length(figs);
if nfigs<2
    return
end

set(figs,'Units','normalized');

nSq=ceil(sqrt(nfigs));
nV=ceil(nfigs/nSq);

gapH=.01;
startH=.01;
heightH=.97;
gapV=.08;
startV=.01;
heightV=.92;

% Parse input parameter pairs and rewrite values.
counter=1;
while counter+1 <= length(varargin)
    eval([varargin{counter} '=varargin{counter+1};']);
    counter=counter+2;
end

deltaV=(heightV-gapV*(nV-1))/nV;
deltaH=(heightH-gapH*(nSq-1))/nSq;

for lineCounter=1:nV
    for rowCounter=1:nSq
        place=(lineCounter-1)*nSq+rowCounter;
        if place<=nfigs
            set(figs(nfigs-place+1), 'Position', [startH+(rowCounter-1)*(deltaH+gapH) startV+(lineCounter-1)*(deltaV+gapV)  deltaH deltaV ]);
        end
    end
end
