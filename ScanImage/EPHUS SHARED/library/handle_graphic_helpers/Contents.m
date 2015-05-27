% Svoboda Lab Library - Handle Graphic Helper Functions
% 
% These function will help use MATLAB's handle graphics more efficiently.  
% They are sorted according to the object type they act upon.
% 
% General (Works with most handles)
% 	FINDWITHTAG				- Locates handles of any objects containing a specified tag.
% 	GETPARENT   - Returns parent of handle passed to it.                                          
% 	GETUSERDATAFIELD   - Parses UserData of object as structure.                                  
% 	HASUSERDATAFIELD   - Parses UserData of object as structure.     
%
% Figures
% 	FIGSHIFT    			- Displays Images selightly offset from each other.
% 	FILLSCREEN				- Maximizes figure window.
% 	SPLAYFIGS				- Distributes figures accross.
% 	SHOWMETHEFIGS			- Cycle between multiple figures.
% 
% Axes
% 	COLLAPSEAXES   - takes multiple axes on a figure, and moves all graphics objects to one axes.
% 	EXPANDAXES   - takes an axes handle and moves all graphics objects to different axes.        
% 	GETDATAFROMHANDLE		- Gets data from handles as numerics.
% 	RESHUFFLEAXISHANDLES	- Reorders Axes children.
% 	SPLAYAXISTILE			- Distributes axes in a grid.
% 	SPLAYAXISVERTICAL		- Distributes axes in a column.
% 	SPLAYAXISHORIZONTAL		- Distributes axes in a row.
% 	RESCALEAXIS   - customized scaling of axes based on data on axis.                             
% 	SCALEXDATA   - Changes 'XData' for current axes.                                             

                              
