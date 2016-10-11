function updateColormap(h)
%% UPDATECOLORMAP Handle updates to colormap popup menu(s)

	global state gh;
	
	% 	if nargin < 3
	% 		% called via INI callback mechanism
	% 		dropdownIndex = state.internal.colormapSelected;
	% 		set(h,'Value',dropdownIndex);
	% 		updateGuIByGlobal('gh.imageGUI.imageColormap');
	% 	else
	% 		% called via the GUI callback
	% 		dropdownIndex = get(handles.imageColormap,'Value');
	% 		state.internal.colormapSelected = dropdownIndex;
	% 	end	
	
	prettyNamesMap = containers.Map({'Gray' 'Gray - High Sat.' 'Gray - Low Sat.' 'Gray - Both Sat.' 'Jet'}, ...
									{'gray' 'grayHighSat' 'grayLowSat' 'grayBothSat' 'jet'});
	dropdownColormaps = get(h,'String');
	dropdownIndex = get(h,'Value');
	selectedColormap = dropdownColormaps{dropdownIndex};

	for i=1:state.init.maximumNumberOfInputChannels
		switch selectedColormap
			case 'Custom'
				set(gh.channelGUI.(['textColormap' num2str(i)]),'Enable','on');
				
			case 'R/G/Gray/Gray'
				set(gh.channelGUI.(['textColormap' num2str(i)]),'Enable','off');
				if i == 1
					updateGUIByGlobal(sprintf('state.internal.figureColormap%d',i),'Value','scim_colorMap(''red'',8,5)','Callback',1);
					set(state.internal.MaxFigure(i),'Colormap',scim_colorMap('red'));
				elseif i == 2
					 updateGUIByGlobal(sprintf('state.internal.figureColormap%d',i),'Value','scim_colorMap(''green'',8,5)','Callback',1);
					 set(state.internal.MaxFigure(i),'Colormap',scim_colorMap('green'));
				else
					 updateGUIByGlobal(sprintf('state.internal.figureColormap%d',i),'Value','scim_colorMap(''gray'',8,5)','Callback',1);
					 set(state.internal.MaxFigure(i),'Colormap',scim_colorMap('gray'));
				end
				
			case 'G/R/Gray/Gray'
				set(gh.channelGUI.(['textColormap' num2str(i)]),'Enable','off');
				if i == 1
					updateGUIByGlobal(sprintf('state.internal.figureColormap%d',i),'Value','scim_colorMap(''green'',8,5)','Callback',1);
					set(state.internal.MaxFigure(i),'Colormap',scim_colorMap('green'));
				elseif i == 2
					 updateGUIByGlobal(sprintf('state.internal.figureColormap%d',i),'Value','scim_colorMap(''red'',8,5)','Callback',1);
					 set(state.internal.MaxFigure(i),'Colormap',scim_colorMap('red'));
				else
					 updateGUIByGlobal(sprintf('state.internal.figureColormap%d',i),'Value','scim_colorMap(''gray'',8,5)','Callback',1);
					 set(state.internal.MaxFigure(i),'Colormap',scim_colorMap('gray'));
				end
				
			otherwise
				set(gh.channelGUI.(['textColormap' num2str(i)]),'Enable','off');
				
				% update the 'colormap' textfield in channelGUI
				updateGUIByGlobal(sprintf('state.internal.figureColormap%d',i),'Value',['scim_colorMap(''' prettyNamesMap(selectedColormap) ''',8,5)'],'Callback',1);

				% manually set the colormap of the maxprojection figure
				if i <= length(state.internal.MaxFigure)
					set(state.internal.MaxFigure(i),'Colormap',scim_colorMap(prettyNamesMap(selectedColormap)));
				end
		end
	end

end

