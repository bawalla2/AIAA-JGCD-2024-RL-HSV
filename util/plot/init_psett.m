function psett_master = init_psett
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE PLOT FORMATTING SETTINGS
%
% Brent Wallace  
%
% 2023-02-16
%
% This function initializes various plot formatting settings to ensure
% consistent plot formatting. For further information on how plots are
% formatted, see 'plot_format.m'.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% ***********************
%
% WARNING SUPPRESSIONS
%  

% Improper compiler syntax
warning('off', 'MATLAB:handle_graphics:exceptions:SceneNode')


% ***********************
%
% PLOT FORMATTING -- MISC.
%  

% Plot formatting -- line width
psett_master.linewidth = 2;

% Plot formatting -- line width (wide)
psett_master.linewidthwide = 4;
psett_master.psett_linewidthwide = ...
    {'LineWidth', psett_master.linewidthwide};

% Plot formatting -- dashed line
psett_master.psett_dash = {'LineStyle', '--'};

% Plot formatting -- dotted line
psett_master.psett_dot = {'LineStyle', ':'};

% Plot formatting -- dot-dash line
psett_master.psett_dotdash = {'LineStyle', '-.'};

% Plot formatting -- bring line to front command
psett_master.psett_bringtofront = {'bringtofront', ''};

% Plot formatting -- bring line up one level command
psett_master.psett_bringup = {'bringup', ''};

% Plot formatting -- bring line down one level command
psett_master.psett_bringdown = {'bringdown', ''};

% ***********************
%
% PLOT FORMATTING -- COLORS (RGB TRIPLES)
%  

% Plot formatting -- colors
% https://www.mathworks.com/help/matlab/creating_plots/specify-plot-colors.html
colors.matlabblue = [0 0.4470 0.7410];
colors.matlaborange = [0.8500 0.3250 0.0980];
colors.matlabyellow = [0.9290 0.6940 0.1250];
colors.matlabpurple = [0.4940 0.1840 0.5560];
colors.matlabgreen = [0.4660 0.6740 0.1880];
colors.matlablightblue = [0.3010 0.7450 0.9330];
colors.matlabmaroon = [0.6350 0.0780 0.1840];

colors.red = [1 0 0];
colors.green = [0 1 0];
colors.blue = [0 0 1];
colors.cyan = [0 1 1];
colors.magenta = [1 0 1];
colors.yellow = [1 1 0];
colors.black = [0 0 0];
colors.white = [1 1 1];
colors.gray = [0.7 0.7 0.7];
psett_master.colors = colors;

% Plot formatting -- color commands
color_cmds.matlabblue = {'Color', colors.matlabblue};
color_cmds.matlaborange = {'Color', colors.matlaborange};
color_cmds.matlabyellow = {'Color', colors.matlabyellow};
color_cmds.matlabpurple = {'Color', colors.matlabpurple};
color_cmds.matlabgreen = {'Color', colors.matlabgreen};
color_cmds.matlablightblue = {'Color', colors.matlablightblue};
color_cmds.matlabmaroon = {'Color', colors.matlabmaroon};


color_cmds.red = {'Color', colors.red};
color_cmds.green = {'Color', colors.green};
color_cmds.blue = {'Color', colors.blue};
color_cmds.cyan = {'Color', colors.cyan};
color_cmds.magenta = {'Color', colors.magenta};
color_cmds.yellow = {'Color', colors.yellow};
color_cmds.black = {'Color', colors.black};
color_cmds.white = {'Color', colors.white};
color_cmds.gray = {'Color', colors.gray};

% Number of colors
ncolors = length(fieldnames(color_cmds));
psett_master.ncolors = ncolors;

% Settings for individual commands
psett_master.psett_matlabblue = color_cmds.matlabblue;
psett_master.psett_matlaborange = color_cmds.matlaborange;
psett_master.psett_matlabyellow = color_cmds.matlabyellow;
psett_master.psett_matlabpurple = color_cmds.matlabpurple;
psett_master.psett_matlabgreen = color_cmds.matlabgreen;
psett_master.psett_matlablightblue = color_cmds.matlablightblue;
psett_master.psett_matlabmaroon = color_cmds.matlabmaroon;
psett_master.psett_black = color_cmds.black;
psett_master.psett_gray = color_cmds.gray;

% ***********************
%
% TIME DOMAIN PLOTS
%   

% Plot formatting -- time label
psett_master.tlabel = 'Time $t$ (s)';

% ***********************
%
% FREQUENCY RESPONSE PLOTS
%   

% Plot formatting -- frequency label
psett_master.wlabel = 'Frequency (rad/s)';

% Plot formatting -- magnitude label
psett_master.maglabel = 'Magnitude (dB)';

% Plot formatting -- phase label
psett_master.phlabel = 'Phase (deg)';

% ***********************
%
% SURFACE PLOT SETTINGS
%        
  

% Surface plot face transparency
psett_master.facealpha = 0.5;

% Surface plot edge transparency
psett_master.edgealpha = 0.75;

% ***********************
%
% SURFACE PLOT SETTINGS -- GRAY ENTRY
%        

% Surface plot face color -- "highlighted" plot entry
psett_master.facecolor_hl = 'k';

% Surface plot face transparency -- "highlighted" plot entry
psett_master.facealpha_hl = 0.1;

% Surface plot edge transparency -- "highlighted" plot entry
psett_master.edgealpha_hl = 0.5;  


psett_master.psett_matlabblue = {'Color', colors.matlabblue};
psett_master.psett_matlaborange = {'Color', colors.matlaborange};
psett_master.psett_matlabyellow = {'Color', colors.matlabyellow};
psett_master.psett_matlabpurple = {'Color', colors.matlabpurple};
psett_master.psett_matlabgreen = {'Color', colors.matlabgreen};
psett_master.psett_matlablightblue = {'Color', colors.matlablightblue};
psett_master.psett_matlabmaroon = {'Color', colors.matlabmaroon};
psett_master.psett_black = {'Color', colors.black};
psett_master.psett_gray = {'Color', colors.gray};

% ***********************
%
% PLOT SETTINGS -- LIST OF COLOR COMMANDS
%  

psett_master.color_sett_cell = {   color_cmds.matlabblue 
                            color_cmds.matlaborange  
                            color_cmds.matlabyellow
                            color_cmds.matlabpurple
                            color_cmds.matlabgreen
                            color_cmds.matlablightblue
                            color_cmds.matlabmaroon
                            %
                            color_cmds.red 
                            color_cmds.green 
                            color_cmds.blue 
                            color_cmds.cyan 
                            color_cmds.magenta  
                            color_cmds.yellow 
                            color_cmds.black
                            color_cmds.gray
                            };



% ***********************
%
% CONTOUR PLOT SETTINGS
%     

% Contour edge transparency
psett_master.contour.edgealpha = 0.1;