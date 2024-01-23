function figcount = plot_stat_dyn_sweep(alg_settings_cell,...
                        out_data_cell, group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOT STATIC AND DYNAMIC PROPERTIES AS A FUNCTION OF \nu
%
% Brent Wallace  
%
% 2023-09-20
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZATION
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Unpack plot settings
savefigs = master_settings.savefigs;
if savefigs
    relpath = group_settings.relpath;
end
% dolegend = group_settings.dolegend;


% Initialize figure counter
figcount = group_settings.figcount;

% System names
sysnames = master_settings.sysnames;

% Current system being executed
systag = master_settings.systag;

% Array of sweep type names
sweeptypenames = master_settings.sweeptypenames;

% Sweep type
sweeptype = group_settings.sweeptype;
issweep_IC = group_settings.issweep_IC;
issweep_nu = group_settings.issweep_nu;

% ***********************
%
% SURFACE PLOT SETTINGS -- WEIGHT VALUES
%        

% Master plot formatting settings
psett_master = master_settings.psett_master;

% Plot formatting -- dashed line
psett_dash = psett_master.psett_dash;

% Plot formatting -- colors
colors = psett_master.colors;

% Surface plot face transparency
facealpha = psett_master.facealpha;

% Surface plot edge transparency
edgealpha = psett_master.edgealpha;

% ***********************
%
% SYSTEM PLOT SETTINGS
%

% Extract system plot settings
sys_plot_settings = master_settings.sys_plot_settings;

% Properties of output variables
y_propts_cell = sys_plot_settings.y_propts_cell;

% State trajectory x(t) unit scaling. E.g., if x_1(t) is angular
% displacement but is desired in degrees, declare the field 'sclvec' and
% put sclvec(1) = 180/pi.
x_sclvec = sys_plot_settings.x_sclvec;

% Control signal plot titles (trim)
u_propts_cell = sys_plot_settings.u_propts_cell;

% Shift x_0 sweep limits to trim (=1) or not (=0)
shift_sweep_IC_xe = sys_plot_settings.shift_sweep_IC_xe;

% Ticks for surface plots
nu_ticks_cell = sys_plot_settings.nu_ticks_cell;

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

    

% *************************************************************************
% 
% UNPACK ALGORITHM PARAMETERS -- x_0 SWEEP
%
% *************************************************************************


% ***********************
%
% PULL SYSTEM DATA
%   

% System
sys = master_settings.sys;
m = sys.m;

model_cell = sys.model_cell;
indsmodelplot = master_settings.indsmodelplot;
nummodelsplot = master_settings.nummodelsplot;
indnom = sys.indnom;

% Dimensions of system cell array
numnu1 = master_settings.numnu1;
numnu2 = master_settings.numnu2;
nummodels = master_settings.nummodels;

% Get \nu names if is a \nu sweep
nunames_cell = sys.nunames_cell;


% Get vector of \nu values
nuvec = sys.nuvec;
nuvec_plot = nuvec(indsmodelplot);
nu1vec = sys.nu1vec;
nu2vec = sys.nu2vec;

% Get nominal model
model = get_elt_multidim(model_cell,indnom);
lin = model.lin;
sud = lin.io.sud;

inda = model.inda;
R2D = model.R2D;

% Trim
x_e = model.trimconds.xe;


% LQ data
lq_data_cell = master_settings.lq_data_cell;

% Get nominal LQ data
lq_data_nom = get_elt_multidim(lq_data_cell,indnom);

% Get initial stabilizing controller LQ data
lq_data_0 = master_settings.lq_data_0;

% ***********************
%
% ESTABLISH SWEEP VARIABLES
%  

% Set the plot vectors
vecp1 = nu1vec;
vecp2 = nu2vec;
  
% Create meshgrid of IC values
[X1, X2] = meshgrid(vecp1, vecp2);

% Get verticies of IC sweep
x1min = min(vecp1);
x1max = max(vecp1);
x2min = min(vecp2);
x2max = max(vecp2);
x1vert = [x1min x1max x1max x1min];
x2vert = [x2min x2min x2max x2max];

% Size of sweep in each dimension
ns1 = size(vecp1,1);
ns2 = size(vecp2,1);



% ***********************
%
% PULL RHPP, RHPZ, AND P/Z RATIO FOR EACH \nu
%
% ALSO: TRIM CONTROLS, TRIM AOA
% 

% Poles/zeros for each model
pmat = zeros(numnu1,numnu2);
zmat = zeros(numnu1,numnu2);
pzmat = zeros(numnu1,numnu2);

uemat = zeros(numnu1,numnu2,m);
aemat = zeros(numnu1,numnu2);

% Get p/z data for each model
switch systag
    case sysnames.hsv
        for mcnt1 = 1:numnu1
        for mcnt2 = 1:numnu2
            % Get current plant
            modelij = model_cell{mcnt1,mcnt2};
            Pij = modelij.lin.Pvg;
            % Get eigs
            eigsPij = eig(Pij);
            reeigsPij = real(eigsPij);
            % Get transmission zeros
            tzPij = tzero(Pij);
            retzPij = real(tzPij);
            % Get RHPP, RHPZ
            pij = max(reeigsPij);
            zij = max(retzPij);
            pzij = zij / pij;
            % Get trim u
            ueij = modelij.trimconds.ue;
            % Get trim AOA
            aeij = modelij.trimconds.xe(inda);
            % Store
            pmat(mcnt1,mcnt2) = pij;
            zmat(mcnt1,mcnt2) = zij;
            pzmat(mcnt1,mcnt2) = pzij;
            uemat(mcnt1,mcnt2,:) = sud * ueij;
            aemat(mcnt1,mcnt2) = R2D * aeij;
        end
        end
end

% ***********************
%
% PLOT SETTINGS -- COLORS
%  


% List of colors
facecolor_list = cell(1,1);

facecolor_list{1} = colors.matlabblue;


% ***********************
%
% PLOT SETTINGS -- SET LEGEND DEPENDING ON SWEEP TYPE
%  

switch sweeptype
    % *** IC SWEEP
    case sweeptypenames.IC
        lgd_plot = lgd_nu_plot;
    % *** \nu SWEEP
    case sweeptypenames.nu
        lgd_plot = cell(1,1);
        lgd_plot{1} = 'dEIRL';
end



% ***********************
%
% PLOT SETTINGS -- x, y AXIS LABELS FOR IC SWEEP PLOTS
%  

x0surf_labels = cell(2,1);

for i = 1:2
    x0surf_labels{i} = ['$\nu_{' nunames_cell{i} '}$'];
end





%%
% *************************************************************************
% *************************************************************************
%
% PLOT: RHPP, RHPZ, P/Z RATIO vs \nu -- HSV ONLY
%
% ************************************************************************* 
% *************************************************************************

switch systag
case sysnames.hsv

% *************************************************************************
%
% PLOT: RHPP vs \nu
%
% ************************************************************************* 

figure(figcount)

% PLOT
figure(figcount)    
h_fig = surf(X1, X2, pmat');
set(h_fig, 'FaceColor', facecolor_list{1});
set(h_fig, 'FaceAlpha', facealpha);
set(h_fig, 'EdgeAlpha', edgealpha);            


ttl = ['RHP Pole vs. $\nu$'];
title(ttl)    
xticks(nu_ticks_cell{1});
yticks(nu_ticks_cell{2});
xlabel(x0surf_labels{1});
ylabel(x0surf_labels{2});
zlabel(['RHPP']);
xlim([x1min x1max]);
ylim([x2min x2max]);
 
% Format plot
p_sett.figcount = figcount;
plot_format(p_sett);   
clear p_sett;

% SAVE PLOT
if savefigs
    filename = ['RHPP_vs_nu'];
    savepdf(figcount, relpath, filename); 
end

% Increment figure counter
figcount = figcount + 1; 

% *************************************************************************
%
% PLOT: RHPZ vs \nu
%
% ************************************************************************* 

figure(figcount)

% PLOT
figure(figcount)    
h_fig = surf(X1, X2, zmat');
set(h_fig, 'FaceColor', facecolor_list{1});
set(h_fig, 'FaceAlpha', facealpha);
set(h_fig, 'EdgeAlpha', edgealpha);            


ttl = ['RHP Zero vs. $\nu$'];
title(ttl) 
xticks(nu_ticks_cell{1});
yticks(nu_ticks_cell{2});
xlabel(x0surf_labels{1});
ylabel(x0surf_labels{2});
zlabel(['RHPZ']);
xlim([x1min x1max]);
ylim([x2min x2max]);
 
% Format plot
p_sett.figcount = figcount;
plot_format(p_sett);   
clear p_sett;

% SAVE PLOT
if savefigs
    filename = ['RHPZ_vs_nu'];
    savepdf(figcount, relpath, filename); 
end

% Increment figure counter
figcount = figcount + 1; 

% *************************************************************************
%
% PLOT: P/Z RATIO vs \nu
%
% ************************************************************************* 

figure(figcount)

% PLOT
figure(figcount)    
h_fig = surf(X1, X2, pzmat');
set(h_fig, 'FaceColor', facecolor_list{1});
set(h_fig, 'FaceAlpha', facealpha);
set(h_fig, 'EdgeAlpha', edgealpha);            


ttl = ['Z/P Ratio vs. $\nu$'];
title(ttl)    
xticks(nu_ticks_cell{1});
yticks(nu_ticks_cell{2});
xlabel(x0surf_labels{1});
ylabel(x0surf_labels{2});
zlabel(['Z/P']);
xlim([x1min x1max]);
ylim([x2min x2max]);
 
% Format plot
p_sett.figcount = figcount;
plot_format(p_sett);   
clear p_sett;

% SAVE PLOT
if savefigs
    filename = ['PZ_ratio_vs_nu'];
    savepdf(figcount, relpath, filename); 
end

% Increment figure counter
figcount = figcount + 1; 


end


% *************************************************************************
%
% PLOT: TRIM CONTROLS vs. \nu
%
% ************************************************************************* 

for i = 1:m

    figure(figcount)

    uedata = uemat(:,:,i);
    
    % PLOT
    figure(figcount)    
    h_fig = surf(X1, X2, uedata');
    set(h_fig, 'FaceColor', facecolor_list{1});
    set(h_fig, 'FaceAlpha', facealpha);
    set(h_fig, 'EdgeAlpha', edgealpha);            
    
    % Get propts of u_i
    ui_engname = u_propts_cell{i}.engname;
    ui_texname_e = u_propts_cell{i}.texname_e;
    ui_units = u_propts_cell{i}.units;
    
    ttl = ['Trim ' ui_engname ' $' ui_texname_e '$ vs. $\nu$'];
    zlbl = ['$' ui_texname_e '$'];
    if ~strcmp(ui_units, '')
        zlbl = [zlbl '(' ui_units ')'];
    end
    title(ttl)   
    xticks(nu_ticks_cell{1});
    yticks(nu_ticks_cell{2});
    xlabel(x0surf_labels{1});
    ylabel(x0surf_labels{2});
    zlabel(zlbl);
    xlim([x1min x1max]);
    ylim([x2min x2max]);
     
    % Format plot
    p_sett.figcount = figcount;
    plot_format(p_sett);   
    clear p_sett;
    
    % SAVE PLOT
    if savefigs
        filename = ['u_' num2str(i) 'e_vs_nu'];
        savepdf(figcount, relpath, filename); 
    end
    
    % Increment figure counter
    figcount = figcount + 1; 

end

% *************************************************************************
%
% PLOT: TRIM AOA vs \nu
%
% ************************************************************************* 

figure(figcount)

% PLOT
figure(figcount)    
h_fig = surf(X1, X2, aemat');
set(h_fig, 'FaceColor', facecolor_list{1});
set(h_fig, 'FaceAlpha', facealpha);
set(h_fig, 'EdgeAlpha', edgealpha);            


ttl = ['Trim AOA vs. $\nu$'];
title(ttl)    
xticks(nu_ticks_cell{1});
yticks(nu_ticks_cell{2});
xlabel(x0surf_labels{1});
ylabel(x0surf_labels{2});
zlabel(['$\alpha_{e}$']);
xlim([x1min x1max]);
ylim([x2min x2max]);
 
% Format plot
p_sett.figcount = figcount;
plot_format(p_sett);   
clear p_sett;

% SAVE PLOT
if savefigs
    filename = ['ae_vs_nu'];
    savepdf(figcount, relpath, filename); 
end

% Increment figure counter
figcount = figcount + 1; 


end

