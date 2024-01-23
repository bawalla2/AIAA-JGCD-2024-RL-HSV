function figcount = plot_sweep(alg_settings_cell,...
                        out_data_cell, group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOTS FOR INITIAL CONDITION AND MODELING ERROR SWEEP
%
% Brent Wallace  
%
% 2022-03-11
%
% *************************************************************************
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% group_settings    (Struct) Contains the preset group settings for this
%                   program. This contains settings which are auto-set by
%                   previously-called programs, as well as the following
%                   user-configurable settings. NOTE: See
%                   config_preset_group.m to configure these settings:
%   ICs             (Struct) Contains data for the 2D surface plot
%                   settings. Has the following fields:
%       x10vec      (Vector) Contains a vector of evaluations points for
%                   the first state variable x_1 (i.e., the variable
%                   plotted on the x-axis of the surface plots).
%       x20vec      (Vector) Contains a vector of evaluations points for
%                   the first state variable x_2 (i.e., the variable
%                   plotted on the y-axis of the surface plots).
% out_data_cell     (3-dim Cell) A cell array containing algorithm output
%                   data with the dimensions indexed in: (x_1, x_2, \nu),
%                   where x_1 is the first IC state variable swept, x_2 is
%                   the second IC state variable swept, and \nu is the
%                   vector of modeling error parameters swept.
% master_settings   (Struct) Contains master program settings. Has the
%                   following fields relevant to this program:
%   savefigs        (Bool) Save figures to PDF (=1) or not (=0).
%   relpath         (String) Relative path to figures (for saving only).
%                   NOTE: This string is auto-set. See '/00 figures'.
%   indsmodelplot   (Vector) This contains the indices in the system
%                   modeling error vector 'nuvec' (see config_settings.m)
%                   for which to plot sweep data for.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% figcount          (Integer) Cumulative figure count after all plots in
%                   this function have been created.
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


% Plot nominal LQ curves in surface plots (=1) or not (=0)
plot_nom_lq = 1;

% Number of significant digits to display data with at command window
nsd = 3;

% Number of decimal points to display to at command window
fspc = '%.2f';

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
issweep_rand_nu = group_settings.issweep_rand_nu;

% Algorithm names
algnames = master_settings.algnames;

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

% Plot formatting -- color commands
psett_matlabblue = psett_master.psett_matlabblue;
psett_matlaborange = psett_master.psett_matlaborange;
psett_matlabyellow = psett_master.psett_matlabyellow;
psett_matlabpurple = psett_master.psett_matlabpurple;
psett_matlabgreen = psett_master.psett_matlabgreen;
psett_matlablightblue = psett_master.psett_matlablightblue;
psett_matlabmaroon = psett_master.psett_matlabmaroon;
psett_black = psett_master.psett_black;
psett_gray = psett_master.psett_gray;

% Plot formatting -- loop-wise color commands
% loop_color_sett_cell = psett_master.loop_color_sett_cell;
color_sett_cell = psett_master.color_sett_cell;


% Surface plot face transparency
facealpha = psett_master.facealpha;

% Surface plot edge transparency
edgealpha = psett_master.edgealpha;

% ***********************
%
% SURFACE PLOT SETTINGS -- OPTIMAL WEIGHT VALUE PLANES
%        

% Surface plot face color
facecolor_hl = psett_master.facecolor_hl;

% Surface plot face transparency
facealpha_hl = psett_master.facealpha_hl;

% Surface plot edge transparency
edgealpha_hl = psett_master.edgealpha_hl;    


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
if issweep_nu
    nunames_cell = sys.nunames_cell;
    nu_ticks_cell = sys_plot_settings.nu_ticks_cell;
end

% Get vector of \nu values
nuvec = sys.nuvec;
nuvec_plot = nuvec(indsmodelplot);
nu1vec = sys.nu1vec;
nu2vec = sys.nu2vec;


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

switch sweeptype

case sweeptypenames.IC
   
% Sweep variable indices
inds_x_sweep = group_settings.inds_x_sweep;

% IC data
ICs = group_settings.ICs;
ns1 = ICs.numx10;
ns2 = ICs.numx20;
% numICs = ICs.numICs;
x10vec = ICs.x10vec;
x20vec = ICs.x20vec;
tx10vec = ICs.tx10vec;
tx20vec = ICs.tx20vec;
% x0mat = ICs.x0mat;

% Scale these IC vectors to desired units
for i = 1:2
    ind_xri = inds_x_sweep(i);
    currscl = x_sclvec(ind_xri);
    % Shift by trim if desired
    if shift_sweep_IC_xe
        switch i
            case 1
                x10vecp = tx10vec;
            case 2
                x20vecp = tx20vec;
        end
    else
        switch i
            case 1
                x10vecp = x10vec;
            case 2
                x20vecp = x20vec;
        end
    end
    % Apply scaling
    switch i
        case 1
            x10vecp = x10vecp * currscl;
        case 2
            x20vecp = x20vecp * currscl;
    end
end
       
% % Indices in the state vector corresponding to the state variables swept
% xinds = group_settings.xinds;
% 
% % Check whether surface plots should be plotted or not
% do_sweep_plots = (nx1 > 1) && (nx2 > 1);

% Set the plot vectors
vecp1 = x10vecp;
vecp2 = x20vecp;

case {sweeptypenames.nu; sweeptypenames.rand_nu}


% Set the plot vectors
vecp1 = nu1vec;
vecp2 = nu2vec;
    
end             % END switch sweeptype

if ~issweep_rand_nu

    % Create meshgrid of IC values
    [X1, X2] = meshgrid(vecp1, vecp2);
    
    % Get verticies of IC sweep
    x1min = min(vecp1);
    x1max = max(vecp1);
    x2min = min(vecp2);
    x2max = max(vecp2);
    x1vert = [x1min x1max x1max x1min];
    x2vert = [x2min x2min x2max x2max];
   
end

% Size of sweep in each dimension
ns1 = size(vecp1,1);
ns2 = size(vecp2,1);



% ***********************
%
% PULL OPTIMAL LQ DATA
%   

% Number of loops executed
numloops = size(alg_settings_cell{1}.loop_cell,1);

% Simulation model optimal LQ data
lq_data_opt_sim_cell = cell(numnu1,numnu2);

% Optimal controllers in each loop
Kstar_cell = cell(numnu1,numnu2,numloops);

% Get optimal LQ data for each model
for mcnt1 = 1:numnu1
for mcnt2 = 1:numnu2
    lqdataoptsim = lq_data_cell{mcnt1,mcnt2};
    lq_data_opt_sim_cell{mcnt1,mcnt2} = lqdataoptsim;
    for j = 1:numloops
        switch numloops
            case 1
                Kstar_cell{mcnt1,mcnt2,j} = lqdataoptsim.Kcirl;
            case 2
                switch j
                    case 1
                        Kstar_cell{mcnt1,mcnt2,j} = ...
                            lqdataoptsim.lq_data_11.K;
                    case 2
                        Kstar_cell{mcnt1,mcnt2,j} = ...
                            lqdataoptsim.lq_data_22.K;
                end
        end
    end
end
end

% ***********************
%
% PLOT SETTINGS -- COLORS
%  

% Loop colors
loop_color_sett_cell = color_sett_cell(1:numloops);

% List of colors
facecolor_list = cell(nummodelsplot,1);

% Counter to keep track of number of models so far that weren't the nominal
cntnotnom = 1;

for mcnt1 = 1:nummodelsplot
    
    % Get the index of the current model
    indmcnt = indsmodelplot(mcnt1); 

    % If \nu = 1, set color to blue. Else, go down the line
    if indmcnt == indnom
        facecolor_list{mcnt1} = colors.matlabblue;
    else
        switch cntnotnom
            case 1
                facecolor_list{mcnt1} = colors.matlabgreen;
            case 2
                facecolor_list{mcnt1} = colors.matlabyellow;      
            case 3
                facecolor_list{mcnt1} = colors.matlaborange;                   
        end
        % If this model is the last one, make its color maroon
        if (cntnotnom + 1) == nummodelsplot
            facecolor_list{mcnt1} = colors.matlabmaroon;
        end
        % Increment counter
        cntnotnom = cntnotnom + 1;        
    end

end

% ***********************
%
% PLOT SETTINGS -- LEGEND -- \nu VALUES
%  

lgd_nu = cell(nummodelsplot,1);

for mcnt1 = 1:numnu1   

    % Get \nu for this model
    numcnt = nuvec(mcnt1);

    % Add legend entry
    lgd_nu{mcnt1} = ['$\nu = ' num2str(numcnt) '$'];

end

% Plotted values of \nu only
lgd_nu_plot = lgd_nu(indsmodelplot);

% ***********************
%
% PLOT SETTINGS -- LEGEND -- LOOP NAMES
%  

lgd_loop = cell(numloops,1);

for j = 1:numloops
    
    lgd_loop{j} = ['$j =' num2str(j) '$ $('...
        y_propts_cell{j}.texname ')$'];

end

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
        lgd_plot{1} = algnames.mi_dirl;
end



% ***********************
%
% PLOT SETTINGS -- x, y AXIS LABELS FOR IC SWEEP PLOTS
%  

x0surf_labels = cell(2,1);

switch sweeptype
    case sweeptypenames.IC
        for i = 1:2
            y_propts = y_propts_cell{i};
            currtexname = y_propts.texname;
            currunits = y_propts.units;
            x0surf_labels{i} = ['$' currtexname '_{0}$ (' currunits ')'];
        end
    case sweeptypenames.nu
        for i = 1:2
            x0surf_labels{i} = ['$\nu_{' nunames_cell{i} '}$'];
        end
end



% *************************************************************************
% 
% PULL, CALCULATE SWEEP DATA
%
% *************************************************************************

% ***********************
%
% PULL SWEEP DATA
%  

switch sweeptype

    case sweeptypenames.IC

        sweep_lq_data_cell = cell(ns1,ns2,nummodels);
        cond_data_cell = cell(ns1,ns2,nummodels);
        
        % Stores the iteration-max conditioning at each IC for each model
        max_cond_data_mat = zeros(ns1,ns2,nummodels,numloops);
        
        % Final controller error 
        normeKjmat = zeros(ns1,ns2,nummodels,numloops);
        
        for s1c = 1:ns1
            for s2c = 1:ns2
                for mcnt1 = 1:nummodels
        
                    % Extract current out_data struct
                    outdata = out_data_cell{s1c,s2c,mcnt1};
        
                    % Extract lq_data
                    lqdata = outdata.dirl_data.lq_data;
        
                    % Extract conditioning data
                    if isfield(outdata,'cond_AS_vec_cell')
                        conddata = outdata.cond_AS_vec_cell;
                    else
                        conddata = outdata.cond_A_vec_cell;
                    end
        
                    % For each of the loops j, calculate ||K_{i^*,j} - K_j^*||
                    for j = 1:numloops
                        
                        % Get optimal LQ controller for this model, this
                        % loop
                        Kstarj = Kstar_cell{mcnt1,1,j};
        
                        % Get final controller for this IC, this model
                        Kistarj = outdata.K_cell{j}(:,:,end);
        
                        % Calculate final controller error
                        eKj = Kstarj - Kistarj;
                        normeKj = norm(eKj);
        
                        % Store error
                        normeKjmat(s1c,s2c,mcnt1,j) = normeKj;
                                       
                    end
        
                    % For each of the loops j, calculate max conditioning
                    for j = 1:numloops
                        
                        % Get conditioning data in this loop
                        conddataj = conddata{j};
        
                        % Get iteration-max conditioning
                        maxcondj = max(conddataj);
        
                        % Store
                        max_cond_data_mat(s1c,s2c,mcnt1,j) = maxcondj;
        
                    end
        
        
                    % Store lq_data
                    sweep_lq_data_cell{s1c,s2c,mcnt1} = lqdata;
        
                    % Store conditioning data
                    cond_data_cell{s1c,s2c,mcnt1} = conddata;
        
                end
            end
        end

    case {sweeptypenames.nu; sweeptypenames.rand_nu}

        sweep_lq_data_cell = cell(ns1,ns2);
        cond_data_cell = cell(ns1,ns2);
        
        % Stores the iteration-max conditioning at each IC for each model
        max_cond_data_mat = zeros(ns1,ns2,numloops);
        
        % Final controller error 
        normeKjmat = zeros(ns1,ns2,numloops);
        
        for s1c = 1:ns1
            for s2c = 1:ns2
    
                % Extract current out_data struct
                outdata = out_data_cell{s1c,s2c};
    
                % Extract lq_data
                lqdata = outdata.dirl_data.lq_data;
    
                % Extract conditioning data
                if isfield(outdata,'cond_AS_vec_cell')
                    conddata = outdata.cond_AS_vec_cell;
                else
                    conddata = outdata.cond_A_vec_cell;
                end
    
                % For each of the loops j, calculate ||K_{i^*,j} - K_j^*||
                for j = 1:numloops
                    
                    % Get optimal LQ controller for this model, this loop
                    Kstarj = Kstar_cell{s1c,s2c,j};
    
                    % Get final controller for this IC, this model
                    Kistarj = outdata.K_cell{j}(:,:,end);
    
                    % Calculate final controller error
                    eKj = Kstarj - Kistarj;
                    normeKj = norm(eKj);
    
                    % Store error
                    normeKjmat(s1c,s2c,j) = normeKj;
                                   
                end
    
                % For each of the loops j, calculate max conditioning
                for j = 1:numloops
                    
                    % Get conditioning data in this loop
                    conddataj = conddata{j};
    
                    % Get iteration-max conditioning
                    maxcondj = max(conddataj);
    
                    % Store
                    max_cond_data_mat(s1c,s2c,j) = maxcondj;
    
                end
    
    
                % Store lq_data
                sweep_lq_data_cell{s1c,s2c} = lqdata;
    
                % Store conditioning data
                cond_data_cell{s1c,s2c} = conddata;

            end
        end


end

% ***********************
%
% CALCULATE NOMINAL LQ CONTROLLER ERROR
%  

K_nom_cell = cell(numloops,1);
normeKjnommat = zeros(numnu1,numnu2,numloops);

% Get nominal LQ controllers
for j = 1:numloops
    switch numloops
        case 1
            K_nom_cell{j} = lq_data_nom.Kcirl;
        case 2
            switch j
                case 1
%                     Kstar_nom_cell{j} = lq_data_nom.lq_data_11.K;
                    K_nom_cell{j} = lq_data_0.lq_data_11.K;
                case 2
%                     Kstar_nom_cell{j} = lq_data_nom.lq_data_22.K;
                    K_nom_cell{j} = lq_data_0.lq_data_22.K;
            end
    end
end

% For each model and loop, calculate ||K_{nom,j}^{*} - K_j^*||
for mcnt1 = 1:numnu1
    for mcnt2 = 1:numnu2
        for j = 1:numloops
            
            % Get optimal LQ controller for this model, this loop
            Kstarj = Kstar_cell{mcnt1,mcnt2,j};
    
            % Get nominal controller for this loop
            Knomj = K_nom_cell{j};
    
            % Calculate controller error
            eKj = Kstarj - Knomj;
            normeKj = norm(eKj);
    
            % Store error
            normeKjnommat(mcnt1,mcnt2,j) = normeKj;
    
        end
    end
end

% ***********************
%
% CALCULATE MEAN, MAX, STD CONTROLLER ERROR, PERCENT CONTROLLER ERROR,
% CONDITIONING BASED ON SWEEP DATA
%  

switch sweeptype

% *** IC SWEEP
case sweeptypenames.IC

% IC sweep mean, max, std controller error vs \nu
avgnormeKjmat = zeros(nummodels,numloops);
maxnormeKjmat = zeros(nummodels,numloops);
stdnormeKjmat = zeros(nummodels,numloops);

% IC sweep mean, max, std conditioning vs \nu
avgcondjmat = zeros(nummodels,numloops);
maxcondjmat = zeros(nummodels,numloops);
stdcondjmat = zeros(nummodels,numloops);

% IC sweep mean, min, std percent controller error reduction vs \nu
avgpctredKjmat = zeros(nummodels,numloops);
minpctredKjmat = zeros(nummodels,numloops);
stdpctredKjmat = zeros(nummodels,numloops);

for mcnt1 = 1:numnu1
    for j = 1:numloops

        % Get nominal LQ error
        normeKjnom = normeKjnommat(mcnt1,1,j);
%         isoptKjnom = (normeKjnom < 1e-4);

        % Pull controller error data
        eKjs = normeKjmat(:,:,mcnt1,j);
        eKjs = eKjs(:);

        % Calculate mean, max, std controller error
        avgnormeKj = mean(eKjs);
        maxnormeKj = max(eKjs);
        stdnormeKj = std(eKjs);

        % Calculate percentage reduction
        if normeKjnom ~= 0
            pctredKj = (1 - eKjs ./ normeKjnom) * 100;
        else
            pctredKj = nan;
        end


        % Pull conditioning data
        conddataj = max_cond_data_mat(:,:,mcnt1,j);
        conddataj = conddataj(:);

        % Calculate mean, max, std conditioning
        avgcondj = mean(conddataj);
        maxcondj = max(conddataj);
        stdcondj = std(conddataj);

        % Store mean, max, std controller error
        avgnormeKjmat(mcnt1,j) = avgnormeKj;
        maxnormeKjmat(mcnt1,j) = maxnormeKj;
        stdnormeKjmat(mcnt1,j) = stdnormeKj;

        % Store mean, min, std percent controller error reduction vs \nu
        avgpctredKjmat(mcnt1,j) = mean(pctredKj);
        minpctredKjmat(mcnt1,j) = min(pctredKj);
        stdpctredKjmat(mcnt1,j) = std(pctredKj);        

        % Store mean, max, std conditioning
        avgcondjmat(mcnt1,j) = avgcondj;
        maxcondjmat(mcnt1,j) = maxcondj;
        stdcondjmat(mcnt1,j) = stdcondj;

    end
end

% *** \nu SWEEP
case {sweeptypenames.nu; sweeptypenames.rand_nu}

% \nu sweep mean, max, std controller error vs \nu
avgnormeKjmat = zeros(numloops,1);
maxnormeKjmat = zeros(numloops,1);
stdnormeKjmat = zeros(numloops,1);

% \nu sweep mean, max, std nominal controller error vs \nu
avgnormeKjnommat = zeros(numloops,1);
minnormeKjnommat = zeros(numloops,1);
maxnormeKjnommat = zeros(numloops,1);
stdnormeKjnommat = zeros(numloops,1);

% \nu sweep mean, max, std percent controller error vs \nu
avgpctredKjmat = zeros(numloops,1);
minpctredKjmat = zeros(numloops,1);
maxpctredKjmat = zeros(numloops,1);
stdpctredKjmat = zeros(numloops,1);

% \nu sweep mean, max, std conditioning vs \nu
avgcondjmat = zeros(numloops,1);
maxcondjmat = zeros(numloops,1);
stdcondjmat = zeros(numloops,1);

for j = 1:numloops

    % Pull controller error data
    eKjs = normeKjmat(:,:,j);
    eKjs = eKjs(:);

    % Pull nominal controller error data
    eKjsnom = normeKjnommat(:,:,j);
    eKjsnom = eKjsnom(:);

    % Calculate percentage reduction
    pctredKj = (1 - eKjs ./ eKjsnom) * 100;

    % Calculate mean, max, std controller error
    avgnormeKj = mean(eKjs);
    maxnormeKj = max(eKjs);
    stdnormeKj = std(eKjs);

    % Pull conditioning data
    conddataj = max_cond_data_mat(:,:,j);
    conddataj = conddataj(:);

    % Calculate mean, max, std conditioning
    avgcondj = mean(conddataj);
    maxcondj = max(conddataj);
    stdcondj = std(conddataj);

    % Store mean, max, std controller error
    avgnormeKjmat(j) = avgnormeKj;
    maxnormeKjmat(j) = maxnormeKj;
    stdnormeKjmat(j) = stdnormeKj;

    % Store mean, max, std nominal controller error
    avgnormeKjnommat(j) = mean(eKjsnom);
    minnormeKjnommat(j) = min(eKjsnom);
    maxnormeKjnommat(j) = max(eKjsnom);
    stdnormeKjnommat(j) = std(eKjsnom);

    % Store mean, max, std percent reduction
    avgpctredKjmat(j) = mean(pctredKj);
    minpctredKjmat(j) = min(pctredKj);
    maxpctredKjmat(j) = max(pctredKj);
    stdpctredKjmat(j) = std(pctredKj);

    % Store mean, max, std conditioning
    avgcondjmat(j) = avgcondj;
    maxcondjmat(j) = maxcondj;
    stdcondjmat(j) = stdcondj;

end

end

% ***********************
%
% CALCULATE WORST-CASE OPTIMALITY RECOVERY
%  

switch sweeptype

% *** IC SWEEP
case sweeptypenames.IC

% IC sweep worst-case optimality recovery vs \nu
minerrredmat = zeros(nummodels,numloops);

for mcnt1 = 1:nummodels
    for j = 1:numloops
        
        % Get worst-case controller error
        maxnormeKj = maxnormeKjmat(mcnt1,j);

        % Get nominal LQ error
        normeKjnom = normeKjnommat(mcnt1,j);

        % Calculate worst-case optimality recovery
        minerrred = (1 - maxnormeKj / normeKjnom) * 100;

        % Store worst-case optimality recovery
        minerrredmat(mcnt1,j) = minerrred;

    end
end

% *** \nu SWEEP
case {sweeptypenames.nu; sweeptypenames.rand_nu}

% IC sweep worst-case optimality recovery vs
minerrredmat = zeros(numloops);

for j = 1:numloops
    
    % Get worst-case controller error
    maxnormeKj = maxnormeKjmat(j);

    % Get nominal LQ error
    normeKjnom = normeKjnommat(j);

    % Calculate worst-case optimality recovery
    minerrred = (1 - maxnormeKj / normeKjnom) * 100;

    % Store worst-case optimality recovery
    minerrredmat(j) = minerrred;

end


end

%%
% *************************************************************************
% *************************************************************************
%
% PLOTS: CONTROLLER ERROR VS. IC FOR VARYING MODELS
%
% ************************************************************************* 
% *************************************************************************

if ~issweep_rand_nu

for j = 1:numloops 

    for mcnt1 = 1:nummodelsplot 
    
        % Get the index of the current model
        indmcnt = indsmodelplot(mcnt1);

        % Extract data for this model, this loop
        switch sweeptype
            case sweeptypenames.IC
                neKjmat = normeKjmat(:,:,indmcnt,j);
            case sweeptypenames.nu
                neKjmat = normeKjmat(:,:,j);
        end
        
        % PLOT
        figure(figcount)    
        h_fig = surf(X1, X2, neKjmat');
        set(h_fig, 'FaceColor', facecolor_list{mcnt1});
        set(h_fig, 'FaceAlpha', facealpha);
        set(h_fig, 'EdgeAlpha', edgealpha);            
        hold on
        
    end

    % Plot the nominal LQ error
    if plot_nom_lq

        % Extract the nominal LQ error
        switch sweeptype
            case sweeptypenames.IC

                % Extract the nominal LQ error    
                normeKjnom = normeKjnommat(end,j);
        
                % Extract \nu for this model
                nuend = nuvec_plot(end);
        
                % PLOT
                h_fig = patch('XData', x1vert, 'YData', x2vert, ...
                        'ZData', normeKjnom * ones(1,4));

            case sweeptypenames.nu

                % Extract the nominal LQ error    
                normeKjnom = normeKjnommat(:,:,j);
        
                % PLOT
                h_fig = surf(X1, X2, normeKjnom');               

        end

        set(h_fig, 'FaceColor', facecolor_hl);
        set(h_fig, 'FaceAlpha', facealpha_hl);
        set(h_fig, 'EdgeAlpha', edgealpha_hl);      

        % Legend entry
        switch sweeptype
            case sweeptypenames.IC
            eKjnom_str_nd = ...
                ['e_{K_{' num2str(j) '},nom}(' num2str(nuend) ')'];
            case sweeptypenames.nu
            eKjnom_str_nd = ...
                ['e_{K_{' num2str(j) '},nom}( \nu )'];
        end
%         eKjnom_str = ['$' eKjnom_str_nd '$'];
        eKjnom_str = [algnames.lq_K0];

    end     
    

%     Kerr_str = ['$||K_{i^{*},' num2str(j) '} - K_{' num2str(j) '}^{*}(\nu)||$'];  
%     Kerr_str = ['$e_{K_{ '  num2str(j)  '}}$'];
Kerr_str = ['$||K_{i^{*},' num2str(j) '} - K_{' num2str(j) '}^{*}||$']; 
    ttl = ['Controller Optimality Error ' Kerr_str ' vs. '];
    switch sweeptype
        case sweeptypenames.IC
        ttl = [ttl '$x_{0}$'];
        case sweeptypenames.nu
        ttl = [ttl '$\nu$'];
    end
    title(ttl)          
    xlabel(x0surf_labels{1});
    ylabel(x0surf_labels{2});
    zlabel([Kerr_str]);
    if issweep_nu
        xticks(nu_ticks_cell{1});
        yticks(nu_ticks_cell{2});
    end
    xlim([x1min x1max]);
    ylim([x2min x2max]);
    
    if plot_nom_lq
        lgd = legend([lgd_plot; {eKjnom_str}]);    
    else
        lgd = legend(lgd_plot);   
    end       

    % Position legend, view manually
    % Get current legend position
    currfig = gcf;
    lgdpos = currfig.CurrentAxes.Legend.Position;
    switch systag
        case sysnames.hsv
            lgdpos(1:2) = [0.6245 0.4661];
            p_sett.custom_sett.lgd_position = lgdpos;
        case sysnames.ddmr
            lgdpos(1:2) = [0.5414 0.4982];  
            p_sett.custom_sett.lgd_position = lgdpos;
            p_sett.custom_sett.axview = [-30.4919 13.8250];
    end

    % Format plot
    p_sett.figcount = figcount;
    plot_format(p_sett);   
    clear p_sett;
    
    % SAVE PLOT
    if savefigs
        jstr = strrep(num2str(j),'.','p');
        switch sweeptype
            % *** IC SWEEP
            case sweeptypenames.IC
                filename = ['K_err_vs_x0_j_' jstr];
            % *** \nu SWEEP
            case sweeptypenames.nu
                filename = ['K_err_vs_nu_j_' jstr];
        end 
        savepdf(figcount, relpath, filename); 
    end

    % Increment figure counter
    figcount = figcount + 1; 

end

end


%%
% *************************************************************************
% *************************************************************************
%
% PLOT: CONDITIONING VS. x_0
%
% ************************************************************************* 
% *************************************************************************

if issweep_IC

for j = 1:numloops

for mcnt1 = 1:nummodelsplot

    % Get the index of the current model
    indmcnt = indsmodelplot(mcnt1);

    % Get current conditioning data
    conddata = max_cond_data_mat(:,:,indmcnt,j);
    
    % PLOT
    figure(figcount)    
    h_fig = surf(X1, X2, conddata');
    set(h_fig, 'FaceColor', facecolor_list{mcnt1});
    set(h_fig, 'FaceAlpha', facealpha);
    set(h_fig, 'EdgeAlpha', edgealpha);            
    hold on   

end
    
    
    Aijstr = ['$\max\limits_{i} \kappa(\mathbf{A}_{i,' num2str(j) '})$'];
    ttl = ['Max Conditioning ' Aijstr ' vs. $x_{0}$'];
    title(ttl)          
    xlabel(x0surf_labels{1});
    ylabel(x0surf_labels{2});
    zlabel([Aijstr]);
    xlim([x1min x1max]);
    ylim([x2min x2max]);
    legend(lgd_plot);
     
    % Format plot
    p_sett.figcount = figcount;
    plot_format(p_sett);   
    clear p_sett;
    
    % SAVE PLOT
    if savefigs
        filename = ['cond_A_vs_x0_j_' num2str(j)];
        savepdf(figcount, relpath, filename); 
    end
    
    % Increment figure counter
    figcount = figcount + 1; 

end
end

%%
% *************************************************************************
% *************************************************************************
%
% PLOT: CONDITIONING VS. \nu
%
% ************************************************************************* 
% *************************************************************************

if issweep_nu

for j = 1:numloops


    % Get current conditioning data
    conddata = max_cond_data_mat(:,:,j);
    
    % PLOT
    figure(figcount)    
    h_fig = surf(X1, X2, conddata');
    set(h_fig, 'FaceColor', facecolor_list{1});
    set(h_fig, 'FaceAlpha', facealpha);
    set(h_fig, 'EdgeAlpha', edgealpha);            
    hold on   

    
    Aijstr = ['$\max\limits_{i} \kappa(\mathbf{A}_{i,' num2str(j) '})$'];
    ttl = ['Max Conditioning ' Aijstr ' vs. $\nu$'];
    title(ttl)          
    xlabel(x0surf_labels{1});
    ylabel(x0surf_labels{2});
    zlabel([Aijstr]);
    xticks(nu_ticks_cell{1});
    yticks(nu_ticks_cell{2});    
    xlim([x1min x1max]);
    ylim([x2min x2max]);

    % Format plot
    p_sett.figcount = figcount;
    plot_format(p_sett);   
    clear p_sett;
    
    % SAVE PLOT
    if savefigs
        filename = ['cond_A_vs_nu_j_' num2str(j)];
        savepdf(figcount, relpath, filename); 
    end
    
    % Increment figure counter
    figcount = figcount + 1; 

end
end





%%
% *************************************************************************
% *************************************************************************
%
% PRINT SWEEP METRICS
%
% ************************************************************************* 
% *************************************************************************

disp('***************************************************************')
disp('*')
disp('* DISPLAYING x_0 SWEEP PERFORMANCE METRICS')
disp('*')
disp('***************************************************************')


% ***********************
%
% FORMATTING
%  

% One tab
onetab = '  ';
twotab = [onetab onetab];
fourtab = [twotab twotab];

% String to print out \nu values
strng_j = [fourtab twotab];
for j = 1:numloops
    strng_j = [strng_j lgd_loop{j} onetab];
end

% ***********************
%
% NOMINAL CONTROLLER ERROR
%  

if issweep_IC
    disp('*****')
    disp('*')
    disp('* NOMINAL CONTROLLER ERROR:')
    disp('*')
    
    disp(strng_j);
    tmpmat = normeKjnommat;
    for mcnt1 = 1:nummodels
        tmp = num2str(tmpmat(mcnt1,:));
        disp([lgd_nu{mcnt1} twotab tmp])
    end
end

% ***********************
%
% MAX NOMINAL CONTROLLER ERROR
%  

if issweep_nu || issweep_rand_nu
    disp('*****')
    disp('*')
    disp('* MAX NOMINAL CONTROLLER ERROR:')
    disp('*')
    
    disp(strng_j);
    tmpmat = maxnormeKjnommat;
    disp([fourtab twotab num2str(tmpmat')])
end


% ***********************
%
% MEAN NOMINAL CONTROLLER ERROR
%  

if issweep_nu || issweep_rand_nu
    disp('*****')
    disp('*')
    disp('* MEAN NOMINAL CONTROLLER ERROR:')
    disp('*')
    
    disp(strng_j);
    tmpmat = avgnormeKjnommat;
    disp([fourtab twotab num2str(tmpmat')])
end

% ***********************
%
% STD NOMINAL CONTROLLER ERROR
%  

if issweep_nu || issweep_rand_nu
    disp('*****')
    disp('*')
    disp('* STD NOMINAL CONTROLLER ERROR:')
    disp('*')
    
    disp(strng_j);
    tmpmat = stdnormeKjnommat;
    disp([fourtab twotab num2str(tmpmat')])
end

% ***********************
%
% MAX CONTROLLER ERROR
%  

disp('*****')
disp('*')
disp('* MAX CONTROLLER ERROR:')
disp('*')

disp(strng_j);
tmpmat = maxnormeKjmat;
if issweep_IC
    for mcnt1 = 1:nummodels
        tmp = num2str(tmpmat(mcnt1,:));
        disp([lgd_nu{mcnt1} twotab tmp])
    end
else
    disp([fourtab twotab num2str(tmpmat')])
end

% ***********************
%
% MEAN CONTROLLER ERROR
%  

disp('*****')
disp('*')
disp('* MEAN CONTROLLER ERROR:')
disp('*')

disp(strng_j);
tmpmat = avgnormeKjmat;
if issweep_IC
for mcnt1 = 1:nummodels
    tmp = num2str(tmpmat(mcnt1,:));
    disp([lgd_nu{mcnt1} twotab tmp])
end
else
    disp([fourtab twotab num2str(tmpmat')])
end


% ***********************
%
% STD CONTROLLER ERROR
%  

disp('*****')
disp('*')
disp('* STD CONTROLLER ERROR:')
disp('*')

disp(strng_j);
tmpmat = stdnormeKjmat;
if issweep_IC
for mcnt1 = 1:nummodels
    tmp = num2str(tmpmat(mcnt1,:));
    disp([lgd_nu{mcnt1} twotab tmp])
end
else
    disp([fourtab twotab num2str(tmpmat')])
end

% ***********************
%
% MIN CONTROLLER OPTIMALITY PERCENT REDUCTION
%  

disp('*****')
disp('*')
disp('* MIN CONTROLLER OPTIMALITY PERCENT REDUCTION:')
disp('*')

disp(strng_j);
tmpmat = minpctredKjmat;
if issweep_IC
    for mcnt1 = 1:nummodels
        tmp = num2str(tmpmat(mcnt1,:));
        disp([lgd_nu{mcnt1} twotab tmp])
    end
else
    disp([fourtab twotab num2str(tmpmat')])
end


% ***********************
%
% MEAN CONTROLLER OPTIMALITY PERCENT REDUCTION
%  

disp('*****')
disp('*')
disp('* MEAN CONTROLLER OPTIMALITY PERCENT REDUCTION:')
disp('*')

disp(strng_j);
tmpmat = avgpctredKjmat;
if issweep_IC
    for mcnt1 = 1:nummodels
        tmp = num2str(tmpmat(mcnt1,:));
        disp([lgd_nu{mcnt1} twotab tmp])
    end
else
    disp([fourtab twotab num2str(tmpmat')])
end

% ***********************
%
% STD CONTROLLER OPTIMALITY PERCENT REDUCTION
%  

disp('*****')
disp('*')
disp('* STD CONTROLLER OPTIMALITY PERCENT REDUCTION:')
disp('*')
 
disp(strng_j);
tmpmat = stdpctredKjmat;
if issweep_IC
    for mcnt1 = 1:nummodels
        tmp = num2str(tmpmat(mcnt1,:));
        disp([lgd_nu{mcnt1} twotab tmp])
    end
else
    disp([fourtab twotab num2str(tmpmat')])
end

% ***********************
%
% MAX CONDITIONING
%  

disp('*****')
disp('*')
disp('* MAX CONDITIONING:')
disp('*')

disp(strng_j);
tmpmat = maxcondjmat;
if issweep_IC
for mcnt1 = 1:nummodels
    tmp = num2str(tmpmat(mcnt1,:));
    disp([lgd_nu{mcnt1} twotab tmp])
end
else
    disp([fourtab twotab num2str(tmpmat')])
end

% ***********************
%
% MEAN CONDITIONING
%  

disp('*****')
disp('*')
disp('* MEAN CONDITIONING:')
disp('*')

disp(strng_j);
tmpmat = avgcondjmat;
if issweep_IC
for mcnt1 = 1:nummodels
    tmp = num2str(tmpmat(mcnt1,:));
    disp([lgd_nu{mcnt1} twotab tmp])
end
else
    disp([fourtab twotab num2str(tmpmat')])
end

% ***********************
%
% STD CONDITIONING
%  

disp('*****')
disp('*')
disp('* STD CONDITIONING:')
disp('*')

disp(strng_j);
tmpmat = stdcondjmat;
if issweep_IC
for mcnt1 = 1:nummodels
    tmp = num2str(tmpmat(mcnt1,:));
    disp([lgd_nu{mcnt1} twotab tmp])
end
else
    disp([fourtab twotab num2str(tmpmat')])
end

