function [alg_settings_cell, out_data_cell] = calc_tracking_metrics...
    (alg_settings_cell, out_data_cell, group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALCULATE TRACKING METRICS
%
% Brent Wallace  
%
% 2022-10-19
%
% This program, given simulation data, calculates for each system output
% the relevant tracking performance metrics (e.g., max error, step response
% metrics).
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


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
% UNPACK ALGORITHM SETTINGS/PARAMETERS
% 
% *************************************************************************

% Number of presets
numpresets = group_settings.numpresets;

% Save figures control
savefigs = group_settings.savefigs;

% Relative path to data
if savefigs
    relpath_data = [group_settings.relpath group_settings.relpath_data];
    filename_data = group_settings.filename_textable1;
    rowpad_cell = group_settings.rowpad_cell1;
end

% Print performance metrics (=1) or don't print (=0)
print_metrics = group_settings.print_metrics;

% Reference signal r(t) settings
if ~group_settings.istraining
    r_sett = group_settings.r_sett;
else
    r_sett = alg_settings_cell{1}.r_sett_train;
end

% % Thresholds to determine settling time
% thres_eV = group_settings.thres_eV;
% thres_eh = group_settings.thres_eh;

% System
sys = master_settings.sys;             % System array
n = sys.n;                          % System order
m = sys.m;                          % System input dimension
% model = sys.model;                  % System model

% System cell array
model_cell = sys.model_cell;
nummodels = sys.nummodels;

% Indices of nominal, perturbed models
indnom = sys.indnom;

% Nominal, perturbed models
model = get_elt_multidim(model_cell, indnom);                  % System model

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% Indices of state variables to be tracked
inds_xr = model.inds_xr;

% Check if step velocity reference command was issued
isstep = isfield(r_sett,'Arvec');
if isstep
    Arvec = r_sett.Arvec;
end
% isstep = isstep_V;
% if isstep_V
%     Vr = r_sett.Vr;
% end
% 
% % Check if step altitude reference command was issued
% isstep_h = isfield(r_sett,'hr');
% if isstep_h
%     hr = r_sett.hr;
% end
% 
% % Check if step FPA reference command was issued
% isstep_g = isfield(r_sett,'gr');
% if isstep_g
%     gr = r_sett.gr;
% end

% Output properties
y_propts_cell = master_settings.sys_plot_settings.y_propts_cell;

% Control properties
u_propts_cell = master_settings.sys_plot_settings.u_propts_cell;

% State trajectory x(t) unit scaling. E.g., if x_1(t) is angular
% displacement but is desired in degrees, declare the field 'sclvec' and
% put sclvec(1) = 180/pi.
x_sclvec = group_settings.sys_plot_settings.x_sclvec;

% Get settling, rise time thresholds
if isstep
    trpctvec = group_settings.trpctvec;
    tspctvec = group_settings.tspctvec;
end
threshmat = group_settings.threshmat;
threshmat_txt = group_settings.threshmat_txt;
numthresh = size(threshmat,1);

% Check if is a random \nu sweep
issweep_rand_nu = group_settings.issweep_rand_nu;

% Get number of models in the sweep (if is a sweep)
if issweep_rand_nu
    nummodels_sweep = nummodels;
else
    nummodels_sweep = 1;
end

% Algorithm names
algnames = master_settings.algnames;

% List of algorithm presets
alg_list = group_settings.alg_list;

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN LOOP
%
% Loop over the presets in the group
%
% *************************************************************************
% *************************************************************************
% *************************************************************************



% Cell array to hold performance metrics
if issweep_rand_nu
    perf_metric_cell = cell(nummodels_sweep,numpresets);
else
    perf_metric_cell = cell(numpresets,1);
end


for mcnt = 1:nummodels_sweep

for presetcount = 1:numpresets


% *************************************************************************
% *************************************************************************
%
% INIT
%
% *************************************************************************
% *************************************************************************

% % Reset figure counter
% figcount = figcount_begin;

% Get current alg_settings struct
if issweep_rand_nu
    alg_settings = alg_settings_cell{mcnt,presetcount};
else
    alg_settings = alg_settings_cell{presetcount};
end

% Get current out_data struct
if issweep_rand_nu
    out_data = out_data_cell{mcnt,presetcount};
else
    out_data = out_data_cell{presetcount};
end

% Get current algorithm name
curralg = alg_list{presetcount};


% ***********************
%       
% PRESET SETTINGS
%   

% % Model linear (=1) or nonlinear (=0)
% lin1nonlin0 = alg_settings.lin1nonlin0;



% ***********************
%       
% OUTPUT DATA
%   

tvec = out_data.tvec;
xmat = out_data.xmat;
umat = out_data.umat;

% *************************************************************************
%
% EXTRACT SIMULATION MODEL SETTINGS
% 
% *************************************************************************

% % Simulation model
% model_sim = alg_settings.model_sim;

% Simulation model
model_sim_ind = alg_settings.model_sim_ind;
model_sim = model_cell{model_sim_ind};

% ***********************
%       
% TRIM CONDITIONS
%    

% Numerically solved-for values
xe = model_sim.trimconds.xe;
ue = model_sim.trimconds.ue;


% *************************************************************************
%
% INITIALIZE SETTINGS
% 
% *************************************************************************

% x_e = xe;
% u_e = ue;

% *************************************************************************
%
% INITIALIZE DANAMICAL VARIABLES
% 
% *************************************************************************

% Initialize empty matrices

% Reference trajectory r(t)
rtmat = zeros(size(tvec,1), m);


% *************************************************************************
%
% CALCULATE DANAMICAL VARIABLES
% 
% *************************************************************************

for k = 1:size(tvec,1)
    
    % Get time
    t = tvec(k);

    % Get state vector 
    xs = xmat(k,:)';

    % Get vehicle states
    xv = xs(1:n);

    % Get control
    u = umat(k,:)';

    % Evaluate reference trajectory r(t)
    rt = eval_xr(t, r_sett);

 
    % ***********************
    %
    % EVALUATE FUNCTIONS
    %


    % ***********************
    %
    % STORE
    %

%     rtmat(k,:) = rt';
    rtmat(k,:) = rt(:,1)';


end


% % Reference command data
% yrmat = [rtmat(:,1) rtmat(:,5)];
% Reference command data
yrmat = rtmat;

% Output data
ymat = xmat(:,inds_xr);

% Tracking error
emat = ymat - yrmat;


% *************************************************************************
%
% PACK OUTPUT
% 
% *************************************************************************

out_data.rtmat = rtmat;

% Reference command data
out_data.yrmat = yrmat;

% Output data
out_data.ymat = ymat;

% Tracking error data
out_data.emat = emat;



% *************************************************************************
% *************************************************************************
%
% PRINT OUT PERFORMANCE METRICS
%
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% SETTINGS
%


% Format spec -- scientific notation
fspec_scinot = '%.2e';


if print_metrics
disp('*******************************************************************')
disp('*******************************************************************')
disp('*')
disp(['* DISPLAYING PERFORMANCE METRICS FOR PRESET ' presetcount])
disp('*')
disp('*******************************************************************')
disp('*******************************************************************')
end


% *************************************************************************
%
% TRACKING/STEP RESPONSE METRICS
%
% *************************************************************************

for i = 1:m

    % ***********************
    %
    % GET CURRENT OUTPUT VARIABLE
    %
    
    ind_xri = inds_xr(i);

    y_propts = y_propts_cell{ind_xri};
    y_scl = x_sclvec(ind_xri);

    u_propts = u_propts_cell{i};
    u_scl = u_propts.scale;

    currvarname = y_propts.varname;
    currtexname = y_propts.texname;

    currvarname_u = u_propts.varname;
    currtexname_u = u_propts.texname;

    % Threshold vector
    threshvec = threshmat(:,i);
    threshvec_txt = threshmat_txt(:,i);

    % ***********************
    %
    % GET OUTPUT, TRACKING ERROR
    %
    
    yvec = y_scl * xmat(:,ind_xri);
    y0 = yvec(1);
    yrvec = y_scl * yrmat(:,i);
    if isstep
        yri = y_scl * Arvec(i);
    end
    evec = yvec - yrvec;
    tyvec = yvec - y_scl * xe(ind_xri);

    % ***********************
    %
    % GET CONTROL
    %

    uvec = u_scl * umat(:,i);
    u0 = uvec(1);

    % ***********************
    %
    % (y_{i})_{min}
    %
    
    ymin = min(yvec);
    
    if print_metrics
    disp('*****')
    disp('*****')
    disp(['y_{i,min} =          ' num2str(ymin)])
    end
    
    % Store
    perf_metrics.([currvarname 'min']) = ymin;

    % ***********************
    %
    % (y_{i})_{max}
    %
    
    ymax = max(yvec);
    
    if print_metrics
    disp('*****')
    disp('*****')
    disp(['y_{i,max} =          ' num2str(ymax)])
    end
    
    % Store
    perf_metrics.([currvarname 'max']) = ymax;


    % ***********************
    %
    % (u_{i})_{min}
    %
    
    umin = min(uvec);
    
    if print_metrics
    disp('*****')
    disp('*****')
    disp(['u_{i,min} =          ' num2str(umin)])
    end
    
    % Store
    perf_metrics.([currvarname_u 'min']) = umin;

    % ***********************
    %
    % (u_{i})_{max}
    %
    
    umax = max(uvec);
    
    if print_metrics
    disp('*****')
    disp('*****')
    disp(['u_{i,max} =          ' num2str(umax)])
    end
    
    % Store
    perf_metrics.([currvarname_u 'max']) = umax;

    % ***********************
    %
    % |u_{i}(t) - u_{i}(0)|_{max}
    %
    
    umaxdev = max(abs(uvec - u0));
    
    if print_metrics
    disp('*****')
    disp('*****')
    disp(['|u_{i}(t) - u_{i}(0)|_{max} =          ' num2str(umaxdev)])
    end
    
    % Store
    perf_metrics.([currvarname_u 'maxdev']) = umaxdev;



    % ***********************
    %
    % [(y_{i})(t) / y_i(0)]_max (%)
    %
    
    if y0 == 0
        ymaxdy0 = -1;
    else
        ymaxdy0 = max(abs(yvec-y0)) / y0 * 100;
    end
    
    
    if print_metrics
    disp('*****')
    disp('*****')
    disp(['[(y_{i})(t) / y_i(0)]_max (%) =          ' num2str(ymaxdy0)])
    end
    
    % Store
    perf_metrics.([currvarname 'maxdevpct']) = ymaxdy0;    

    % ***********************
    %
    % e_{y_i,max}
    %
    
    emax = max(abs(evec));
    
    if print_metrics
    disp('*****')
    disp('*****')
    disp(['e_{' currvarname ',max} =          ' num2str(emax)])
    end
    
    % Store
    perf_metrics.(['e' currvarname 'max']) = emax;
    
    % ***********************
    %
    % t_{s, e_{y_i}, THRESHOLD}
    %
    
    for j = 1:numthresh
        % Current value to determine settling time to
        currsetval = threshvec(j);
        
        [ts, ~] = find_ts(tvec, evec, 0, currsetval);
        
        if print_metrics
        disp('*****')
        disp('*****')
        disp(['t_{s, e_{' currtexname '}} TO WITHIN +/- '...
            num2str(currsetval) ...
            ' =          ' num2str(ts)])
        end
        
        % Store
        perf_metrics.(['ts_' currvarname '_' threshvec_txt{j}]) = ts;

    end

    % ***********************
    %
    % t_{s, e_{y_i}, %}
    %
    
    if isstep
    for j = 1:length(tspctvec)
        % Current % value to determine settling time to (%)
        currpct = tspctvec(j);
        
        % Calculate the threshold corresponding to this percentage
        currsetval = abs(yri) * (currpct/100);
        
        [ts, ~] = find_ts(tvec, evec, 0, currsetval);
        
        if print_metrics
        disp('*****')
        disp('*****')
        disp(['t_{s, e_{' currtexname '}, ' num2str(currpct) ...
                    '%} =          ' num2str(ts)])
        end

        % Store
        perf_metrics.(['ts_' currvarname '_' num2str(currpct) 'pct']) = ts;     
    end
    end
    

    % ***********************
    %
    % t_{r, y_i, %}
    %
    
    if isstep
    for j = 1:length(trpctvec)
        % Current % value to determine rise time to (%)
        currpct = trpctvec(j);
        
        [tr, ~] = find_tr(tvec, tyvec, yri, currpct);
        
        if print_metrics
        disp('*****')
        disp('*****')
        disp(['t_{r, ' currtexname ', ' num2str(currpct) ...
                    '%} =          ' num2str(tr)])
        end
        
        % Store
        perf_metrics.(['tr_' currvarname '_' num2str(currpct) 'pct']) = tr;
    end
    end
    
    % ***********************
    %
    % M_{p, y_i}
    %
    
    if isstep
    [Mp, ~] = find_Mp(tvec, tyvec, yri);
    
    if print_metrics
    disp('*****')
    disp('*****')
    disp(['M_{p, ' currtexname '} =          ' num2str(Mp) ' %'])
    end
    
    % Store
    perf_metrics.(['Mp' currvarname]) = Mp;
    end

end

% ***********************
%
% STABILITY
%

if isstep
% Store
perf_metrics.(['stable']) = perf_metrics.(['e' currvarname 'max']) < inf;
end    


% ***********************
%
% STORE SETTLING, RISE TIME THRESHOLDS
%

% Output properties
perf_metrics.y_propts_cell = y_propts_cell;

% Store output variable indices
perf_metrics.inds_xr = inds_xr; 

% Store settling, rise time thresholds
perf_metrics.isstep = isstep;
if isstep
    perf_metrics.trpctvec = trpctvec;
    perf_metrics.tspctvec = tspctvec;
end
perf_metrics.threshmat = threshmat;
perf_metrics.threshmat_txt = threshmat_txt;

% ***********************
%
% STORE PERFORMANCE METRIC DATA
%

if issweep_rand_nu
    perf_metric_cell{mcnt,presetcount} = perf_metrics;
else
    perf_metric_cell{presetcount} = perf_metrics;
    out_data.perf_metrics = perf_metrics;
end

% ***********************
%
% IF IS A SWEEP, DELETE STATE/CONTROL DATA FOR SAVING
%   
if issweep_rand_nu
    out_data = rmfield(out_data,'tvec');
    out_data = rmfield(out_data,'xmat');
    out_data = rmfield(out_data,'umat');
    out_data = rmfield(out_data,'rtmat');
    out_data = rmfield(out_data,'yrmat');
    out_data = rmfield(out_data,'ymat');
    out_data = rmfield(out_data,'emat');
    if strcmp(curralg,algnames.mi_dirl)
        out_data = rmfield(out_data,'loopdata');
        out_data = rmfield(out_data,'loop_cell');
        out_data = rmfield(out_data,'inds');
        out_data = rmfield(out_data,'cmat_cell');
        out_data = rmfield(out_data,'P_cell');
        out_data = rmfield(out_data,'K_cell');
        out_data = rmfield(out_data,'cond_A_vec_cell');
        out_data = rmfield(out_data,'dirl_data');
    end
end

% ***********************
%
% STORE UPDATED OUTPUT DATA CELL
%
if issweep_rand_nu
    out_data_cell{mcnt,presetcount} = out_data;
else
    out_data_cell{presetcount} = out_data;
end

% ***********************
%
% STORE UPDATED ALG SETTINGS CELL
%
if issweep_rand_nu
    alg_settings = [];
    alg_settings_cell{mcnt,presetcount} = alg_settings;
end

end                     % END MAIN LOOP -- presetcount

end                     % END MAIN LOOP -- mcnt



% *************************************************************************
% *************************************************************************
%
% END MAIN LOOP
%
% *************************************************************************
% *************************************************************************



% ***********************
%
% SAVE PERFORMANCE METRIC DATA
%

out_data_cell{1}.perf_metric_cell = perf_metric_cell;

if savefigs
    varname = 'perf_metric_cell';
    filename = 'perf_metric_cell';
    mkdir([relpath_data]);
    save([relpath_data  filename], varname);
end


% ***********************
%
% CREATE/SAVE TABULATED METRIC DATA FOR LATEX TABLE
%

if savefigs

    % Create objects for table
    [metricdatatmat, metricdata_cell, metric_cell] = ...
    make_textable_data(perf_metric_cell);
    
    % Write to struct
    perf_metric_cell_table.metricdatatmat = metricdatatmat;
    perf_metric_cell_table.metricdata_cell = metricdata_cell;
    perf_metric_cell_table.metric_cell = metric_cell;

    % Save struct
    varname = 'perf_metric_cell_table';
    filename = 'perf_metric_cell_table';
    mkdir([relpath_data]);
    save([relpath_data filename], varname);

    % Write settings to input struct. See 'make_tex_table.m' for
    % details
    setts.relpath_data = relpath_data;
    setts.filename_data = filename_data;
    setts.donewline = 1;
    setts.rowpad_cell = rowpad_cell;

    % Call LaTEX table function to write data to .txt file
    make_tex_table(metricdata_cell, metric_cell, setts);

end


% % ***********************
% %
% % CREATE/SAVE LATEX TABLE
% %
% 
% if savefigs
% 
%     % Write settings to input struct. See 'make_tex_table.m' for
%     % details
%     setts.relpath_data = relpath_data;
%     setts.filename_data = filename_data;
%     setts.donewline = 1;
%     setts.rowpad_cell = rowpad_cell;
% 
%     % Call LaTEX table function to write data to .txt file
%     make_tex_table(metricdata_cell, metric_cell, setts);
% 
% end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
%
% FUNCTION: FIND SETTLING TIME TO WITHIN A PRESCRIBED THRESHOLD
%
% *************************************************************************
% *************************************************************************

function [ts, ind_ts] = find_ts(tvec, xvec, xval, threshold)

% Calculate |x - xval|
absxmxval = abs(xvec - xval);

% See where |x - xval| >= threshold
notwithinthreshold = absxmxval >= threshold;
inds_notwithinthreshold = find(notwithinthreshold);

% Check if |x - xval| < threshold for all samples
alwaysinthreshold = isempty(inds_notwithinthreshold);

% If |x - xval| >= threshold for final value in the vector, throw flag
has_settled = ~(notwithinthreshold(end) == 1);

if has_settled

    if alwaysinthreshold
        % Samples are always within threshold. Return t_s = 0
        ind_ts = 1;
        ts = 0;
    else
        % Return the settling time, index of the settling time
        ind_ts = max(inds_notwithinthreshold) + 1;
        ts = tvec(ind_ts);
    end

else

    % Return dummy values 
    % ind_ts = inf;
    % ts = inf;

    ind_ts = length(tvec);
    ts = tvec(ind_ts);

end


%%
% *************************************************************************
% *************************************************************************
%
% FUNCTION: FIND RISE TIME BASED ON TRACKING ERROR
%
% *************************************************************************
% *************************************************************************

function [tr, ind_tr] = find_tr(tvec, xvec, stepsize, pct)

% Calculate settling threshold value
threshold = stepsize * pct/100;

% See where x >= threshold
abovethreshold = xvec >= threshold;
inds_abovethreshold = find(abovethreshold);

% Check if x < threshold for all samples
neverabovethreshold = isempty(inds_abovethreshold);

if ~neverabovethreshold

    % Return the rise time, index of the rise time
    ind_tr = min(inds_abovethreshold);
    tr = tvec(ind_tr);

else

    % Return dummy values
    % ind_tr = inf;
    % tr = inf;

    ind_tr = length(tvec);
    tr = tvec(ind_tr);

end


%%
% *************************************************************************
% *************************************************************************
%
% FUNCTION: FIND PERCENT OVERSHOOT BASED ON TRACKING ERROR
%
% *************************************************************************
% *************************************************************************

function [Mp, ind_Mp] = find_Mp(tvec, xvec, stepsize)

% Find 100% rise time
[tr100, ind_tr100] = find_tr(tvec, xvec, stepsize, 100);

% Check if stepsize = 0
zero_stepsize = stepsize == 0;

% If zero stepsize, return flag
if zero_stepsize

    % Dummy values
    Mp = inf;
    ind_Mp = inf;

else

    % Check if rise time finite. If is infinite, assume Mp = 0 by default
    if isinf(tr100)
    
        % Dummy values
        Mp = 0;
        ind_Mp = 0;
    
    else
    
        % Calulate the percent overshoot. Check the largest error deviation
        % for t > t_r
        xvectgtr = xvec(ind_tr100:end);
        [maxerr, ind_maxerr] = max(xvectgtr);
    
        Mp = (maxerr / abs(stepsize) - 1) * 100;
        ind_Mp = ind_tr100 + ind_maxerr - 1;
    
    
    end

end







