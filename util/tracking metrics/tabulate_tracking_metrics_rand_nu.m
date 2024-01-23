function [out_data_cell, figcount] = tabulate_tracking_metrics_rand_nu...
    (alg_settings_cell, out_data_cell, group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% TABULATE TRACKING METRICS FOR RANDOM \nu SWEEP
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

% Figure counter
figcount = group_settings.figcount;

% Figure counter
lgd_p = group_settings.lgd_p;

% Check if user desired preset-specific formatting
do_indiv_sett = isfield(group_settings, 'indiv_sett_cell');
if do_indiv_sett   
    indiv_sett_cell = group_settings.indiv_sett_cell;
end


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
    relpath = group_settings.relpath;
end

% Print performance metrics (=1) or don't print (=0)
print_metrics = group_settings.print_metrics;

% Reference signal r(t) settings
if ~group_settings.istraining
    r_sett = group_settings.r_sett;
else
    r_sett = alg_settings_cell{1}.r_sett_train;
end

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

% Load the previously-calculated performance metrics cell
perf_metric_cell = out_data_cell{1}.perf_metric_cell;

% Check if step velocity reference command was issued
isstep = isfield(r_sett,'Arvec');

% Output properties
y_propts_cell = master_settings.sys_plot_settings.y_propts_cell;

% Control properties
u_propts_cell = master_settings.sys_plot_settings.u_propts_cell;

% Check which loop the step is being applied to 
if isstep
    indstep = group_settings.indstep; 
    ind_xri = inds_xr(indstep);
    tmp = (1:m)';
    indsnotstep = tmp(tmp ~= indstep);

    y_propts = y_propts_cell{ind_xri};

    u_propts = u_propts_cell{indstep};

    currvarname = y_propts.varname;
    currtexname = y_propts.texname;
end

% State trajectory x(t) unit scaling. E.g., if x_1(t) is angular
% displacement but is desired in degrees, declare the field 'sclvec' and
% put sclvec(1) = 180/pi.
x_sclvec = group_settings.sys_plot_settings.x_sclvec;

% Get output, control check thresholds
if isstep
    trpctvec = group_settings.trpctvec;
    tspctvec = group_settings.tspctvec;
    numtr = size(trpctvec,1);
    numts = size(tspctvec,1);
end
threshmat = group_settings.threshmat;
threshmat_txt = group_settings.threshmat_txt;
numthresh = size(threshmat,1);


% % Get output, control check thresholds
% threshmat_check_y = group_settings.threshmat_check_y;
% threshmat_check_y_abs1_pct_0 = group_settings.threshmat_check_y_abs1_pct_0;
% threshmat_check_u = group_settings.threshmat_check_u;
% 
% numthresh_y = size(threshmat_check_y,1);
% numthresh_u = size(threshmat_check_u,1);

systag = master_settings.systag;
sysnames = master_settings.sysnames;

% *************************************************************************
% 
% THRESHOLDS
% 
% *************************************************************************

% Initialize empty threshold check cell
data_check_cell = [];

% Threshold times to check for settling time in each loop
tscheck_cell = cell(m, numts);        
switch systag
    case {sysnames.hsv}
        tscheck_cell{1,1} = [25; 50];
        tscheck_cell{1,2} = [75; 100];                
        tscheck_cell{2,1} = [7.5; 10];
        tscheck_cell{2,2} = [10; 15];
    otherwise
end

% Threshold times to check for rise time in each loop
trcheck_cell = cell(m, numtr);        
switch systag
    case {sysnames.hsv}
        trcheck_cell{1,1} = [25; 30];
        trcheck_cell{2,1} = [5; 7.5];                
    otherwise
end

% Threshold percentage overshoots to check for in each loop
Mpcheck_cell = cell(m);        
switch systag
    case {sysnames.hsv}
        Mpcheck_cell{1} = [5; 10];
        Mpcheck_cell{2} = [5; 10];
    otherwise
end

% Threshold max control deviations to check for in each loop
ucheck_cell = cell(m,m);        
switch systag
    case {sysnames.hsv}
        ucheck_cell{1,1} = [0.2; 0.25];
        ucheck_cell{1,2} = [0.25; 0.5];
        ucheck_cell{2,1} = [0.25; 0.5];
        ucheck_cell{2,2} = [5; 7.5];
    otherwise
end

% Threshold max output deviations to check for in each loop
% Element {i,1}: Threshold check vector
% Element {i,2}: Is an absolute deviation (=1) or percent deviation (=0)
ycheck_cell = cell(m,2);  
switch systag
    case {sysnames.hsv}
        ycheck_cell{1,1} = [0.01; 0.05];
        ycheck_cell{1,2} = 1;        
        ycheck_cell{2,1} = [0.2; 0.3];
        ycheck_cell{2,2} = 0;
    otherwise
end

% *************************************************************************
% 
% ADD THRESHOLDS -- STABILITY
% 
% *************************************************************************

if indstep == 1
tmpmetric.name = ['stable'];
tmpmetric.checkvec = 1;
tmpmetric.checktype = '=';

% Append to array
data_check_cell = [data_check_cell; {tmpmetric}];
end


% *************************************************************************
% 
% ADD THRESHOLDS -- t_s
% 
% *************************************************************************

for i = 1:numts

    % Current % value to determine settling time to (%)
    currpct = tspctvec(i);

    tmpmetric.name = ['ts_' currvarname '_' num2str(currpct) 'pct'];
    tmpmetric.checkvec = tscheck_cell{indstep,i};
    tmpmetric.checktype = '<';

    % Append to array
    data_check_cell = [data_check_cell; {tmpmetric}];

end

% *************************************************************************
% 
% ADD THRESHOLDS -- t_r
% 
% *************************************************************************

for i = 1:numtr

    % Current % value to determine rise time to (%)
    currpct = trpctvec(i);

    tmpmetric.name = ['tr_' currvarname '_' num2str(currpct) 'pct'];
    tmpmetric.checkvec = trcheck_cell{indstep,i};
    tmpmetric.checktype = '<';

    % Append to array
    data_check_cell = [data_check_cell; {tmpmetric}];

end

% *************************************************************************
% 
% ADD THRESHOLDS -- M_p
% 
% *************************************************************************

tmpmetric.name = ['Mp' currvarname];
tmpmetric.checkvec = Mpcheck_cell{indstep,i};
tmpmetric.checktype = '<';

% Append to array
data_check_cell = [data_check_cell; {tmpmetric}];

% *************************************************************************
% 
% ADD THRESHOLDS -- |u_{i}(t) - u_{i}(0)|_{max}
% 
% *************************************************************************

for i = 1:m

    % Current u variable name
    currvarname_u = u_propts_cell{i}.varname;

    tmpmetric.name = [currvarname_u 'maxdev'];
    tmpmetric.checkvec = ucheck_cell{indstep,i};
    tmpmetric.checktype = '<';

    % Append to array
    data_check_cell = [data_check_cell; {tmpmetric}];

end

% *************************************************************************
% 
% ADD THRESHOLDS -- OUTPUT DEVIATIONS
% 
% *************************************************************************

for i = 1:m-1

    currind = indsnotstep(i);

    currvarname = y_propts_cell{inds_xr(currind)}.varname;

    if ycheck_cell{indstep,2}
        tmpmetric.name = ['e' currvarname 'max'];
    else
        tmpmetric.name = [currvarname 'maxdevpct'];
    end

    tmpmetric.checkvec = ycheck_cell{indstep,i};
    tmpmetric.checktype = '<';

    % Append to array
    data_check_cell = [data_check_cell; {tmpmetric}];

end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN CALCULATION LOOP
%
% Loop over the metrics
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

nummetric = size(data_check_cell,1);
numthresh = max([numts numtr]);

% Holds bar graph data
bardata_success = [];
bardata_fail = [];

% Counter for total number of metrics
nummetric_tot = 0;

for i = 1:nummetric

    % Get the current metric
    currmetric = data_check_cell{i};
    currname = currmetric.name;
    currcheckvec = currmetric.checkvec;
    currchecktype = currmetric.checktype;

    % Number of checks to make
    numcurrcheck = size(currcheckvec,1);

    % Get the current data
    currdata = get_field_cell(perf_metric_cell, currname);

    disp('% *************************************************************')
    disp('%')
    disp(['% DISPLAYING DATA FOR: '  currname])
    disp('%')
    disp('% *************************************************************')

    disp('***** MEAN *****')
    disp('*')

    dispamp(mean(currdata))

    disp('***** MAX *****')
    disp('*')

    dispamp(max(currdata))    
    
    disp('***** MIN *****')
    disp('*')

    dispamp(min(currdata))   

    disp('***** STD *****')
    disp('*')

    dispamp(std(currdata)) 

    % ***********************
    %
    % DISPLAY FAILURE PERCENTAGE
    %

    for j = 1:numcurrcheck

        % Get current check threshold
        currcheckthresh = currcheckvec(j);

        % Get the successful trials
        switch currchecktype
            case '<'
                successmat = currdata < currcheckthresh;
            case '='
                successmat = currdata == currcheckthresh;
            case '>'
                successmat = currdata > currcheckthresh;
        end
        

        % Success ratio
        successratio = sum(successmat) / nummodels;

        % Success percentage
        successpct = successratio * 100;
        failpct = (1 - successratio) * 100;

        % Display
%         disp(['***** SUCCESS PERCENTAGE: THRESHOLD = ' ...
%             num2str(currcheckthresh) ' *****'])
%         disp('*')   
%         dispamp(successpct);

        disp(['***** FAILURE PERCENTAGE: THRESHOLD = ' ...
            num2str(currcheckthresh) ' *****'])
        disp('*')   
        dispamp(failpct);


        % Store
        bardata_success = [bardata_success ; successpct];
        bardata_fail = [bardata_fail ; failpct];

    end

    % Increment total number of metrics
    nummetric_tot = nummetric_tot + numcurrcheck;

end

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN PLOT LOOP
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Number the metrics depending on the loop
if indstep == 1
    metricinds = (1:nummetric_tot)';
else
    metricinds = (1:nummetric_tot)' + (indstep - 1) * nummetric_tot + 1;
end

for i = 1:2

    switch i
        case 1
            currstrng = 'Success';
            currstrng_filename = 'success';
            bardata = bardata_success;
            p_sett.custom_sett.lgd_position = ...
                [0.1375    0.6763    0.1756    0.1960];
        case 2
            currstrng = 'Failure';
            currstrng_filename = 'fail';
            bardata = bardata_fail;
            p_sett.custom_sett.lgd_loc = 'northeast';
    end

    figure(figcount)
    
    % PLOT
    h_fig = bar(metricinds, bardata); 
    
    ttl = ['Performance Metric ' currstrng ' Percentages -- $' ...
        currtexname '$' ];
    title(ttl)          
    xlabel('Performance Metric Number');
    ylabel([currstrng ' Percentage (\%)']);
    
    lgd = legend(lgd_p);  
    
    % Format plot
    p_sett.figcount = figcount;    
    if do_indiv_sett
        p_sett.indiv_sett_cell = indiv_sett_cell;
    end
    plot_format(p_sett);   
    clear p_sett;
    
    % SAVE PLOT
    if savefigs
        filename = [currstrng_filename '_pct'];
        savepdf(figcount, relpath, filename); 
    end
    
    % Increment figure counter
    figcount = figcount + 1; 

end




