function [out_data_cell] = hsv_wang_stengel_2000_calc...
    (alg_settings_cell, out_data_cell, group_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% HSV PITCH-AXIS MODEL -- CALCULATE VEHICLE DATA FROM SIMULATION DATA
%
% Brent Wallace  
%
% 2022-10-19
%
% This program, given an HSV model and simulation data, calculates
% aerodynamic forces, moments, etc. Model is from:
%
%   Q. Wang and R. F. Stengel. "Robust nonlinear control of a hypersonic
%   aircraft." AIAA J. Guid., Contr., & Dyn., 23(4):577–585, July 2000
%
% And:
%
%   C. I. Marrison and R. F. Stengel. "Design of robust control systems for
%   a hypersonic aircraft." AIAA J. Guid., Contr., & Dyn., 21(1):58–63,
%   Jan. 1998.
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
r_sett = group_settings.r_sett;

% % Thresholds to determine settling time
% thres_eV = group_settings.thres_eV;
% thres_eh = group_settings.thres_eh;

% System
sys = group_settings.sys;             % System array
n = sys.n;                          % System order
m = sys.m;                          % System input dimension

% System cell array
model_cell = sys.model_cell;

% Indices of nominal model
indnom = sys.indnom;

% Nominal model
model = get_elt_multidim(model_cell, indnom); 

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% Indices of how state vector is partitioned
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
y_propts_cell = group_settings.sys_plot_settings.y_propts_cell;

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


% ***********************
%       
% DEFINE SYMBOLIC VARIABLES
%      

% x = [V, \gamma, h, \alpha, q] in R^{5}
xs = sym('x', [1 5], 'real')';

% x_v = [V, \gamma, h, \alpha, q, \delta_{T}, \dot{\delta}_{T}] in R^{7}
xvs = sym('xv', [1 7], 'real')';

% u in R^{m}
us = sym('u', [1 m], 'real')'; 

% [x_v, u] in R^{7+m}
xvus = [    xvs
            us   ];

% Index variables (i.e., where each state is in x)
indV = 1;
indg = 2;
indh = 3;
inda = 4;
indq = 5;
inddT = 6;
indddT = 7;
% indVI = 8;
% indhI = 9;

% Index variables (i.e., where each control is in [xv u])
inddTc = 8;
inddE = 9;


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
perf_metric_cell = cell(numpresets,1);

%%
% *************************************************************************
% *************************************************************************
%
% HSV METRICS -- INDIVIDUAL PRESETS
%
% *************************************************************************
% *************************************************************************

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
alg_settings = alg_settings_cell{presetcount};

% Get current out_data struct
out_data = out_data_cell{presetcount};

% ***********************
%       
% PRESET SETTINGS
%   

% Model linear (=1) or nonlinear (=0)
lin1nonlin0 = alg_settings.lin1nonlin0;

% % Plant model MP (=1) or NMP (=0)
% Pmp1nmp0 = alg_settings.Pmp1nmp0;

% % Controller design that for MP plant (=1) or NMP plant (=0)
% Kmp1nmp0 = alg_settings.Kmp1nmp0;

% Algorithm is NDI (=1) or not NDI (=0)
% % ndi1notndi0 = alg_settings.ndi1notndi0;
ndi1notndi0 = 0;


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

% Simulation model
% % model_sim_tag = alg_settings.model_sim_tag;
% % switch model_sim_tag
% %     case 'default'
% %         model_sim = sys.model;
% %     case 'perturbed'
% %         model_sim = sys.model_nu;
% % end
model_sim_ind = alg_settings.model_sim_ind;
model_sim =  get_elt_multidim(model_cell, model_sim_ind);

% ***********************
%       
% SYSTEM PARAMETERS
%      

% Unit conversions
sy = model_sim.lin.io.syd;

g = model_sim.g;
mu = model_sim.mu;
RE = model_sim.RE;
mref = model_sim.mref;
wref = model_sim.wref;
Iyy = model_sim.Iyy;
S = model_sim.S;
cbar = model_sim.cbar;

% ***********************
%       
% AERO FUNCTIONS
%    

% r = model_sim.r;
% rho = model_sim.rho;
% a = model_sim.a;
% M = model_sim.M;
% 
% CL = model_sim.CL;
% CD = model_sim.CD;
% 
% c_CMdE = model_sim.c_CMdE;
% CMdEu = model_sim.CMdEu;
% CMdE0 = model_sim.CMdE0;
% CMdE = model_sim.CMdE;
% 
% kxv = model_sim.kxv;
% CT = model_sim.CT;
% 
% qinf = model_sim.qinf;
% 
T = model_sim.T;
Tu = model_sim.Tu;
%
% L = model_sim.L;
% D = model_sim.D;
% 
Myy0 = model_sim.Myy0;
Myyu = model_sim.Myyu;
Myy = model_sim.Myy;

% ELEVATOR-INDUCED LIFT EFFECTS
LdE0 = model_sim.LdE0;
LdEu = model_sim.LdEu;
LdE = model_sim.LdE;
dgudE = model_sim.dgudE;

% ***********************
%       
% INLINE FUNCTIONS
%     

% % Inline function f(x)
% fil = model_sim.fil;
% 
% % Inline function g(x) -- depends on MP or NMP model
% if mp1nmp0
%     gil = model_sim.gil;
% else
%     gil = model_sim.gildE;
% end

% r(h)
ril = model_sim.ril;

% Dynamic pressure q_{\infty}
qinfil = model_sim.qinfil;

% Air density \rho
rhoil = model_sim.rhoil;

% Mach number M
Mil = model_sim.Mil;

% L(x), D(x), T(x)
Lil = model_sim.Lil;
Dil = model_sim.Dil;
Til = model_sim.Til;

% Lift L -- WITH ELEVATOR LIFT EFFECTS
LdEil = model_sim.LdEil;

% M_{yy}
Myyil = model_sim.Myyil;

% k(x_v) used for thrust calculation
kil = model_sim.kil;


% L(x) -- partial derivatives
L_Vil = model_sim.L_Vil;
L_ail = model_sim.L_ail;
L_hil = model_sim.L_hil;

% D(x) -- partial derivatives
D_Vil = model_sim.D_Vil;
D_ail = model_sim.D_ail;
D_hil = model_sim.D_hil;

% T(x) -- partial derivatives
T_Vil = model_sim.T_Vil;
T_ail = model_sim.T_ail;
T_hil = model_sim.T_hil;

% M_{yy}|_{u=0} -- partial derivatives
Myy0_Vil = model_sim.Myy0_Vil;
Myy0_hil = model_sim.Myy0_hil;
Myy0_ail = model_sim.Myy0_ail;
Myy0_qil = model_sim.Myy0_qil;

% ***********************
%       
% TRIM CONDITIONS
%
% x_v = [V, \gamma, h, \alpha, q, \delta_{T}, \dot{\delta}_{T}] in R^{7}
%    

% % Wang, Stengel values
% xve = [15060 ; 0 ; 110e3 ; 0.0315 ; 0 ; 0.183 ; 0];
% ue = [0.183 ; -0.0066];

% Numerically solved-for values
xe = model_sim.trimconds.xe;
ue = model_sim.trimconds.ue;

% % Numerically solved-for values -- WITH ELEVATOR-LIFT EFFECTS
% xe_dE = model_sim.trimconds.xe_dE;
% ue_dE = model_sim.trimconds.ue_dE;

% ***********************
%
% LINEARIZATION TERMS
%

% % Linearization params
% lin = model_sim.lin;
% 
% % A, B, C, D matrix -- nominal MP model
% Ap = lin.Ap;
% Bp = lin.Bp;
% Cvh = lin.Cvh;
% Cvg = lin.Cvg;
% Dp = lin.Dp;
% 
% % Plant ss objects -- nominal MP model
% Pvh = lin.Pvh;          % Plant y = [V, h]^T
% Pvg = lin.Pvg;          % Plant y = [V, \gamma]^T
% Px = lin.Px;            % Plant y = x
% 
% % A, B, C, D matrix -- WITH ELEVATOR-LIFT EFFECTS
% ApdE = lin.ApdE;
% BpdE = lin.BpdE;
% 
% % Plant ss objects -- WITH ELEVATOR-LIFT EFFECTS
% PdEvh = lin.PdEvh;      % Plant y = [V, h]^T
% PdEvg = lin.PdEvg;      % Plant y = [V, \gamma]^T
% PdEx = lin.PdEx;        % Plant y = x



% *************************************************************************
%
% INITIALIZE SETTINGS
% 
% *************************************************************************

% Get proper trim states, controls corresponding to the plant selection
% % if Kmp1nmp0
% %     x_e = xe;
% %     u_e = ue;
% % else
% %     x_e = xe_dE;
% %     u_e = ue_dE;
% % end

% x_e = xe_dE;
% u_e = ue_dE;

x_e = xe;
u_e = ue;


% Evaluate partial derivative terms at trim and trim values
if lin1nonlin0

    % Trim state with zeros appended to the last two terms
    xe0 = [x_e; zeros(2,1)];
    
    % L(x) -- partial derivatives, trim value
%     if Pmp1nmp0
%         Le = Lil(xe0);
%     else
%         Le = LdEil([xe0; u_e]);
%         LdEue = double(subs(LdEu,xvs,xe0));
%     end
    Le = LdEil([xe0; u_e]);
    LdEue = double(subs(LdEu,xvs,xe0));   
    L_Ve = L_Vil(xe0);
    L_ae = L_ail(xe0);
    L_he = L_hil(xe0);
    
    % D(x) -- partial derivatives, trim value
    De = Dil(xe0);
    D_Ve = D_Vil(xe0);
    D_ae = D_ail(xe0);
    D_he = D_hil(xe0);
    
    % T(x) -- partial derivatives, trim value
    Te = Til([x_e; u_e]);
    Tue = double(subs(Tu,xvs,xe0));
    T_Ve = T_Vil(xe0);
    T_ae = T_ail(xe0);
    T_he = T_hil(xe0);

    % M_{yy}(x) -- partial derivatives, trim value
    Myye = Myyil([xe0; u_e]);
    Myyue = double(subs(Myyu,xvs,xe0));
    Myy0_Ve = Myy0_Vil(xe0);
    Myy0_ae = Myy0_ail(xe0);
    Myy0_he = Myy0_hil(xe0);   
    Myy0_qe = Myy0_qil(xe0); 

end


% *************************************************************************
%
% INITIALIZE DANAMICAL VARIABLES
% 
% *************************************************************************

% Initialize empty matrices

% Reference trajectory r(t)
rtmat = zeros(size(tvec,1), m);

qinfvec = zeros(size(tvec,1), 1);
rhovec = zeros(size(tvec,1), 1);
Mvec = zeros(size(tvec,1), 1);
Lvec = zeros(size(tvec,1), 1);
Dvec = zeros(size(tvec,1), 1);
Tvec = zeros(size(tvec,1), 1);
Myyvec = zeros(size(tvec,1), 1);

if ~lin1nonlin0
    fgvec = zeros(size(tvec,1), 1);
    kvec = zeros(size(tvec,1), 1);
end

% G* terms
if ndi1notndi0
    detGstaril = group_settings.controller.detGstaril;
    detGstaruil = group_settings.controller.detGstaruil;
    detGstaraeroil = group_settings.controller.detGstaraeroil;
    Gstaril = group_settings.controller.Gstaril;
end

% NDI terms
if ndi1notndi0
    detGstarvec = zeros(size(tvec,1), 1);
    detGstaruvec = zeros(size(tvec,1), 1);
    detGstaraerovec = zeros(size(tvec,1), 1);
    condGstarvec = zeros(size(tvec,1), 1);
end


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
    if ndi1notndi0
        xv = xs(1:7);
    else
        % Throttle setting \delta_T is a control. System is fifth-order.
        % Put zeros in slots for states \delta_T, \dot{\delta}_T
        xv = [xs(1:5); zeros(2,1)];
    end

    % Get control
    u = umat(k,:)';

%     % Evaluate reference trajectory r(t)
%     rt = eval_xr(t, r_sett);

    % Calculate distance to earth center
    rk = ril(xv);

    % Calculate dynamic pressure
    qinfk = qinfil(xv); 

    % Calculate air density
    rhok = rhoil(xv);

    % Mach number M
    Mk = Mil(xv);
    
    % ***********************
    %
    % EVALUATE FUNCTIONS
    %

    if lin1nonlin0

        % ***********************
        %
        % EVALUATE FUNCTIONS -- LINEAR CASE
        %

        % Calculate \tilde{x} = x - x_e
        tx = xv(1:5) - x_e;
        tV = tx(indV);
        ta = tx(inda);
        th = tx(indh);
        tq = tx(indq);

        % Calculate \tilde{u} = u - u_e
        tu = u - u_e;
        tT = tu(1);
        tdE = tu(2);

        % Increment to lift L
        dL = L_Ve * tV + L_he * th + L_ae * ta;
%         if ~Pmp1nmp0
%             dL = dL + LdEue * tdE;
%         end
        dL = dL + LdEue * tdE;
        % Total lift L = L_0 + d_L
        Lxv = Le + dL;

        % Increment to drag D
        dD = D_Ve * tV + D_he * th + D_ae * ta;
        % Total drag D = D_0 + d_D
        Dxv = De + dD;        

        % Increment to thrust T
        % NOTE: Thrust terms are isolated to the "B" matrix, so partials
        % with respect to the state do not enter in the linear system's
        % model of the thrust
%         dT = T_Ve * tV + T_he * th + T_ae * ta;
%         dT = dT + Tue * tT;
        dT = Tue * tT;
        % Total thrust T = T_0 + d_T
        Txv = Te + dT;    

        % Increment to pitching moment
        dMyy = Myy0_Ve * tV + Myy0_he * th + Myy0_ae * ta + Myy0_qe * tq;
        dMyy = dMyy + Myyue * tdE;
        % Total pitching moment M_{yy} = M_{yy,0} + d_T
        Myyxvu = Myye + dMyy;   

    else

        % ***********************
        %
        % EVALUATE FUNCTIONS -- NONLINEAR CASE
        %
        
        % Lift L
%         if Pmp1nmp0
%             Lxv = Lil(xv);
%         else
%             Lxv = LdEil([xv; u]);
%         end
        Lxv = LdEil([xv; u]);

        % Drag D
        Dxv = Dil(xv);

        % Thrust T
        if ndi1notndi0
            Txv = Til(xv);
        else
            Txv = Til([xs(1:5); u]);
        end

        % Pitching moment M_{yy}
        Myyxvu = Myyil([xv; u]);

        % Force of gravity
        fgk = mu * mref / rk^2;

        % Term k(x) in thrust calc
        k_xv = kil(xv);

    end

    % ***********************
    %
    % EVALUATE FUNCTIONS -- NDI ONLY
    %

    if ndi1notndi0
        detGstarxv = detGstaril(xv);
        detGstaruxv = detGstaruil(xv);
        detGstaraeroxv = detGstaraeroil(xv);
        condGstarxv = cond(Gstaril(xv));
    end


    % ***********************
    %
    % STORE
    %

%     rtmat(k,:) = rt';
%     rtmat(k,:) = rt(:,1)';

    qinfvec(k) = qinfk;
    Mvec(k) = Mk;
    rhovec(k) = rhok;
    Lvec(k) = Lxv;
    Dvec(k) = Dxv;
    Tvec(k) = Txv;
    Myyvec(k) = Myyxvu;

    if ~lin1nonlin0
        fgvec(k) = fgk;
        kvec(k) = k_xv;
    end

    if ndi1notndi0
        detGstarvec(k) = detGstarxv;
        detGstaruvec(k) = detGstaruxv;
        detGstaraerovec(k) = detGstaraeroxv;
        condGstarvec(k) = condGstarxv;
    end

    

end

% *************************************************************************
%
% PERFORM CALULATIONS ON DATA
% 
% *************************************************************************

% AOA,FPA data
alphavec_r = xmat(:,inda);
gammavec_r = xmat(:,indg);
alphavec = R2D*alphavec_r;
gammavec = R2D*gammavec_r;
thetavec = gammavec + alphavec;

LDvec = Lvec./Dvec;

Nvec = Lvec.*cos(alphavec_r) + Dvec.*sin(alphavec_r);
loadvec = Nvec / (mref * g);

if ~lin1nonlin0
    sumFLvec = Tvec .* sin(alphavec_r) + Lvec - fgvec .* cos(gammavec_r);
    sumFDvec = Tvec .* cos(alphavec_r) - Dvec - fgvec .* sin(gammavec_r);
end

% Velocity, altitude data
Vvec = xmat(:,indV);
hvec = xmat(:,indh);

% % % Reference command data
% % yrmat = [rtmat(:,1) rtmat(:,5)];
% % Reference command data
% yrmat = rtmat;
% 
% % Output data
% ymat = xmat(:,inds_xr);
% 
% % Tracking error
% emat = ymat - yrmat;


% \delta_{T}, \delta_{T,com}
dTcomvec = umat(:,1);
if ndi1notndi0
    dTvec = xmat(:,inddT);
else
    dTvec = dTcomvec;
end

% *************************************************************************
%
% PACK OUTPUT
% 
% *************************************************************************

% out_data.rtmat = rtmat;

out_data.qinfvec = qinfvec;
out_data.rhovec = rhovec;
out_data.Mvec = Mvec;
out_data.Lvec = Lvec;
out_data.Dvec = Dvec;
out_data.Tvec = Tvec;
out_data.Myyvec = Myyvec;

if ~lin1nonlin0
    out_data.fgvec = fgvec;
    out_data.kvec = kvec;
end

if ndi1notndi0
    out_data.detGstarvec = detGstarvec;
    out_data.detGstaruvec = detGstaruvec;
    out_data.detGstaraerovec = detGstaraerovec;
    out_data.condGstarvec = condGstarvec;
end

out_data.LDvec = LDvec;

out_data.Nvec = Nvec;
out_data.loadvec = loadvec;

if ~lin1nonlin0
    out_data.sumFLvec = sumFLvec;
    out_data.sumFDvec = sumFDvec;
end

% Velocity, altitude data
out_data.Vvec = Vvec;
out_data.hvec = hvec;

% FPA, AOA, pitch data
out_data.alphavec = alphavec;
out_data.gammavec = gammavec;
out_data.thetavec = thetavec;

% % Reference command data
% out_data.yrmat = yrmat;
% 
% % Output data
% out_data.ymat = ymat;
% 
% % Tracking error data
% out_data.emat = emat;

% \delta_{T}, \delta_{T,com}
out_data.dTcomvec = dTcomvec;
out_data.dTvec = dTvec;




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

% ***********************
%
% EXTRACT TRACKING METRICS CALCULATED ABOVE
%

perf_metrics = out_data.perf_metrics;


% ***********************
%
% V_min, V_max
%

Vmin = min(Vvec);
Vmax = max(Vvec);

if print_metrics
disp('*****')
disp('*****')
disp(['V_{min} =          ' num2str(Vmin)])
disp(['V_{max} =          ' num2str(Vmax)])
end

% Store
perf_metrics.Vmin = Vmin;
perf_metrics.Vmax = Vmax;

% ***********************
%
% \gamma_min, \gamma_max
%

gammamin = min(gammavec);
gammamax = max(gammavec);

if print_metrics
disp('*****')
disp('*****')
disp(['gamma_{min} =          ' num2str(gammamin)])
disp(['gamma_{max} =          ' num2str(gammamax)])
end

% Store
perf_metrics.gammamin = gammamin;
perf_metrics.gammamax = gammamax;

% ***********************
%
% h_min, h_max
%

hmin = min(hvec);
hmax = max(hvec);

if print_metrics
disp('*****')
disp('*****')
disp(['h_{min} =          ' num2str(hmin)])
disp(['h_{max} =          ' num2str(hmax)])
end

% Store
perf_metrics.hmin = hmin;
perf_metrics.hmax = hmax;

% ***********************
%
% \alpha_min, \alpha_max
%

alphamin = min(alphavec);
alphamax = max(alphavec);

if print_metrics
disp('*****')
disp('*****')
disp(['alpha_{min} =          ' num2str(alphamin)])
disp(['alpha_{max} =          ' num2str(alphamax)])
end

% Store
perf_metrics.alphamin = alphamin;
perf_metrics.alphamax = alphamax;

% ***********************
%
% \d_Tmin, \d_Tmax
%


dTmin = min(dTvec);
dTmax = max(dTvec);

if print_metrics
disp('*****')
disp('*****')
disp(['delta_{T,min} =          ' num2str(dTmin)])
disp(['delta_{T,max} =          ' num2str(dTmax)])
end

% Store
perf_metrics.dTmin = dTmin;
perf_metrics.dTmax = dTmax;


% ***********************
%
% T_min, T_max
%

Tmin = min(Tvec);
Tmax = max(Tvec);

if print_metrics
disp('*****')
disp('*****')
disp(['T_{min} =          ' num2str(Tmin,fspec_scinot)])
disp(['T_{max} =          ' num2str(Tmax,fspec_scinot)])
end

% Store
perf_metrics.Tmin = Tmin;
perf_metrics.Tmax = Tmax;

% ***********************
%
% \d_Emin, \d_Emax
%

dEvec = R2D * umat(:,2);
dEmin = min(dEvec);
dEmax = max(dEvec);

if print_metrics
disp('*****')
disp('*****')
disp(['delta_{E,min} =          ' num2str(dEmin)])
disp(['delta_{E,max} =          ' num2str(dEmax)])
end

% Store
perf_metrics.dEmin = dEmin;
perf_metrics.dEmax = dEmax;

% ***********************
%
% q_{\infty,min}, q_{\infty,max}
%

qinfmin = min(qinfvec);
qinfmax = max(qinfvec);

if print_metrics
disp('*****')
disp('*****')
disp(['q_{\infty,min} =          ' num2str(qinfmin,fspec_scinot)])
disp(['q_{\infty,max} =          ' num2str(qinfmax,fspec_scinot)])
end

% Store
perf_metrics.qinfmin = qinfmin;
perf_metrics.qinfmax = qinfmax;


% ***********************
%
% L_min, L_max
%

Lmin = min(Lvec);
Lmax = max(Lvec);

if print_metrics
disp('*****')
disp('*****')
disp(['L_{min} =          ' num2str(Lmin,fspec_scinot)])
disp(['L_{max} =          ' num2str(Lmax,fspec_scinot)])
end

% Store
perf_metrics.Lmin = Lmin;
perf_metrics.Lmax = Lmax;

% ***********************
%
% D_min, D_max
%

Dmin = min(Dvec);
Dmax = max(Dvec);

if print_metrics
disp('*****')
disp('*****')
disp(['D_{min} =          ' num2str(Dmin,fspec_scinot)])
disp(['D_{max} =          ' num2str(Dmax,fspec_scinot)])
end

% Store
perf_metrics.Dmin = Dmin;
perf_metrics.Dmax = Dmax;

% ***********************
%
% L/D_min, L/D_max
%

LDmin = min(LDvec);
LDmax = max(LDvec);

if print_metrics
disp('*****')
disp('*****')
disp(['(L/D)_{min} =          ' num2str(LDmin)])
disp(['(L/D)_{max} =          ' num2str(LDmax)])
end

% Store
perf_metrics.LDmin = LDmin;
perf_metrics.LDmax = LDmax;


% ***********************
%
% loading_min, loading_max
%


loadmin = min(loadvec);
loadmax = max(loadvec);

if print_metrics
disp('*****')
disp('*****')
disp(['(loading)_{min} =          ' num2str(loadmin) ' g'])
disp(['(loading)_{max} =          ' num2str(loadmax) ' g'])
end

% Store
perf_metrics.loadmin = loadmin;
perf_metrics.loadmax = loadmax;


if ~lin1nonlin0

% ***********************
%
% (\sum F_L)_min, (\sum F_L)_max
%

sumFLmin = min(sumFLvec);
sumFLmax = max(sumFLvec);

if print_metrics
disp('*****')
disp('*****')
disp(['(\sum F_L)_{min} =          ' num2str(sumFLmin,fspec_scinot)])
disp(['(\sum F_L)_{max} =          ' num2str(sumFLmax,fspec_scinot)])
end

% Store
perf_metrics.sumFLmin = sumFLmin;
perf_metrics.sumFLmax = sumFLmax;


% ***********************
%
% (\sum F_D)_min, (\sum F_D)_max
%

sumFDmin = min(sumFDvec);
sumFDmax = max(sumFDvec);

if print_metrics
disp('*****')
disp('*****')
disp(['(\sum F_D)_{min} =          ' num2str(sumFDmin,fspec_scinot)])
disp(['(\sum F_D)_{max} =          ' num2str(sumFDmax,fspec_scinot)])
end

% Store
perf_metrics.sumFDmin = sumFDmin;
perf_metrics.sumFDmax = sumFDmax;

end

% ***********************
%
% |G*(x)|_min, |G*(x)|_max
%

if ndi1notndi0

detGstarmin = min(detGstarvec);
detGstarmax = max(detGstarvec);

if print_metrics
disp('*****')
disp('*****')
disp(['|G*(x)|_{min} =          ' num2str(detGstarmin,fspec_scinot)])
disp(['|G*(x)|_{max} =          ' num2str(detGstarmax,fspec_scinot)])
end

% Store
perf_metrics.detGstarmin = detGstarmin;
perf_metrics.detGstarmax = detGstarmax;

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

perf_metric_cell{presetcount} = perf_metrics;
out_data.perf_metrics = perf_metrics;


% ***********************
%
% STORE UPDATED OUTPUT DATA CELL
%

out_data_cell{presetcount} = out_data;

end                     % END MAIN LOOP


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








