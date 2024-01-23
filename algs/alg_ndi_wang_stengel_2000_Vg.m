function out_data = alg_ndi_wang_stengel_2000_Vg(alg_settings, ...
    group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BACKSTEPPING NONLINEAR CONTROL
%
% Brent Wallace  
%
% 2022-08-13
%
% This program implements nonlinear dynamic inversion (NDI) for a
% hypersonic vehicle as developed in
%
%   Wang, Qian, and Robert F. Stengel. "Robust nonlinear control of a 
%   hypersonic aircraft." Journal of guidance, control, and dynamics 
%   23.4 (2000): 577-585.
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% out_data = alg_ndi_wang_stengel_2000(alg_settings)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings  struct with the following fields:
%   
%   preset                  (String) example preset (see main.m for
%                           options).
%   sys                     (Struct) contains system tag/info. See notes in
%                           'config.m' for specific fields.
%   alg                     (String) redundant for this function. Contains
%                           the tag of this algorithm.
%   u_sett                  (Struct) contains parameters for the
%                           passivity-based control u(x). Fields are
%                           example-specific.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) algorithm output data. Has the
%                           following fields:
%       .tvec               ('simlength'-dimensional vector) vector of time
%                           indices corresponding to the simulation time
%                           instants over the course of the algorithm
%                           execution.
%       .xmat               ('simlength' x n matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%       .umat               ('simlength' x m matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose m-columns are the control signal u(t)
%                           at the respective time instant.
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% 
% GLOBAL VARIABLES
% 
% *************************************************************************

global sys;


% Reference signal r(t) settings
global r_sett;

% Do nominal min-phase model (=1) or nonmin-phase model (=0)
global u_sett;

        

% *************************************************************************
% 
% UNPACK ALGORITHM SETTINGS/PARAMETERS
% 
% *************************************************************************

% System
sys = master_settings.sys;             % System array
n = sys.n;                          % System order
m = sys.m;                          % System input dimension

% System cell array
model_cell = sys.model_cell;

% Indices of nominal, perturbed models
indnom = sys.indnom;
model = get_elt_multidim(model_cell, indnom); 

% Number of states in FBL plant
n_fbl = sys.n_fbl;

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;


% Reference signal r(t) settings
r_sett = alg_settings.r_sett;

% Simulation length
tsim = alg_settings.tsim;

% ***********************
%       
% TRIM CONDITIONS
%
% x_v = [V, \gamma, h, \alpha, q, \delta_{T}, \dot{\delta}_{T}] in R^{7}
%    

% % Wang, Stengel values
% xve = [15060 ; 0 ; 110e3 ; 0.0315 ; 0 ; 0.183 ; 0];
% ue = [0.183 ; -0.0066];

% % Numerically solved-for values
% xe = model.trimconds.xe;
% ue = model.trimconds.ue;

% Numerically solved-for values -- WITH ELEVATOR-LIFT EFFECTS
xe = model.trimconds.xe;
ue = model.trimconds.ue;


% ***********************
%       
% PRESET SETTINGS
%   

% Simulation model
model_sim_ind = alg_settings.model_sim_ind;
model_sim = get_elt_multidim(model_cell, model_sim_ind);
xe_sim = model_sim.trimconds.xe;
ue_sim = model_sim.trimconds.ue;

% Do prefilter (=1) or not (=0)
pf1nopf0 = alg_settings.pf1nopf0;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    pfavec = alg_settings.pfavec;   
end


% ***********************
%       
% CONTROLLER
%

% Final controller params
controller = master_settings.controller_fbl;
u_sett.controller = controller;

% DEBUGGING: Print done loading
disp('***** LOADING PARAMETERS COMPLETE *****')

% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************

% ***********************
%       
% MISCELLANEOUS VARIABLES
%

% Initial condition
x0 = alg_settings.x0;  

% Initial actual thrust \delta_{T}
dT0 = ue(1);

% Derivative of initial actual thrust \dot{\delta}_{T}
ddT0 = 0;

% Append actuator, integrator ICs
x0_sim = [  x0
            dT0
            ddT0
            zeros(m,1)  ];

% If prefilter is used, append ICs for the prefilter states
if pf1nopf0
    x0_sim = [  x0_sim
                zeros(m,1)  ];
end

% Indices of how state vector is partitioned
indsx = (1:n_fbl)';
indsz = (n_fbl+1:n_fbl+m)';
indspf = (n_fbl+m+1:n_fbl+m+m)';

inds.indsx = indsx;
inds.indsz = indsz;
if pf1nopf0
    inds.indspf = indspf;
end
u_sett.inds = inds;

% ***********************
%       
% CONTROL SETTINGS
%   

% Simulation model
u_sett.model_sim = model_sim;
u_sett.xe_sim = xe_sim;
u_sett.ue_sim = ue_sim;

% Do prefilter (=1) or not (=0)
u_sett.pf1nopf0 = pf1nopf0;

% Trim
u_sett.xe = xe;
u_sett.ue = ue;

% Indices of state variables to be tracked
inds_xr = alg_settings.inds_xr;
u_sett.inds_xr = inds_xr;


% Coordinate transformations
u_sett.su = model.lin.io.sud;
u_sett.sx = model.lin.io.sxd;
u_sett.sy = model.lin.io.syd;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    u_sett.pfavec = pfavec;   
end


% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% Time vector, state trajectory, control signal
tvec = [];
xmat = [];
umat = [];


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
% *************************************************************************
%
% RUN SIMULATION WITH FINAL CONTROLLER
% 
% *************************************************************************
% *************************************************************************


% ***********************
%       
% RUN SIMULATION
%

% Time span for simulation
tspan = [0, tsim];

% Run simulation
[t, x] = ode45(@odefunct, tspan, x0_sim);

% % DEBUGGING: Plot states while calling ode45
% options = odeset('OutputFcn',@odeplot);
% figure(100);
% [t, x] = ode45(@odefunct, tspan, x0_sim, options); 


% ***********************
%       
% STORE DATA
%

% Store time data
tvec = [    tvec
            t       ];


% Store system state data
xmat = [    xmat
            x       ];


% DEBUGGING: Print done simulating
disp('***** SIMULATION COMPLETE *****')



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PREPARE OUTPUT DATA
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% CONTROL SIGNAL
% 
% *************************************************************************

% Initialize empty matrix
umat = zeros(size(tvec,1), m);

% Calculate control
for k = 1:size(tvec,1)
    
    % Get time
    t = tvec(k);

    % Get state vector 
    xs = xmat(k,:)';
    
    % Evaluate control 
    u = uxt_alg(xs, t);
                
    % Store control
    umat(k,:) = u';

end

% Store control signal
out_data.umat = umat;

% *************************************************************************
%
% TIME, STATE
% 
% *************************************************************************

% Time vector
out_data.tvec = tvec;

% Rescale integrator states to reflect original units
xmat(:,indsz) = (inv(u_sett.sy) * xmat(:,indsz)')';

% If a prefilter was used, rescale and shifted filtered reference command
% responses to reflect trim (pre-transformation)
if pf1nopf0
   xmat(:,indspf) = ...
       (inv(u_sett.sy) * xmat(:,indspf)')' ...
       +  [xe(inds_xr(1)); xe(inds_xr(2))]';
end


% Store modified state vector
out_data.xmat = xmat;

% Store indices
out_data.inds = inds;


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
% *************************************************************************
%
% CALCULATE DYNAMICS 
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function xdot = odefunct(t, x)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Number of states in FBL plant
n_fbl = sys.n_fbl;

% Control settings
global r_sett;
global u_sett;

% Extract simulation model
model_sim = u_sett.model_sim;

% Get coordinate transformations
% sx = u_sett.sx;
% su = u_sett.su;
sy = u_sett.sy;

% Get equilibrium point x_e (pre-transformation)
xe = u_sett.xe;

% % Get equilibrium point x_e (pre-transformation) -- simulated system
% xe = u_sett.xe_sim;

% Extract vehicle states x_v
xv = x(1:n_fbl);

% Evaluate drift dynamics
f_x = model_sim.fil(xv);

% Evaluate input gain matrix
g_x = model_sim.gil(xv);

% Calculate control signal
u = uxt_alg(x, t);
% u = [0.183; -0.0066];

% Evaluate reference trajectory r(t) (pre-transformation)
rt = eval_xr(t, r_sett);

% Evaluate reference trajectory r(t) (post-transformation)
% yr = [rt(1); rt(5)];
yr = [rt(1,1); rt(2,1)];
yrp = sy * yr;

% % Get y_{r}(t) (post-transformation)
% yrp = [rtp(1); rtp(5)];

% Get y(t) (post-transformation)
yp = sy * x(u_sett.inds_xr);

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(u_sett.inds_xr);

% Get (small-signal) reference command r (post-transformation)
trp = yrp - yep;

% Get (small-signal) output y (post-transformation)
typ = yp - yep;

% If prefilters used, get filtered reference command (post-transformation)
if u_sett.pf1nopf0
    trfp = x(u_sett.inds.indspf);
end

% Evaluate integrator state derivative \dot{z} (post-transformation)
if u_sett.pf1nopf0
    zdot = -(trfp - typ);
else
    zdot = -(trp - typ);
end

% Append integral augmentation state derivatives if executing LQ servo
xdot = [    f_x + g_x * u
            zdot            ];

% If prefilter inserted, evaluate prefilter dynamics (post-transformation)
if u_sett.pf1nopf0
    pfdot = -diag(u_sett.pfavec) * trfp + diag(u_sett.pfavec) * trp;
    xdot = [    xdot
                pfdot   ];
end        






%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE CONTROL SIGNAL u(x, t)
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function u = uxt_alg(x, t)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Control settings
global r_sett;
global u_sett;

% Number of states in FBL plant
n_fbl = sys.n_fbl;

% FBL Controller
controller = u_sett.controller;
K1 = controller.K1;
K2 = controller.K2;

% Get coordinate transformations
% sx = u_sett.sx;
% su = u_sett.su;
sy = u_sett.sy;

% Extract vehicle states x_v
xv = x(1:n_fbl);

% Evaluate reference trajectory r(t) (pre-transformation)
rt = eval_xr(t, r_sett);

% Evaluate reference trajectory r(t) (pre-transformation)
% yr = [rt(1); rt(5)];
yr = [rt(1,1); rt(2,1)];

% Get equilibrium point x_e (pre-transformation)
xe = u_sett.xe;

% Get y(t) (pre-transformation)
y = x(u_sett.inds_xr);

% Get equilibrium value of the output y_{e} (pre-transformation)
ye = xe(u_sett.inds_xr);

% Get (small-signal) reference command r (pre-transformation)
tr = yr - ye;

% Get (small-signal) output y (pre-transformation)
ty = y - ye;

% If prefilters used, get filtered reference command (pre-transformation)
if u_sett.pf1nopf0
    trf = sy \ x(u_sett.inds.indspf);
end

% Evaluate error signal y - r (pre-transformation)
if u_sett.pf1nopf0
    e = -(trf - ty);
else
    e = -(tr - ty);
end

% Evaluate 

% Extract integral errors (post-transformation)
zp = x(u_sett.inds.indsz);

% Calculate integral errors (pre-transformation)
z = sy \ zp;

% Extract \xi from the state (pre-transformation)
% \xi = [   \int(V - V_r)dt 
%           V - V_r 
%           \dot{V} - \dot{V}_r 
%           \ddot{V} - \ddot{V}_r ] \in R^{4}
% Cf. Wang, Eqn. (52)
% xi = [  z(1)
%         [   xv(u_sett.inds_xr(1))
%             controller.diffVil(xv)     ] - rt(1:3) ];
xi = [  z(1)
        [   e(1)
            controller.diffVil(xv)     ] ];
    
% Extract \eta from the state (pre-transformation)
% \eta = [  \int(\gamma-\gamma_r)dt
%           \gamma-\gamma_r 
%           \dot{\gamma}-\dot{\gamma}_r 
%           \ddot{\gamma}-\ddot{\gamma}_r ] \in R^{4}
% Cf. Wang, Eqn. (52)
% eta = [ z(2)
%         [   xv(u_sett.inds_xr(2))
%             controller.diffgil(xv)     ] - rt(5:7) ];
eta = [ z(2)
        [   e(2)
            controller.diffgil(xv)     ]  ];

% Evaluate LQR control v_1 = - K_1 * \xi
% Cf. Wang, Eqn. (59)
v1 = - K1 * xi;

% Evaluate LQR control v_2 = - K_2 * \eta
% Cf. Wang, Eqn. (61)
v2 = - K2 * eta;

% Control vector v = [v_1, v_2]^T
v = [v1; v2];

% Evaluate f^*(x)
fstarx = controller.fstaril(xv);

% Evaluate G^*(x)
Gstarx = controller.Gstaril(xv);

% Get V_r^{(3)}, \gamma_r^{(3)}
% drrhot = rt([4 8]);
drrhot = [rt(1,4); rt(2,4)];

% Calculate the final NDI control u(x)
% u(x) = {G^*}^{-1}(x) [-f^*(x) + v]
% Cf. Wang, Eqn. (16)
u = Gstarx \ (-fstarx + v + drrhot);









