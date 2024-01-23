function out_data = alg_dirl_nonlin(alg_settings,...
    group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% DECENTRALIZED EXCITABLE INTEGRAL REINFORCEMENT LEARNING (dEIRL) ALGORITHM
%
% Brent Wallace  
%
% 2022-11-14
%
% This program implements the dEIRL algorithm for nonlinear systems.
%
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings  struct with the following fields:
%   
%   sys                     (Struct) contains system tag/model info. 
%   Ts                      (Double) Base sample period to collect data (s)
%   loop_cell               (N-entry Struct) This cell array contains the
%                           learning hyperparameters in each of the 1 <= j
%                           <= N loops of the system 'sys'. Each entry 1 <=
%                           j <= N has the following fields (see
%                           config_preset.m):
%       indsx               (n_j-dim Vector -- NOT HYPERPARAM) Contains the 
%                           indices of the total state vector x
%                           corresponding to the states in this loop x_j
%                           \in R^{n_j}.
%       indsy               (m_j-dim Vector -- NOT HYPERPARAM) Contains the
%                           indices of the total state vector x
%                           corresponding to the outputs y_j \in R^{m_j} in
%                           this loop.
%       Q                   (n_j x n_j Matrix) State penalty 
%                           Q_j = Q_j^T >= 0 in this loop j.
%       R                   (m_j x m_j Matrix) Control penalty
%                           R_j = R_j^T > 0 in this loop j.
%       K0                  (m_j x n_j Matrix) Initial stabilizing
%                           controller K_{0,j} in this loop j. NOTE: This
%                           is configurable, but it should be set by the
%                           LQR design performed in the respective system's
%                           controller initialization function (see
%                           /config/config_controllers_<system>.m).
%       nTs                 (Integer) Multiple of the base sample period Ts
%                           to sample learning data at in this loop j.
%       l                   (Integer) Number of data samples to collect for
%                           this loop j.
%       istar               (Integer) Number of learning iterations to
%                           execute for this loop j.
%   noise                   (Struct) Contains parameters for the
%                           plant-input exploration noise d(t) \in R^m.
%                           This basically allows a sum of sinusoids to be
%                           injected at each input, and the fields
%                           correspond to the desired amplitude, frequency,
%                           and phase of the sinusoids. For further
%                           description, see eval_noise.m. In what follows,
%                           suppose M sinusoidal terms are injected in each
%                           input channel (this M is configurable). Then
%                           'noise' has the following fields:
%       cos1_sin0           (m x M Matrix) In input channel 1 <= i <= m,
%                           make the sinusoid 1 <= j <= M a cosine (=1) or
%                           sine (=0).
%       Amat                (m x M Matrix) The (i,j)th entry of this matrix
%                           determines the amplitude of the sinusoid 
%                           1 <= j <= M injected in input channel 
%                           1 <= i <= m.
%       Tmat                (m x M Matrix) The (i,j)th entry of this matrix
%                           determines the period of the sinusoid 
%                           1 <= j <= M injected in input channel 
%                           1 <= i <= m.
%   r_sett_train            (Struct) Contains parameters for the
%                           reference excitation r(t) \in R^m. This has
%                           fields identical to 'noise', so see above.
%       
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) algorithm output data. Has the
%                           following fields:
%   dirl_data               (Struct) Contains algorithm learning output
%                           data. Has the following fields:
%       P_cell              (N-dim Cell) The j-th entry contains a matrix
%                           with all the ALE solutions P_{i,j} in loop j.
%       K_cell              (N-dim Cell) The j-th entry contains a matrix
%                           with all the controllers K_{i,j} in loop j.
%       lq_data             (Struct) Contains the LQR data for the FINAL
%                           controllers in each loop. This struct is used
%                           by other programs to pull final controller data
%                           for plotting, simulation, etc.
%       cmat_cell           (N-dim cell) The j-th entry contains the 
%                           VECTORIZED ALE solutions v(P_{i,j}) in loop j
%                           (used for plotting weight responses).
%       cond_A_vec_cell     (N-dim cell) The j-th entry contains an
%                           i_{j}^{*}-dim vector, the i-th entry of which
%                           is the conditioning of the least-squares matrix
%                           A_{i,j} in loop j at iteration i.
%   tlvec                   ('simlength'-dimensional vector) vector of time
%                           indices corresponding to the simulation time
%                           instants over the course of the learning
%                           algorithm simulation (if executed).
%   xlmat                   ('simlength' x n matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%   ulmat                   ('simlength' x m matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose m-columns are the control signal u(t)
%                           at the respective time instant.
%   tvec, xmat, umat        (Matrices) Analogous to tlvec, xlmat, ulmat,
%                           but for the POST-LEARNING simulation conducted
%                           on the final controller (if executed).
%   inds                    (Struct) Contains the state vector partition
%                           for the numerical simulation conducted by
%                           ode45. E.g., to find where the plant states are
%                           in the ode45 state vector, query inds.indsx.
%   loopdata                (N-dim Struct) Contains system data for each of
%                           the N loops (e.g., state dimension n_j, etc.)
%                           This is mainly used for back-end purposes.
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

% Clear global variables
clear global

global sys;


% Reference signal r(t) settings
global r_sett;

% Control settings
global u_sett;



% *************************************************************************
% 
% UNPACK ALGORITHM SETTINGS/PARAMETERS
% 
% *************************************************************************

% System
sys = master_settings.sys;             % System array
n = sys.n;                          % System order
nlq = sys.nlq;                          % System order for LQ servo
m = sys.m;                          % System input dimension

% System cell array
model_cell = sys.model_cell;

% Indices of nominal, perturbed models
indnom = sys.indnom;
indmodel = alg_settings.indmodel;

% Nominal, perturbed models
% System model
model = get_elt_multidim(model_cell, indnom); 
% System model -- perturbed
model_nu = get_elt_multidim(model_cell, indmodel); 

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% Do learning (=1) or not (=0)
dolearning = group_settings.dolearning;

% Do post-learning sim (=1) or not (=0)
dopostlearning = group_settings.dopostlearning; 

% Save learning data (=1) or not (=0)
if dolearning
    savetrainingdata = group_settings.savetrainingdata; 
end

% Relative path to load learning data from if learning isn't active
relpath_data = group_settings.relpath_data_dirl;
filename_data = group_settings.filename_data_dirl;

% Reference signal r(t) settings -- training phase
r_sett_train = alg_settings.r_sett_train;

% Reference signal r(t) settings
r_sett = alg_settings.r_sett;
r_sett.r_sett_train = r_sett_train;

% Exploration noise
noise = alg_settings.noise;   

% Base sample period (sec)
Ts = alg_settings.Ts;

% Sample period begin time (multiples of base sample period)
nTs_begin = alg_settings.nTs_begin;

% Post-learning simulation length
tsim = group_settings.tsim;

% Do x_3 loop (=1) or not (=0)
dox3 = alg_settings.dox3;
lenx3 = alg_settings.lenx3;

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;

% Overwrite current controller (=1) or not (=0)
updatecontroller = alg_settings.updatecontroller;

% Use the nominal linearization A for w = f(x) - A x at nominal trim (=1)
% or simulation trim (=0)
wnom1sim0 = alg_settings.wnom1sim0;

% Simulate w = f(x) - Ax for simulation model (=1) or not (=0)
sim_w = alg_settings.sim_w;

% Nominal model
model_nom_tag = alg_settings.model_nom_tag;
model_nom_ind = alg_settings.model_nom_ind;
model_nom = get_elt_multidim(model_cell, model_nom_ind);
xe_nom = model_nom.trimconds.xe;
ue_nom = model_nom.trimconds.ue;

% Simulation model
model_sim_tag = alg_settings.model_sim_tag;
model_sim_ind = alg_settings.model_sim_ind;
model_sim = get_elt_multidim(model_cell, model_sim_ind);
xe_sim = model_sim.trimconds.xe;
ue_sim = model_sim.trimconds.ue;

% LQ data
lq_data_cell = master_settings.lq_data_cell;

% Optimal LQ data -- simulation model
lq_data_opt_sim = get_elt_multidim(lq_data_cell, model_sim_ind);



% ***********************
%       
% LOOP DATA
%  

% Holds settings for each loop
loop_cell = alg_settings.loop_cell;

% ***********************
%       
% PRESET SETTINGS
%   

% Model linear (=1) or nonlinear (=0)
lin1nonlin0 = alg_settings.lin1nonlin0;

% Do prefilter (=1) or not (=0)
pf1nopf0 = alg_settings.pf1nopf0;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    pfavec = alg_settings.pfavec;   
end

% DEBUGGING: Print done loading
disp('***** LOADING PARAMETERS COMPLETE *****')

% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************



% ***********************
%       
% CONTROL SETTINGS
%   

% Exploration noise
u_sett.noise = noise;

% Learning flag
u_sett.islearning = 1;

% Model linear (=1) or nonlinear (=0)
u_sett.lin1nonlin0 = lin1nonlin0;

% % Plant model MP (=1) or NMP (=0)
% u_sett.Pmp1nmp0 = Pmp1nmp0;

% Do prefilter (=1) or not (=0)
u_sett.pf1nopf0 = pf1nopf0;

% Has integral augmentation (=1) or not (=0)
u_sett.hasintaug = hasintaug;

% Coordinate transformations
u_sett.su = model.lin.io.sud;
u_sett.sx = model.lin.io.sxd;
sy = model.lin.io.syd;
u_sett.sy =sy;

% Transform [z, y, x_r] -> [x_1, x_2]
Sxirl = model.lin.io.Sxirl;
u_sett.Sxirl = Sxirl;
% Transform [x_1, x_2] -> [z, y, x_r]
invSxirl = model.lin.io.invSxirl;
u_sett.invSxirl = invSxirl;
% Transform [z, y, x_r, x_3] -> [x_1, x_2, x_3]
Sxirl_x3 = model.lin.io.Sxirl_x3;
u_sett.Sxirl_x3 = Sxirl_x3;
% Transform [x_1, x_2, x_3] -> [z, y, x_r, x_3]
invSxirl_x3 = model.lin.io.invSxirl_x3;
u_sett.invSxirl_x3 = invSxirl_x3;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    u_sett.pfavec = pfavec;   
end

% Do x_3 loop (=1) or not (=0)
u_sett.dox3 = dox3;
u_sett.lenx3 = lenx3;

% Use the nominal linearization A for w = f(x) - A x at nominal trim (=1)
% or simulation trim (=0)
u_sett.wnom1sim0 = wnom1sim0;

% Simulate w = f(x) - Ax for simulation model (=1) or not (=0)
u_sett.sim_w = sim_w;


% ***********************
%       
% LOOP DATA
%  

% Get number of loops
numloops = size(loop_cell,1);
loopdata.numloops = numloops;

% Store state, output dimensions in each loop
nxztot = 0;
nxzbtot = 0;
for k = 1:numloops
    loop_cell{k}.nx = size(loop_cell{k}.indsx, 1);
    loop_cell{k}.ny = size(loop_cell{k}.indsy, 1);
    if hasintaug
        loop_cell{k}.nxz = loop_cell{k}.nx + loop_cell{k}.ny;
    else
        loop_cell{k}.nxz = loop_cell{k}.nx;
    end
    loop_cell{k}.nxzb = loop_cell{k}.nxz * (loop_cell{k}.nxz + 1) / 2;
    nxztot = nxztot + loop_cell{k}.nxz;
    nxzbtot = nxzbtot + loop_cell{k}.nxzb;
end
if dox3
    nxztot = nxztot + 1;
end
nxztotb = nxztot * (nxztot + 1) / 2;
loopdata.nxztotb = nxztotb;
loopdata.nxztot = nxztot;

% Whether or not to perform learning in each loop
doloopvec = alg_settings.doloopvec;
doloopvec = logical(doloopvec);
loopdata.doloopvec = doloopvec;

% Number of iterations to conduct for each loop
istarvec = zeros(numloops,1);
for k = 1:numloops
    istarvec(k) = loop_cell{k}.istar;
end
loopdata.istarvec = istarvec;

% Number of samples to collect per iteration for each loop
lvec = zeros(numloops,1);
for k = 1:numloops
    lvec(k) = loop_cell{k}.l;
end
loopdata.lvec = lvec;

% Integer multiples of base sample period to collect data at for each loop
nTsvec = zeros(numloops,1);
for k = 1:numloops
    nTsvec(k) = loop_cell{k}.nTs;
end
loopdata.nTsvec = nTsvec;

% Determine total number of samples to collect for each loop
nsampvec = zeros(numloops,1);
for k = 1:numloops
    nsampvec(k) = lvec(k) * nTsvec(k);
end
loopdata.nsampvec = nsampvec;

% Max total number of samples to collect during learning
% NOTE: Only consider active loops
maxnsamp = max(nsampvec);
loopdata.maxnsamp = maxnsamp;

% Max i^{*} among the active loops
maxistar = max(istarvec .* doloopvec);
loopdata.maxistar = maxistar;


% ***********************
%       
% STATE NORMALIZATION (IF APPLICABLE)
%  

% Stores whether at least one of the loops has prescaling
hasS = 0;
% Stores whether each loop has prescaling (=1) or not (=0)
hasSvec = zeros(numloops, 1);
% Stores prescaling matrix in each loop (if applicable)
S_cell = cell(numloops,1);

for k = 1:numloops
    hasSvec(k) = isfield(loop_cell{k}, 'S');
    if hasSvec(k)
        S_cell{k} = loop_cell{k}.S;
        hasS = 1;
    end
end


% ***********************
%       
% STATE VECTOR PARTITION INDICES
%   

% Indices of how state vector is partitioned
indsxr = model.inds_xr;

% Vehicle states
cnt = 1;
len = n;
indsx = (cnt:cnt+len-1)';
cnt = cnt + len;

% Integral augmentation states
if hasintaug
    len = m;
    indsz = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Pre-filter states
if pf1nopf0
    len = m;
    indspf = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Kronecker product integrals -- (x, x)
len = nxztotb;
indsIbx = (cnt:cnt+len-1)';
cnt = cnt + len;

% Kronecker product integrals -- (x_k, w_k)
if sim_w
    indsIBxw = cell(numloops+1,1);
    len = nxzbtot;
    indsIBxw{numloops+1} = (cnt:cnt+len-1)';
    for k = 1:numloops
        len = loop_cell{k}.nxzb;
        indsIBxw{k} = (cnt:cnt+len-1)';
        cnt = cnt + len;
    end
end

% Kronecker product integrals -- (x_k, \tilde{w}_k)
indsIBxtw = cell(numloops+1,1);
len = nxzbtot;
indsIBxtw{numloops+1} = (cnt:cnt+len-1)';
for k = 1:numloops
    len = loop_cell{k}.nxzb;
    indsIBxtw{k} = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Kronecker product integrals -- (x_k, g_k(x)u)
indsIBxgu = cell(numloops+1,1);
len = nxzbtot;
indsIBxgu{numloops+1} = (cnt:cnt+len-1)';
for k = 1:numloops
    len = loop_cell{k}.nxzb;
    indsIBxgu{k} = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Output integrals \int y
if hasintaug
    len = m;
    indsIy = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% TOTAL SIMULATION STATE VECTOR LENGTH
n_sim = cnt - 1;
u_sett.n_sim = n_sim;

% Indices of loop-wise Kronecker product integrals
indsIBxwloc = cell(numloops,1);
cntloc = 1;
for k = 1:numloops
    len = loop_cell{k}.nxzb;
    indsIBxwloc{k} = (cntloc:cntloc+len-1)';
    cntloc = cntloc + len;
end

% Indices of loop-wise state vectors in the IRL state partition
cntirl = 1;
if dox3
    indsxirl = cell(numloops+1,1);
else
    indsxirl = cell(numloops,1);
end
for k = 1:numloops
    len = loop_cell{k}.nxz;
    indsxirl{k} = (cntirl:cntirl+len-1)';
    cntirl = cntirl + len;
end
if dox3
    len = lenx3;
    indsxirl{numloops+1} = (cntirl:cntirl+len-1)';
    cntirl = cntirl + len;
end

% % Indices of loop-wise state vectors in the IRL state partition -- master
% cntirl = 1;
% indsxirl_m = cell(numloops_m,1);
% for k = 1:numloops_m
%     len = loop_cell_m{k}.nxz;
%     indsxirl_m{k} = (cntirl:cntirl+len-1)';
%     cntirl = cntirl + len;
% end

% Store
inds.indsxr = indsxr;
inds.indsx = indsx;
if hasintaug
    inds.indsz = indsz;
end
if pf1nopf0
    inds.indspf = indspf;
end
inds.indsIbx = indsIbx;
if sim_w
    inds.indsIBxw = indsIBxw;
end
inds.indsIBxtw = indsIBxtw;
inds.indsIBxgu = indsIBxgu;
if hasintaug
    inds.indsIy = indsIy;
end
inds.indsxirl = indsxirl;
% inds.indsxirl_m = indsxirl_m;
u_sett.inds = inds;


% Store loop_cell struct in global settings
u_sett.loop_cell = loop_cell;
u_sett.numloops = numloops;


% ***********************
%       
% GET STATE-SPACE MATRICES FOR EACH LOOP
%   

% Nominal system
if wnom1sim0
    [A_cell_nom, B_cell_nom] = make_AB_cells(model_nom.lin);
else
    switch model_sim_tag
        case 'default'
            switch model_nom_tag
                case 'default'
                    [A_cell_nom, B_cell_nom] = make_AB_cells(model.lin);
                case 'perturbed'
                    [A_cell_nom, B_cell_nom] = ...
                        make_AB_cells(model_nu.lin_atnom);
            end
        case 'perturbed'
            switch model_nom_tag
                case 'default'
                       lintmp = get_elt_multidim(model.lin_atnu, indmodel);
                    [A_cell_nom, B_cell_nom] =...
                        make_AB_cells(lintmp);
                case 'perturbed'
                    [A_cell_nom, B_cell_nom] = make_AB_cells(model_nu.lin);
            end
    end
end

% Simulation system
[A_cell_sim, B_cell_sim] = make_AB_cells(model_sim.lin);

% Store
model_nom.A_cell = A_cell_nom;
model_nom.B_cell = B_cell_nom;
model_sim.A_cell = A_cell_sim;
model_sim.B_cell = B_cell_sim;

% Store nominal model in global settings
u_sett.model_nom = model_nom;
u_sett.xe_nom = xe_nom;
u_sett.ue_nom = ue_nom;

% Store simulation model in global settings
u_sett.model_sim = model_sim;
u_sett.xe_sim = xe_sim;
u_sett.ue_sim = ue_sim;


% ***********************
%       
% ICS
%

% Initial condition
x0 = alg_settings.x0; 
x0_sim = x0;

% Append integrator ICs
if hasintaug
    x0_sim = [  x0_sim
                zeros(m,1)  ];
end

% If prefilter is used, append ICs for the prefilter states
if pf1nopf0
    x0_sim = [  x0_sim
                zeros(m,1)  ];
end


% % Save this IC vector for post-learning simulation
% x0_sim_f = x0_sim;

% Kronecker product integrals -- (x, x)
x0_sim = [  x0_sim
            zeros(nxztotb,1)  ];

% Kronecker product integrals -- (x_k, w_k)
if sim_w
    x0_sim = [  x0_sim
                zeros(nxzbtot,1)  ];
end

% Kronecker product integrals -- (x_k, \tilde{w}_k)
x0_sim = [  x0_sim
            zeros(nxzbtot,1)  ];

% Kronecker product integrals -- (x_k, g_k(x)u)
x0_sim = [  x0_sim
            zeros(nxzbtot,1)  ];

% Output integrals \int y
if hasintaug
    x0_sim = [  x0_sim
                zeros(m,1)  ];
end

% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% Time vector, state trajectory, control signal
tlvec = [];
xlmat = [];
ulmat = [];
enmat = [];

% Weight storage
cmat_cell = cell(numloops,1);
for k = 1:numloops
    nxzbk = loop_cell{k}.nxzb;
    cmat_cell{k} = zeros(istarvec(k), nxzbk);
end

% Stores controllers, "P" matrices in each loop
P_cell = cell(numloops,1);
K_cell = cell(numloops,1);
for k = 1:numloops
    % Get number of states, outputs associated with this loop
    nxk = loop_cell{k}.nx;
    nyk = loop_cell{k}.ny;
    nxzk = loop_cell{k}.nxz;
    P_cell{k} = zeros(nxzk, nxzk, istarvec(k));
    K_cell{k} = zeros(nyk, nxzk, istarvec(k)+1);
    K_cell{k}(:,:,1) = loop_cell{k}.K0;
end
u_sett.K_cell = K_cell;

if hasS
    PS_cell = P_cell;
    KS_cell = K_cell;    
end


% Stores previous sample
x0_cell = cell(numloops,1);

% Stores state trajectory samples at (base) sample instants
xTsmat = zeros(maxnsamp+1,nxztot);

% Stores Kronecker product integrals -- (x, x)
Ibxmat = zeros(maxnsamp,nxztotb);

% Stores Kronecker product integrals -- (x_k, w_k)
if sim_w
    IBxwmat = zeros(maxnsamp,nxzbtot);
end

% Stores Kronecker product integrals -- (x_k, \tilde{w}_k)
IBxtwmat = zeros(maxnsamp,nxzbtot);

% Stores Kronecker product integrals -- (x_k, g_k(x)u)
IBxgumat = zeros(maxnsamp,nxzbtot);


% ***********************
%       
% CONDITIONING DATA
%

% Stores condition number of LS "A" matrix at each iteration in each loop
cond_A_vec_cell = cell(numloops,1);
for k = 1:numloops
    cond_A_vec_cell{k} = zeros(istarvec(k),1);
end

% Stores data if state prescaling is performed
if hasS
    cond_AS_vec_cell = cond_A_vec_cell;
end


% ***********************
%       
% MATRIX RELATING B(x,y) TO kron(x,y)
%   
%   B(x,y) = M kron(x,y)
%
W_cell = cell(numloops,1);
Wr_cell = cell(numloops,1);
for k = 1:numloops
    % Get number of states, outputs associated with this loop
%     nxk = loop_cell{k}.nx;
%     nyk = loop_cell{k}.ny;
    nxzk = loop_cell{k}.nxz;
%     nxzbk = loop_cell{k}.nxzb;
    [Wk, Wrk] = make_W(nxzk);
    W_cell{k} = Wk;
    Wr_cell{k} = Wrk;
end

% Make for the overall DIRL system
[Wtot, Wrtot] = make_W(nxztot);


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

% If learning is desired, execute it
if dolearning

% *************************************************************************
% *************************************************************************
%
% DATA COLLECTION
% 
% *************************************************************************
% *************************************************************************

% Keeps track of iteration count in each loop
ivec = ones(numloops,1);
u_sett.ivec = ivec;

% ***********************
%       
% RUN INITIAL SIMULATION
%

if nTs_begin > 0

% Time span for initial simulation
tspan = [0, nTs_begin] * Ts;

% Run simulation
[t, x] = ode45(@odefunct, tspan, x0_sim);

% Get state at end of simulation
x1 = x(end,:)';
t0 = t(1);
t1 = t(end);

% Store time, state, control data
if savetrainingdata
    tlvec = [    tlvec
                t       ];    
    xlmat = [    xlmat
                x       ];
    [utmp, entmp] = uxt_all(x,t);
    ulmat = [    ulmat
                utmp    ];    
    enmat = [    enmat
                entmp    ];    
end

% Reset \int y states for initialization
x1(indsIy) = 0;

% RESET KRONECKER PRODUCT INTEGRALS

% Reset Kronecker product integrals -- (x, x)
x1(indsIbx) = 0;

% Reset Kronecker product integrals -- (x_k, w_k)
if sim_w
    x1(indsIBxw{numloops+1}) = 0;
end

% Reset Kronecker product integrals -- (x_k, \tilde{w}_k)
x1(indsIBxtw{numloops+1}) = 0;

% Reset Kronecker product integrals -- (x_k, g_k(x)u)
x1(indsIBxgu{numloops+1}) = 0;

% Set up ICs for first simulation
x0_sim = x1;

end


% Stores previous sample
for k = 1:numloops
    x0_cell{k} = get_loop_state(x0_sim,t1,k,xe_sim);
end

% Stores state trajectory samples at (base) sample instants
xTsmat(1,:) = get_loop_state(x0_sim, t1, 0, xe_sim);

% *************************************************************************
%       
% MAIN LOOP
%
% *************************************************************************

for sampcnt = 1:maxnsamp

%     % Display sample count
%     disp(['RUNNING SAMPLE   ' num2str(sampcnt) ...
%         '     OUT OF  ' num2str(maxnsamp)])

    % ***********************
    %       
    % RUN SIMULATION
    %
    
    % Time span for simulation
    tspan = [sampcnt-1+nTs_begin, sampcnt+nTs_begin] * Ts;
    
    % Run simulation
    [t, x] = ode45(@odefunct, tspan, x0_sim);
    
%     % DEBUGGING: Plot states while calling ode45
%     options = odeset('OutputFcn',@odeplot);
%     figure(100);
%     [t, x] = ode45(@odefunct, tspan, x0_sim, options); 

    % ***********************
    %       
    % DATA STORAGE
    %

    % Store time, state, control data
    if savetrainingdata
        tlvec = [    tlvec
                    t       ];    
        xlmat = [    xlmat
                    x       ];
        [utmp, entmp] = uxt_all(x,t);
        ulmat = [    ulmat
                    utmp    ];    
        enmat = [    enmat
                    entmp    ];    
    end

    % Set up ICs for next simulation
    x1 = x(end,:)';
    t1 = t(end);

    % ***********************
    %       
    % STORE REGRESSION DATA
    %

    % Store current state trajectory data
    xTsmat(sampcnt+1,:) = get_loop_state(x1, t1, 0, xe_sim);

    % Store Kronecker product integrals -- (x, x)
    Ibxmat(sampcnt,:) = x1(indsIbx);
    
    % Store Kronecker product integrals -- (x_k, w_k)
    if sim_w
        IBxwmat(sampcnt,:) = x1(indsIBxw{numloops+1});
    end

    % Store Kronecker product integrals -- (x_k, \tilde{w}_k)
    IBxtwmat(sampcnt,:) = x1(indsIBxtw{numloops+1});

    % Store Kronecker product integrals -- (x_k, g_k(x) u)
    IBxgumat(sampcnt,:) = x1(indsIBxgu{numloops+1});

    % ***********************
    %       
    % RESET KRONECKER PRODUCT INTEGRALS
    %

    % Reset Kronecker product integrals -- (x, x)
    x1(indsIbx) = 0;

    % Reset Kronecker product integrals -- (x_k, w_k)
    if sim_w
        x1(indsIBxw{numloops+1}) = 0;
    end

    % Reset Kronecker product integrals -- (x_k, \tilde{w}_k)
    x1(indsIBxtw{numloops+1}) = 0;
    
    % Reset Kronecker product integrals -- (x_k, g_k(x)u)
    x1(indsIBxgu{numloops+1}) = 0;

    % Set up ICs for next simulation
    x0_sim = x1;

end         % END MAIN LOOP


% *************************************************************************
% *************************************************************************
%
% LEARNING
% 
% *************************************************************************
% *************************************************************************


% Learn in each loop
for k = 1:numloops

% If loop is active, perform learning
if doloopvec(k)

    % ***********************
    %       
    % GET REGRESSION DATA
    %    

    % Get number of states, outputs associated with this loop
    nxk = loop_cell{k}.nx;
    nyk = loop_cell{k}.ny;
    nxzk = loop_cell{k}.nxz;
    nxzbk = loop_cell{k}.nxzb;

    % Number of base sample period to collect, number of samples
    nTsk = nTsvec(k);
    lk = lvec(k);

    % Identity matrix of dimension n_k
    Ink = eye(nxzk);

    % Indices of states/outputs corresponding to this loop
    indsxirlk = indsxirl{k};
    indsyk = loop_cell{k}.indsy;

    % B_{k,k}
    Bkk = B_cell_sim{k,k};

    % Q_k, R_k
    Qk = loop_cell{k}.Q;
    Rk = loop_cell{k}.R;

    % Matrix relating B(x,y) to kron(x,y) in this loop
    Wk = W_cell{k};

    % Matrix relating kron(x, x) to B(x, x) in this loop
    Wrk = Wr_cell{k};

    % Matrix \delta_{x_k x_k}
    dxkxk = zeros(lk,nxzbk);
    xknTs0 = xTsmat(1,indsxirlk)';
    for scnt = 1:lk

        % Index of next sample
        indTs1 = scnt * nTsk + 1;

        % Next sample
        xknTs1 = xTsmat(indTs1,indsxirlk)';

        % B(x(t_1)+x(t_0), x(t_1)-x(t_0))
        Bx1mx0 = blf(xknTs1+xknTs0,xknTs1-xknTs0);
        
        % Store data
        dxkxk(scnt,:) = Bx1mx0';

        % Store next sample as previous
        xknTs0 = xknTs1;

    end

    % Kronecker product integral matrices
    Ixx_nkTs = zeros(lk,nxztot^2);
    indsIBxwlock = indsIBxwloc{k};
    if sim_w
        IBxkwk_nkTs = zeros(lk,nxzbk);
    end
    IBxktwk_nkTs = zeros(lk,nxzbk);
    IBxkguk_nkTs = zeros(lk,nxzbk);

    for scnt = 1:lk

        % Temp storage
        Ixx_nkTstmp = zeros(nxztot^2,1);
        if sim_w
            IBxw_nkTstmp = zeros(nxzbk,1);
        end
        IBxtw_nkTstmp = zeros(nxzbk,1);
        IBxgu_nkTstmp = zeros(nxzbk,1);

        for j = 1:nTsk

            % Current index
            ind = (scnt-1)*nTsk + j;

            % Extract integrals over current period
            Ibxtmp = Ibxmat(ind,:)';
            if sim_w
                IBxwtmp = IBxwmat(ind,indsIBxwlock)'; 
            end
            IBxtwtmp = IBxtwmat(ind,indsIBxwlock)';
            IBxgutmp = IBxgumat(ind,indsIBxwlock)';

            % For (x, x) integrals -- go from \bar{x} to kron(x,x)
            Ixxtmp = Wrtot * Ibxtmp;

            % Add these integrals to the total
            Ixx_nkTstmp = Ixx_nkTstmp + Ixxtmp;
            if sim_w
                IBxw_nkTstmp = IBxw_nkTstmp + IBxwtmp;
            end
            IBxtw_nkTstmp = IBxtw_nkTstmp + IBxtwtmp;
            IBxgu_nkTstmp = IBxgu_nkTstmp + IBxgutmp;

        end

        % Store in matrix
        Ixx_nkTs(scnt,:) = Ixx_nkTstmp;
        if sim_w
            IBxkwk_nkTs(scnt,:) = IBxw_nkTstmp;
        end
        IBxktwk_nkTs(scnt,:) = IBxtw_nkTstmp;
        IBxkguk_nkTs(scnt,:) = IBxgu_nkTstmp;


    end

    % ***********************
    %       
    % REGRESSION STATE PRESCALING
    %  
    
    hasSk = hasSvec(k);
    if hasSk
        Sk = S_cell{k};
        SkSk = skron(Sk,Sk);
    end

    % ***********************
    %       
    % CALCULATE REGRESSION MATRICES
    %    

    % Indices of all loops besides current one
    if dox3
        indsall = (1:numloops+1)';
    else
        indsall = (1:numloops)';
    end
    indsnotk = indsall(indsall~=k);
    nindsall = size(indsall,1);
    nindsnotk = size(indsnotk,1);

    % Calculate matrices I_{x_k x_j}
    Ixkxj_cell = cell(nindsall,1);
    IBxkxk = zeros(lk, nxzbk);
    for j = indsall'
        indsxirlj = indsxirl{j};
        nxzj = size(indsxirlj,1);
        Ixkxj = subvec(Ixx_nkTs', nxztot, nxztot, indsxirlk, indsxirlj)';
        Ixkxj_cell{j} = Ixkxj;
        if j == k
            IBxkxk = Ixkxj * Wk';
        end
    end


    % ***********************
    %       
    % LEARNING LOOP
    %        

%     % Get the matrix I_{x_k x_k}
%     Ixkxk = Ixkxj_cell{k};
    
    for ik = 1:istarvec(k)

        % Get i + 1
        ikp1 = ik + 1;

        % Get current loop controller K_{i,k}
        Kik = K_cell{k}(:,:,ik);

        % Calculate Q_{i,k} = Q_k + K_{i,k}^T R_{k} K_{i,k}
        Qik = Qk + Kik' * Rk * Kik;

        % Calculate the matrix M_{i,k}
        Mik = Wk * kron(Ink,Bkk*Kik) * Wrk;

        % Calculate matrix I_{B(x_k, x_k)} * M_{i,k}^T
        IBxkxkMik = IBxkxk * Mik';
        
        % FINAL REGRESSION MATRIX
%         ulAlsq = dxkxk - 2 * (IBxkguk_nkTs + IxkxkBkkKik * Mk');
        Aulsq = - 2 * (IBxkguk_nkTs + IBxkxkMik);
        ulAlsq = dxkxk + Aulsq;
        if sim_w
            Alsq_sim = ulAlsq - 2 * IBxkwk_nkTs;
        end
        Alsq = ulAlsq - 2 * IBxktwk_nkTs;        

        % FINAL REGRESSION VECTOR
        blsq = - IBxkxk * vecsym(Qik);

        % LEAST-SQUARES REGRESSION
        cveck = Alsq \ blsq;

        % Calculate corresponding symmetric matrix
        Pik = invvecsym(cveck);

        % Calculate new controller
        Kik = Rk \ (Bkk' * Pik);  

        % Store weights
        cmat_cell{k}(ik,:) = cveck';

        % Store corresponding "P" matrix
        P_cell{k}(:,:,ik) = Pik;

        % Store contoller
        K_cell{k}(:,:,ikp1) = Kik;

        % Store conditioning data
        cond_A_vec_cell{k}(ik) = cond(Alsq);
        cond_AS_vec_cell{k}(ik) = cond(Alsq);

        % STATE PRESCALING -- Perform regression
        if hasSk
            tAlsq = Alsq * SkSk';
            tcveck = tAlsq \ blsq;
            tPik = invvecsym(tcveck);
            PikS = Sk' * tPik * Sk;
            KikS = Rk \ (Bkk' * PikS);
            PS_cell{k}(:,:,ik) = PikS;
            KS_cell{k}(:,:,ikp1) = KikS;
            cond_AS_vec_cell{k}(ik) = cond(tAlsq);            
        end

        % Add one to PI index of this loop
        ivec(k) = ikp1;

        % Update controller in global settings
        u_sett.K_cell = K_cell;

        % Update the PI iteration counter in the global settings
        u_sett.ivec = ivec;

        % LEAST-SQUARES REGRESSION: PERTURBATION TERMS
        if sim_w
            IBxkwkmtwk_nkTs = IBxkwk_nkTs - IBxktwk_nkTs;
        end

        % DEBUGGING: Display PI iteration, controller, condition number
        disp(['  PI ITERATION IN EACH LOOP:  '   num2str(ivec')])
        disp(['cond(\tilde{A}) =        ' num2str(cond(Alsq))])
        disp(['||\tilde{A}|| =          ' num2str(norm(Alsq))])
        if sim_w
            disp(['cond(A) =        ' num2str(cond(Alsq_sim))])
            disp(['||A|| =          ' num2str(norm(Alsq_sim))])   
        end
        disp(['cond(\ul{A}) =        ' num2str(cond(ulAlsq))])
        disp(['||\ul{A}|| =          ' num2str(norm(ulAlsq))])   
        disp(['||\delta_{xx}|| =          ' num2str(norm(dxkxk))])  
        disp(['||I_{B(x,x)}|| =          ' ...
            num2str(norm(IBxkxk))])        
        disp(['||I_{B(x,\tilde{w})}|| =          ' ...
            num2str(norm(IBxktwk_nkTs))])
        if sim_w
            disp(['||I_{B(x,w)}|| =          ' num2str(norm(IBxkwk_nkTs))])  
            disp(['||I_{B(x,w -\tilde{w})}|| =          ' ...
                num2str(norm(IBxkwkmtwk_nkTs))])   
        end
        disp(['||I_{B(x,g(x)u)}|| =          ' ...
            num2str(norm(IBxkguk_nkTs))])
        disp(['||I_{B(x,BK_ix)}|| =          ' ...
            num2str(norm(IBxkxkMik))])
%             num2str(norm((IxkxkBkkKik) * Mk'))])        
        disp(['||W_{i,k}|| =          ' ...
            num2str(norm(Mik))])

        if hasSk
           disp(['cond(\tilde{A}) =        ' num2str(cond(tAlsq)) ...
               '    WITH NORMALIZATION']) 
        end

        Kik

        % DEBUGGING: Display final controller error
        if ik == istarvec(k)
            switch numloops
                case 1
                    Kkstar = lq_data_opt_sim.Kcirl;
                    vPkstar = vecsym(lq_data_opt_sim.P);
                case 2
                    switch k
                        case 1
                            Kkstar = lq_data_opt_sim.lq_data_11.K;
                            vPkstar = ...
                                vecsym(lq_data_opt_sim.lq_data_11.P);
                        case 2
                            Kkstar = lq_data_opt_sim.lq_data_22.K;
                            vPkstar = ...
                                vecsym(lq_data_opt_sim.lq_data_22.P);
                    end
            end        
            eKk = Kik - Kkstar;
            normeKk = norm(eKk);
            disp(['||A v(P*) - b|| =   ' ...
                num2str(norm(Alsq * vPkstar - blsq))])
            disp(['||A v(P_{i*}) - b|| =   ' ...
                num2str(norm(Alsq * cveck - blsq))])               
            disp(['||K_{i*,j} - K_j^*|| =   ' num2str(normeKk)])
        end

    end
    


end

end

% DEBUGGING: Print done learning
disp('***** LEARNING COMPLETE *****')



end                 % END LEARNING PORTION

%%
% *************************************************************************
% *************************************************************************
%
% LOAD, FORMAT FINAL CONTROLLER
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% LOAD
%       

if dolearning

    % Form final controller -- IRL state partition
    switch numloops
        case 1
            Kirl = K_cell{1}(:,:,end);
        case 2
            % Final controllers
            K11 = K_cell{1}(:,:,end);
            K22 = K_cell{2}(:,:,end);
            Kirl = blkdiag(K11, K22);
    end

else

    % Get dEIRL data
    dirl_data = alg_settings.dirl_data;
    lq_data = dirl_data.lq_data;

    % Get composite controller (DIRL coords)
    Kirl = lq_data.Kirl;

end


% ***********************
%       
% FORMAT MULTI-LOOP CONTROLLERS
%   
switch numloops
    
    case 1

        % Nothing

    case 2

        % ***********************
        %       
        % LOAD
        %       

        if dolearning
        
            % Final controllers
            K11 = K_cell{1}(:,:,end);
            K22 = K_cell{2}(:,:,end);
        
        else
        
            % Load final controllers
            K11 = lq_data.lq_data_11.K;
            K22 = lq_data.lq_data_22.K;
        
        end

        % ***********************
        %       
        % FORMAT
        %       
    
        % Extract parts of feedback gain matrix K corresponding to each of
        % the states in the partition
        m1 = loop_cell{1}.ny;
        nx1 = loop_cell{1}.nx;
        if hasintaug
            Kz11 = K11(:,1:m1);
            Ky11 = K11(:,m1+1:2*m1);
            Kr11 = K11(:,2*m1+1:m1+nx1);
        else
            Ky11 = K11(:,1:m1);
            Kr11 = K11(:,m1+1:nx1); 
        end
    
    
        m2 = loop_cell{2}.ny;
        nx2 = loop_cell{2}.nx;
        if hasintaug
            Kz22 = K22(:,1:m2);
            Ky22 = K22(:,m2+1:2*m2);
            Kr22 = K22(:,2*m2+1:m2+nx2);
        else
            Ky22 = K22(:,1:m2);
            Kr22 = K22(:,m2+1:nx2); 
        end        
        
    
        % ***********************
        %       
        % STORE
        %       
    
        % Store (1,1) controller
        lq_data_11.K = K11;
        if hasintaug
            lq_data_11.Kz = Kz11;
        end
        lq_data_11.Ky = Ky11;
        lq_data_11.Kr = Kr11;
        
        % Store (2,2) controller
        lq_data_22.K = K22;
        if hasintaug
            lq_data_22.Kz = Kz22;
        end
        lq_data_22.Ky = Ky22;
        lq_data_22.Kr = Kr22;
        
        % Store individual loop data
        lq_data.lq_data_11 = lq_data_11;
        lq_data.lq_data_22 = lq_data_22;
       
end

% Form composite controller: [z, y, x_r]
K = Kirl * Sxirl;
    
% Individual controllers: z, y, x_r
if hasintaug
    Kz = K(:,1:m);
    Ky = K(:,m+1:2*m);
%     Kr = K(:,2*m+1:m+n-1);
    Kr = K(:,2*m+1:m+nlq);
else
    Ky = K(:,1:m);
    Kr = K(:,m+1:nlq);
end

% Store composite controller -- IRL state partition
lq_data.Kirl = Kirl;

% Store composite controller
lq_data.K = K;
if hasintaug
    lq_data.Kz = Kz;
end
lq_data.Ky = Ky;
lq_data.Kr = Kr; 

  
%%
% *************************************************************************
% *************************************************************************
%
% RUN SIMULATION WITH FINAL CONTROLLER
% 
% *************************************************************************
% *************************************************************************

if dopostlearning

% ***********************
%       
% RUN SIMULATION -- CALL LQ SERVO SIMULATOR
%

% Temp alg_settings struct
alg_settings_tmp = alg_settings;            % Initialize same as parent
alg_settings_tmp.alg = 'lq_servo_inout';
alg_settings_tmp.K = K;
% alg_settings_tmp.model_nom_ind = model_nom_ind;
alg_settings_tmp.model_nom_ind = model_sim_ind; % Shift trim to sim model's

% Run LQ servo simulation
out_data_tmp = alg_lq_servo_inout(alg_settings_tmp, ...
    group_settings, master_settings);


% DEBUGGING: Print done simulating
disp('***** POST-LEARNING SIMULATION COMPLETE *****')

end


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

% Post-learning data
if dopostlearning

    % Store control signal
    out_data.umat = out_data_tmp.umat;
    
    % Time vector
    out_data.tvec = out_data_tmp.tvec;
    
    % Store post-learning state vector
    out_data.xmat = out_data_tmp.xmat;

end

% % LQ data
% out_data.lq_data = lq_data;

% Loop data
out_data.loopdata = loopdata;

% Loop cell
out_data.loop_cell = loop_cell;

% Store indices
out_data.inds = inds;

% Learning data
if dolearning

    if savetrainingdata

        % Control signal learning phase
        out_data.ulmat = ulmat;
        
        % Time vector learning phase
        out_data.tlvec = tlvec;
    
        % State vector learning phase
        out_data.xlmat = xlmat;

        % Control signal learning phase
        out_data.umat = ulmat;
        
        % Time vector learning phase
        out_data.tvec = tlvec;
    
        % State vector learning phase
        out_data.xmat = xlmat;

        % If a prefilter was used, rescale and shifted filtered reference
        % command responses to reflect trim (pre-transformation)
        if pf1nopf0
           xlmat(:,indspf) = (inv(u_sett.sy) * xlmat(:,indspf)')' ...
               +  [xe_sim(indsxr)]';
           out_data.xmat = xlmat;
        end


    end

    % IRL learning data
    out_data.cmat_cell = cmat_cell;
    out_data.P_cell = P_cell;
    out_data.K_cell = K_cell;
    
    % Conditioning data
    out_data.cond_A_vec_cell = cond_A_vec_cell;

    % IRL learning/conditioning data -- with state prescaling
    out_data.hasS = hasS;
    out_data.hasSvec = hasSvec;
    if hasS
        out_data.PS_cell = PS_cell;
        out_data.KS_cell = KS_cell;
        out_data.cond_AS_vec_cell = cond_AS_vec_cell;
    end

else
    
    % IRL learning data
    out_data.cmat_cell = dirl_data.cmat_cell;
    out_data.P_cell = dirl_data.P_cell;
    out_data.K_cell = dirl_data.K_cell;
    
    % Conditioning data
    out_data.cond_A_vec_cell = dirl_data.cond_A_vec_cell;

end

% % Optimal LQ data -- simulation model
% out_data.lq_data_opt_sim = lq_data_opt_sim;

%%
% *************************************************************************
% *************************************************************************
%
% SAVE DIRL LEARNING DATA
%
% *************************************************************************
% *************************************************************************

if dolearning

    % LQ data
    dirl_data.lq_data = lq_data;
    
    % Loop data
    dirl_data.loopdata = loopdata;
    
    % IRL learning data
    dirl_data.cmat_cell = cmat_cell;
    dirl_data.P_cell = P_cell;
    dirl_data.K_cell = K_cell;
    
    % Conditioning data
    dirl_data.cond_A_vec_cell = cond_A_vec_cell;
    
    % Overwrite current controller if desired
    if updatecontroller
    
        % Save data 
        varname = 'dirl_data';
        if ~isfolder(relpath_data)
            mkdir(relpath_data)
        end
        save([relpath_data filename_data], varname);
    
    end

end

% Store DIRL data
out_data.dirl_data = dirl_data;



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

% Control settings
global r_sett;
global u_sett;

% % Extract nominal model
% model_nom = u_sett.model_nom;

% Extract simulation model
model_sim = u_sett.model_sim;

% Get coordinate transformations
sx = u_sett.sx;
su = u_sett.su;
sy = u_sett.sy;
Sxirl_x3 = u_sett.Sxirl_x3;   % [z, y, x_r, x_3] -> [x_1, x_2, x_3]

% Get indices of state partition
inds = u_sett.inds;

% Simulate w = f(x) - Ax for simulation model (=1) or not (=0)
sim_w = u_sett.sim_w;

% Has integral augmentation (=1) or not (=0)
hasintaug = u_sett.hasintaug;

% Initialize empty state derivative vector
xdot = zeros(u_sett.n_sim, 1);

% Extract plant states
xp = x(inds.indsx);

% Get equilibrium point x_e (pre-transformation) -- simulated system
xe = u_sett.xe_sim;

% Get equilibrium control u_e (pre-transformation) -- simulated system
ue = u_sett.ue_sim;

% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% Evaluate drift dynamics
if u_sett.lin1nonlin0
    % System linear
    f_x = model_sim.lin.Ap * tx;
else
    % System nonlinear
    f_x = model_sim.fx(xp);
end

% Evaluate input gain matrix
if u_sett.lin1nonlin0
    % System linear
    g_x = model_sim.lin.Bp;
else
    % System nonlinear
    g_x = model_sim.gx(xp);
end

% Calculate control signal
u = uxt_alg(x, t);

% Calculate \tilde{u} = u - u_{e} (pre-transformation) 
tu = u - ue;


% Evaluate reference trajectory r(t) (pre-transformation)
rt = eval_xr(t, r_sett.r_sett_train);

% Evaluate reference trajectory r(t) (post-transformation)
yr = rt(:,1);
yrp = sy * yr;

% Get y(t) (post-transformation)
yp = sy * x(inds.indsxr);

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(inds.indsxr);

% Get (small-signal) reference command r (post-transformation)
trp = yrp - yep;

% Get (small-signal) output y (post-transformation)
typ = yp - yep;

% If prefilters used, get filtered reference command (post-transformation)
if u_sett.pf1nopf0
    trfp = x(inds.indspf);
end


% State derivatives
if u_sett.lin1nonlin0
    dx = f_x + g_x * tu;
else
    dx = f_x + g_x * u;
end
xdot(inds.indsx) = dx;

% Evaluate integrator state derivative \dot{z} (post-transformation)
if hasintaug
    if u_sett.pf1nopf0 && ~ u_sett.islearning
        zdot = -(trfp - typ);
    else
        zdot = -(trp - typ);
    end
    xdot(inds.indsz) = zdot;
end


% If prefilter inserted, evaluate prefilter dynamics (post-transformation)
if u_sett.pf1nopf0
    pfdot = -diag(u_sett.pfavec) * trfp + diag(u_sett.pfavec) * trp;
    xdot(inds.indspf) = pfdot;
end      

% ***********************
%       
% KRONECKER PRODUCT INTEGRALS -- LEARNING PHASE ONLY
%

if u_sett.islearning

    % Get aggregate state
    xirl = get_loop_state(x,t,0,xe);

    % Kronecker product integrals -- (x, x)
    dIbx = blf(xirl,xirl);

    xdot(inds.indsIbx) = dIbx;

    % Integrals -- (x_k, w_k), (x_k, \tilde{w}_k), (x_k, g_k(x) u)

    % Evaluate control portion of derivative
    if u_sett.lin1nonlin0
        gxu = g_x * tu;
    else
        gxu = g_x * u;
    end

    % Transformed control portion of state derivative
    gxup = sx * gxu;

    % Append integrator portion of g(x)u (zeros)
    if hasintaug
        gxup = [    zeros(m,1)
                    gxup        ];
    end

    % Transform [z, y, x_r, x_3] -> [x_1, x_2, x_3]
    gxupirl = Sxirl_x3 * gxup;

    for k = 1:u_sett.numloops

        % Get x_k (post-trans) -- simulation system
        xk = get_loop_state(x,t,k,xe);

        % Calculate w_k = f_k(x) - A_{kk} x_k
        if sim_w
            wk = get_w(x, t, k, 0, 0);
        end

        % Calculate 
        % \tilde{w}_k = \tilde{f}_k(x) - \tilde{A}_{kk} \tilde{x}_k
        twk = get_w(x, t, k, 1, 0);

        % Get coords of this loop in the DIRL state partition
        indsxirlk = inds.indsxirl{k};

        % Calculate g_{k}(x) u
        gxupk = gxupirl(indsxirlk);
        
        % Calculate B(x_k, w_k)
        if sim_w
            dIBxkwk = blf(xk,wk);
        end

        % Calculate B(x_k, \tilde{w}_k)
        dIBxktwk = blf(xk,twk);

        % Calculate B(x_k, g_{k}(x) u)
        dIBxkguk = blf(xk,gxupk);

        % Store derivative terms
        if sim_w
            xdot(inds.indsIBxw{k}) = dIBxkwk;
        end
        xdot(inds.indsIBxtw{k}) = dIBxktwk;
        xdot(inds.indsIBxgu{k}) = dIBxkguk;

    end


    % Output integrals \int y
    if hasintaug
        xdot(inds.indsIy) = typ;
    end


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

function [u, nt] = uxt_alg(x, t)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Control settings
global r_sett;
global u_sett;


% Get coordinate transformations
% sx = u_sett.sx;
su = u_sett.su;
sy = u_sett.sy;
invSxirl = u_sett.invSxirl;     % [x_1, x_2] -> [z, y, x_r]

% Get loop settings
loop_cell = u_sett.loop_cell;

% % Get indices of state partition
% inds = u_sett.inds;

% Has integral augmentation (=1) or not (=0)
hasintaug = u_sett.hasintaug;


% Get equilibrium point x_e (pre-transformation) -- simulated system
xe = u_sett.xe_sim;

% Get equilibrium control u_e (pre-transformation) -- simulated system
ue = u_sett.ue_sim;


% Evaluate exploration noise (post-transformation)
ntp = eval_noise(t, m, u_sett.noise);


% Calculate control in each loop
tup = zeros(m,1);
for k = 1:u_sett.numloops

    % Get indices of state, output vector to pull
    indsxk = loop_cell{k}.indsx;
    indsyk = loop_cell{k}.indsy;

    % Get number of states, outputs associated with this loop
    nxk = loop_cell{k}.nx;
    nyk = loop_cell{k}.ny;

    % Get PI iteration number of current loop
    ik = u_sett.ivec(k);

    % Get the state vector corresponding to this loop
    [xk, zk, ek] = get_loop_state(x, t, k, xe);

    % Get rest of state associated with this loop
    if hasintaug
        txrk = xk(2*nyk+1:nyk+nxk);
    else
        txrk = xk(nyk+1:nxk);
    end
    txrk = txrk(:);

    % Get contoller
    Kk = u_sett.K_cell{k}(:,:,ik);
    
    switch u_sett.numloops
        case 1
            % Get controller for the LQ servo coords
            Kklq = Kk * invSxirl;
        case 2
            Kklq = Kk;
    end
    if hasintaug
        Kz = Kklq(:,1:nyk);
        Ky = Kklq(:,nyk+1:2*nyk);
        Kr = Kklq(:,2*nyk+1:nyk+nxk);
    else
        Ky = Kklq(:,1:nyk);
        Kr = Kklq(:,nyk+1:nxk);
    end

    % Calculate control (post-transformation) 
    if hasintaug
    %     tup(indsyk) = Kz * (-zk) + Ky * ek - Kr * txrk;
        tup(indsyk) = Kz * (-zk) + Ky * (-ek) - Kr * txrk;
    else
        tup(indsyk) = Ky * (-ek) - Kr * txrk;
    end
    
end

% Insert probing noise if desired
if u_sett.islearning
    tup = tup + ntp;
end

% Calculate linear control (pre-tranformation)
tu = su \ tup;

% Calculate exploration noise (pre-transformation)
nt = su \ ntp;

% Calculate final control u = u_e + \tilde{u}
u = ue + tu;    


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE CONTROL SIGNAL u(x, t) FOR STRING OF SAMPLES
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function [umat, enmat] = uxt_all(xmat, tvec)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Number of data points
ltvec = size(tvec,1);

% Initialize empty matrix
umat = zeros(ltvec, m);
enmat = zeros(ltvec, m);

% Calculate control
for k = 1:ltvec
    
    % Get time
    t = tvec(k);

    % Get state vector 
    xs = xmat(k,:)';
    
    % Evaluate control 
    [u, en] = uxt_alg(xs, t);
                
    % Store control
    umat(k,:) = u';

    % Store noise
    enmat(k,:) = en';

end








