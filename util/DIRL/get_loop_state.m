% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% GET PORTION OF STATE VECTOR CORRESPONDING TO THE CURRENT LOOP
%
% Brent Wallace
%
% 2023-03-13
%
% (Note: After shifting by trim and performing coordinate transformation)
%
% State partition of input:
%
%   x = [   x_p     Plant states (pre-transformation) \in R^{n}
%           z       (IF APPLICABLE) Integrator states \in R^{m}
%           x_{pf}  (IF APPLICABLE) Pre-filter states \in R^{m}
%           x_{ir}  Integral reinforcement functions \in R^{'numloops'} ]
%
% State partition of output
%   
%   \tilde{x}_{k}^{'} =
%       [   \tilde{x}_{v,k}^{'} Plant states (shifted by trim,
%                               post-transformation
%           z_{k}               (IF APPLICABLE) Integrator states 
%                               associated with this loop
%           x_{pf,k}    ]       (IF APPLICABLE) Pre-filter states 
%                               associated with this loop
%           
%       
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function [xk, zk, ek] = get_loop_state(x, t, k, xe)

% % Global variables
% global sys;
% n = sys.n;
% m = sys.m;

% Control settings
global r_sett;
global u_sett;

% Get coordinate transformations
sx = u_sett.sx;
% sxh = u_sett.sxh;
% su = u_sett.su;
sy = u_sett.sy;
Sxirl_x3 = u_sett.Sxirl_x3;   % [z, y, x_r, x_3] -> [x_1, x_2, x_3]

% Indices of state vector partition
inds = u_sett.inds;

% Get loop settings
loop_cell = u_sett.loop_cell;

% Has integral augmentation (=1) or not (=0)
hasintaug = u_sett.hasintaug;

% % Get loop settings -- master partition
% loop_cell_m = u_sett.loop_cell_m;

% Extract plant states x_p (pre-transformation)
xp = x(inds.indsx);

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

% % Extract the plant states x_p (pre-transformation)
% xp = x(inds.indsx);
% 
% % Extract the integral augmentation states z (post-transformation)
% z = x(inds.indsz);
% 
% % Get state of linear system (pre-transformation) 
% % \tilde{x} = x - x_e
% tx = xp - xe;

% Calculate (small-signal) tracking error e = y - r (post-transformation)
if u_sett.pf1nopf0
    e = -(trfp - typ);
else
    e = -(trp - typ);
end


% Extract integral errors (post-transformation)
if hasintaug
    zp = x(inds.indsz);
end

% Extract output integrals \int y (post-transformation)
% if u_sett.islearning
%     Ityp = x(inds.indsIy);
% else
%     Ityp = zeros(m,1);
% end
if hasintaug
    Ityp = x(inds.indsIy);
end

% % If prefilters used, get filtered reference command (pre-transformation)
% if u_sett.pf1nopf0
%     trf = sy \ x(inds.indspf);
% end

% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;


% Apply coordinate transformation
% x' = S_x x
%    = [    y
%           x_r
%           x_3       ]
txp = sx * tx;


% Check if it is desired to pull entire state or not
doall = k == 0;

% Get aggregate state in LQ servo coords: [z, y, x_r, x_3]
if hasintaug
    xlq = [ Ityp
            txp     ];
else
    xlq = txp;
end

% Transform to DIRL coords: [z, y, x_r, x_3] -> [x_1, x_2, x_3]
xirl = Sxirl_x3 * xlq;

% Extract indices corresponding to the current loop
if ~doall

    % Get indices of state vector to pull
    xk = xirl(inds.indsxirl{k});

    % Get indices of output vector to pull
%     indsxk = loop_cell{k}.indsx;
    indsyk = loop_cell{k}.indsy;

    % Pull appropriate state indices
    if hasintaug
        zk = zp(indsyk);
    else
        zk = nan;   % NOT APPLICABLE
    end
    ek = e(indsyk);

else

    % Return aggragate states
    xk = xirl;
    ek = e;
    if hasintaug
        zk = zp;
    else
        zk = nan;   % NOT APPLICABLE
    end    
    

end





