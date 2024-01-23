function ux = eval_u(x, t, u_sett)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE FEEDBACK CONTROL LAW u(x, t)
%
% Brent Wallace  
%
% 2021-11-06
%
% This program, given a state vector x in R^n and preset tag evaluates the
% a specified policy u(x, t). 
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% x         (n-dim. Vector) Current value of the state.
% t         (double, OPTIONAL) Current time. Required only if policy is
%           time-varying.
%
% u_sett    (Struct) contains settings pertaining to the policy. Must have \
%           the following fields:
%   tag     (String) contains the string of the specific preset to evaluate
%           u(x, t) for.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% ux    Initial stabilizing policy u(x), evaluated at x.
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

switch u_sett.tag

    % ***********************
    %
    % ZERO CONTROL SIGNAL
    %
    
    case '0'
        
        ux = 0;

    % ***********************
    %
    % LQ SERVO
    %
    % Required Fields to 'u_sett'
    %   n               (Integer) Number of states (without integral
    %                   augmentation)
    %   m               (Integer) Number of controls
    %   hasintaug       (Bool) Controller has integral augmentation (=1) or
    %                   not (=0)
    %   K               CASE: INTEGRAL AUGMENTATION -- 
    %                       (m x m + n Matrix) Contains the aggregate LQ
    %                       servo controller. State must be partitioned
    %                           x = [z^T, y^T, x_r^T]^T
    %                   CASE: NO INTEGRAL AUGMENTATION -- 
    %                       (m x n Matrix) Contains the aggregate LQ
    %                       servo controller. State must be partitioned
    %                           x = [y^T, x_r^T]^T
    %   xe_ctrl         (n-dim Vector) Contains equilibrium state x_e
    %                   (pre-transformation)
    %   ue_ctrl         (m-dim Vector) Contains equilibrium control u_e
    %                   (pre-transformation)
    %   sx              (n x n Matrix) State transformation to LQ servo
    %                   units.
    %   su              (m x m Matrix) Control transformation to LQ servo
    %                   units. 
    %   sy              (m x m Matrix) Output transformation to LQ servo
    %                   units.    
    %   inds            (Struct) Contains indices of the plant states x_p,
    %                   integral augmentation states z (if used)
    %                   where the rest of the state x_r may be empty.
    %   r_sett          (Struct, OPTIONAL) Reference command settings.
    %   t               (Double, OPTIONAL) Current time. Only needed if
    %                   reference command used.
    %   pf1nopf0        (Bool, OPTIONAL) Use reference command prefilter
    %                   (=1) or not (=0). Required only if prefilters are
    %                   used.
    %   
    %
    
    case 'lq_servo'
       
        % Get system dimensions
        n = u_sett.n;
        m = u_sett.m;
        
        % Reference command settings
        hasrsett = isfield(u_sett, 'r_sett');
        if hasrsett
            r_sett = u_sett.r_sett;            
        end        
        
        % Get coordinate transformations
        sx = u_sett.sx;
        su = u_sett.su;
        sy = u_sett.sy;
        
        % Indices of state vector partition
        inds = u_sett.inds;

        % Check for prefilter
        haspf = isfield(u_sett, 'pf1nopf0');

        % Has integral augmentation (=1) or not (=0)
        hasintaug = u_sett.hasintaug;        

        % Get contoller
        K = u_sett.K;
        
        % Extract parts of feedback gain matrix K corresponding to each of
        % the states in the partition
        if hasintaug
            Kz = K(:,1:m);
            Ky = K(:,m+1:2*m);
%             Kr = K(:,2*m+1:m+n-1);
            Kr = K(:,2*m+1:m+n);
        else
            Ky = K(:,1:m);
            Kr = K(:,m+1:n);   
        end

        % Number of states in x_r
        nxr = size(Kr, 2);
        
        % ***********************
        %       
        % LQ REFERENCE COMMAND FOLLOWING
        %   
        
        
        % Get equilibrium point x_e (pre-transformation) -- nominal system
        xe = u_sett.xe_ctrl;
        
        % Get equilibrium control u_e (pre-transformation) -- nominal
        % system
        ue = u_sett.ue_ctrl;

        % Get equilibrium value of the output y_{e} (pre-transformation)
        ye = xe(inds.indsxr);

        % Evaluate reference trajectory r(t) (pre-transformation)
        if hasrsett
            rt = eval_xr(t, r_sett);
        else
            rt = ye;
        end
        
        % Evaluate reference trajectory r(t) (post-transformation)
        yr = rt(:,1);
        yrp = sy * yr;

        % Get equilibrium value of the output y_{e} (post-transformation)
        yep = sy * ye;

        % Get (small-signal) reference command r (post-transformation)
        trp = yrp - yep;
        
        % If prefilters used, get filtered reference command
        % (post-transformation)
        if haspf
            if u_sett.pf1nopf0
                trfp = x(inds.indspf);
            end
        end
       
        % Extract plant states x_p (pre-transformation)
        xp = x(inds.indsx);
        
        % Extract the integral augmentation states z (post-transformation)
        if hasintaug
            z = x(inds.indsz);
        end
        
        % Get state of linear system (pre-transformation) 
        % \tilde{x} = x - x_e
        tx = xp - xe;
        
        % Apply coordinate transformation
        % x' = S_x x
        %    = [    y
        %           x_r
        %           x_3     ]
        txp = sx * tx;
        typ = txp(1:m);
%         txr = txp(m+1:size(K,2)-m);
        txr = txp(m+1:m+nxr);
        
        % Calculate (small-signal) tracking error e = y - r
        % (post-transformation)
        if haspf
            if u_sett.pf1nopf0
                e = -(trfp - typ);
            else
                e = -(trp - typ);
            end
        else
            e = -(trp - typ);
        end
        
        % Calculate control (post-transformation) 
        if hasintaug
            tup = Kz * (-z) + Ky * (-e) - Kr * txr;
        else
            tup = Ky * (-e) - Kr * txr;
        end
        
        % Calculate linear control (pre-tranformation)
        tu = su \ tup;
        
        % Calculate final control u = u_e + \tilde{u}
        ux = ue + tu;


    
    % *********************************************************************
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %  
    % *********************************************************************
    % *********************************************************************
    
    otherwise
        
        error('*** ERROR: PRESET TAG NOT RECOGNIZED ***');

end