function out_data = f_CLTFM_std_pk(in_data)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% FORM CLOSED-LOOP TRANSFER FUNCTION MATRICES -- INNER-OUTER LOOP STRUCTURE
%
% Brent Wallace  
%
% 2022-10-31
%
% This program, given state-space representations of a plant P, inner-loop
% controller K_i, outer-loop controller K_o, and inner-loop state feedback
% matrix M_i, constructs state-space representations of all relevant
% closed-loop maps in the feedback loop.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%   
% in_data                  (Struct) Input data. Has the following fields:
%
%   P                       (m x p State-Space Object) Linear plant
%                           dynamical system. Order = n.
%   K                      (p x m State-Space Object) Inner-loop
%                           controller dynamical system. Order = n_k.
%   W                       (p x p State-Space Object, OPTIONAL) Reference
%                           command pre-filter. Order = n_w.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) Output data. Has the following fields:
%
% *** OPEN-LOOP MAPS:
%
%   Le                      (ss Obj.) OL map e -> y (broken at error)
%   Lu                      (ss Obj.) OL map u_p -> u (broken at controls)
%
% *** CLOSED-LOOP MAPS:
%
%   KSe                     (ss Obj.) CL map r -> u
%   Te                      (ss Obj.) CL map r -> y
%   Se                      (ss Obj.) CL map r -> e
%   KSe                     (ss Obj.) CL map r -> u
%
%   Tu                      (ss Obj.) CL map d_i -> u
%   Su                      (ss Obj.) CL map d_i -> u_p
%   PSu                     (ss Obj.) CL map d_i -> y
%
%   Tdou                    (ss Obj.) CL map d_o -> u
%   Tdoy                    (ss Obj.) CL map d_o -> y
%   Tdoyp                   (ss Obj.) CL map d_o -> y_p
%   Tdoe                    (ss Obj.) CL map d_o -> e
%
% % %   Tnu                     (ss Obj.) CL map n -> u
% % %   Tny                     (ss Obj.) CL map n -> y
% % %   Tne                     (ss Obj.) CL map n -> e
%
% (WITH PRE-FILTER ONLY)
%
%   WKSe                    (ss Obj.) CL map r -> u
%   WTe                     (ss Obj.) CL map r -> y
%   WSe                     (ss Obj.) CL map r -> e
%
% *************************************************************************
%
% NOTES ON LOOP STRUCTURE
%
% *************************************************************************
%
% ***** STATE PARTITION
%
% For all of the closed-loop maps here, the state is partitioned as
%
%   x = [   x_p
%           x_k     ]     \in R^{n + n_k}    
%
% Where x_p \in R^{n} denotes the plant state, x_k \in R^{n_k} denotes the
% controller state.
%
% If a reference command pre-filter W is used, then the state is
% partitioned as 
%
%   x = [   x_p
%           x_k         \in R^{n + n_k + n_w}
%           x_w     ]  
%
% Where x_w \in R^{n_w} is the pre-filter state.
%
% ***** INPUTS CONSIDERED
%
% r                         (m-dim) Reference command signal.
% d_i                       (p-dim) Plant input disturbance signal.
% d_o                       (m-dim) Plant output disturbance signal.
% % % n                         (m-dim) Sensor noise signal.
%
% ***** OUTPUTS CONSIDERED
%
% e                         (m-dim) Error signal.
% u                         (m-dim) Control signal (pre-input disturbance).
% u_p                       (p-dim) Plant input signal.
% y                         (m-dim) Plant output + output disturbance
%                           signal.
% y_p                       (m-dim) Plant output (pre-output disturbance).
%
% ***** KEY RELATIONSHIPS:
% 
% y = y_p + d_o
% e = r - y
% u_p = u + d_i
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
% INIT   
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
% *************************************************************************
%
% EXTRACT INPUT ARGUMENTS
%
% *************************************************************************
% *************************************************************************

% Plant P
P = in_data.P;

% Controller K
K = in_data.K;


% ***********************
%       
% PRE-FILTER
%

has_pf = isfield(in_data, 'W');
if has_pf
    W = in_data.W;
end


% *************************************************************************
% *************************************************************************
%
% EXTRACT STATE-SPACE MATRICES AND SYSTEM DIMENSIONS   
%
% *************************************************************************
% *************************************************************************

% ***********************
%       
% PLANT
%

% State-space matrices
Ap = P.A;
Bp = P.B;
Cp = P.C;
Dp = P.D;

% Dimensions
n = size(Ap,1);
nu = size(Bp,2);
ny = size(Cp,1);

% ***********************
%       
% CONTROLLER K
%

% State-space matrices
Ak = K.A;
Bk = K.B;
Ck = K.C;
Dk = K.D;

% Dimensions
nk = size(Ak,1);

% Total order of closed-loop system (without pre-filter)
ncl = n + nk;

% ***********************
%       
% PRE-FILTER
%

if has_pf
    % State-space matrices
    Aw = W.A;
    Bw = W.B;
    Cw = W.C;
    Dw = W.D;
    
    % Dimensions
    nw = size(Aw,1);
end



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% OPEN-LOOP MAPS    
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
%
% LOOP BROKEN AT CONTROLS L_u: u_p -> u 
%
% State: [x_p^T x_k^T]^T
%
% *************************************************************************
% *************************************************************************

A_Lu = [    Ap      zeros(n,nk)                  
            -Bk*Cp  Ak               ];
B_Lu = [    Bp
            -Bk*Dp          ];
C_Lu = [    -Dk*Cp     Ck  ];
D_Lu = -Dk*Dp;

% Set output
out_data.Lu = ss(A_Lu,B_Lu,C_Lu,D_Lu);

% *************************************************************************
% *************************************************************************
%
% LOOP BROKEN AT ERROR L_e: e -> y 
%
% State: [x_p^T x_k^T]^T
%
% *************************************************************************
% *************************************************************************


A_Le = [    Ap              Bp*Ck
            zeros(nk,n)     Ak      ];
B_Le = [    Bp*Dk
            Bk      ];
C_Le = [    Cp              Dp*Ck   ];
D_Le = Dp*Dk;

% Set output
out_data.Le = ss(A_Le,B_Le,C_Le,D_Le);


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CLOSED-LOOP MAPS    
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
% *************************************************************************
%
% CLOSED-LOOP "A" MATRIX    
%
% *************************************************************************
% *************************************************************************

% ***********************
%       
% DEFINE FEEDBACK GAIN TERMS
%
% These closed-loop feedback matrix terms appear frequently in the
% derivations.
%   

Ui = inv(eye(nu) + Dk * Dp);
Uo = inv(eye(ny) + Dp * Dk);

% % DEBUGGING: Make these identity matrices
% Ui = eye(nu);
% Uo = eye(ny);

% ***********************
%       
% CLOSED-LOOP "A" MATRIX
%   

Acl = [ Ap-Bp*Ui*Dk*Cp          Bp*Ui*Ck
        -Bk*(Cp-Dp*Ui*Dk*Cp)    Ak-Bk*Dp*Ui*Ck  ];


% *************************************************************************
% *************************************************************************
%
% CLOSED-LOOP "B" MATRICES    
%
% *************************************************************************
% *************************************************************************

% ***********************
%       
% REFERENCE SIGNAL r
%

Br = [  Bp*Ui*Dk
        Bk*Uo       ];

% ***********************
%       
% INPUT DISTURBANCE SIGNAL d_i
%

Bdi = [ Bp*Ui
        -Bk*Dp*Ui       ];

% ***********************
%       
% OUTPUT DISTURBANCE SIGNAL d_o
%

Bdo = [ -Bp*Ui*Dk
        -Bk*Uo          ];

% % ***********************
% %       
% % INNER-LOOP NOISE n_i
% %
% 
% Bni = [ -Bp*Ui*Dk
%         Bk
%         Bk*Dp*Ui*Dk     ];


% *************************************************************************
% *************************************************************************
%
% CLOSED-LOOP "C" MATRICES    
%
% *************************************************************************
% *************************************************************************

% ***********************
%       
% PLANT INPUT u_p
%

% x_p -> u_p
Cxpup = -Ui*Dk*Cp;
% x_k -> u_p
Cxkup = Ui*Ck;

% x -> u_p
Cup = [ Cxpup   Cxkup   ];


% ***********************
%       
% PLANT OUTPUT y_p
%

% x_p -> y_p
Cxpyp = Cp+Dp*Cxpup;
% x_k -> y_p
Cxkyp = Dp*Cxkup;

% x -> y_p
Cyp = [ Cxpyp  Cxkyp   ];


% ***********************
%       
% PLANT OUTPUT + DISTURBANCE y
%

% x_p -> y
Cxpy = Cxpyp;
% x_k -> y
Cxky = Cxkyp;

% x -> y
Cy = [ Cxpy    Cxky   ];


% ***********************
%       
% ERROR e
%

% x_p -> e
Cxpe = -Cxpy;
% x_k -> e
Cxke = -Cxky;

% x -> e
Ce = [ Cxpe   Cxke   ];


% ***********************
%       
% CONTROL SIGNAL u
%

% x_p -> u
Cxpu = Dk*Cxpe;
% x_k -> u_o
Cxku = Ck+Dk*Cxke;

% x -> u_o
Cu = [ Cxpu   Cxku   ];



% *************************************************************************
% *************************************************************************
%
% CLOSED-LOOP "D" MATRICES    
%
% *************************************************************************
% *************************************************************************


% ***********************
%       
% PLANT INPUT u_p
%

% r -> u_p
Drup = Ui*Dk;
% d_i -> u_p
Ddiup = Ui;
% d_o -> u_p
Ddoup = -Ui*Dk;
% % n_i -> u_p
% Dniup = -Ui*Dk;


% ***********************
%       
% PLANT OUTPUT y_p
%

% r -> y_p
Dryp = Dp*Drup;
% d_i -> y_p
Ddiyp = Dp*Ddiup;
% d_o -> y_p
Ddoyp = Dp*Ddoup;
% % n_i -> y_p
% Dniyp = Dp*Dniup;


% ***********************
%       
% PLANT OUTPUT + DISTURBANCE y
%

% r -> y
Dry = Dryp;
% d_i -> y
Ddiy = Ddiyp;
% d_o -> y
Ddoy = Uo;
% % n_i -> y
% Dniy = Dniyp;


% ***********************
%       
% ERROR e
%

% r -> e
Dre = Uo;
% d_i -> e
Ddie = -Ddiy;
% d_o -> e
Ddoe = -Ddoy;
% % n_i -> e
% Dnie = -Dniy;



% ***********************
%       
% CONTROL SIGNAL u
%

% r -> u
Dru = Drup;
% d_i -> u
Ddiu = -Ui*Dk*Dp;
% d_o -> u
Ddou = Ddoup;
% % n_i -> u
% Dniu = Dniup;


%%
% *************************************************************************
% *************************************************************************
%
% FORM CLOSED-LOOP MAPS
%
% *************************************************************************
% *************************************************************************

% ***********************
%       
% PLANT OPEN LOOP
%

%   P
out_data.P = P;

% ***********************
%       
% CONTROLLER OPEN LOOP
%

%   K
out_data.K = K;

% ***********************
%       
% ERROR e
%

%   KSe                     (ss Obj.) CL map r -> u
out_data.KSe = ss(Acl,Br,Cu,Dru);
%   Te                      (ss Obj.) CL map r -> y
out_data.Te = ss(Acl,Br,Cy,Dry);
%   Se                      (ss Obj.) CL map r -> e
out_data.Se = ss(Acl,Br,Ce,Dre);


% ***********************
%       
% CONTROLS u
%

%   Tu                      (ss Obj.) CL map d_i -> u
out_data.Tu = ss(Acl,Bdi,Cu,Ddiu);
%   Su                      (ss Obj.) CL map d_i -> u_p
out_data.Su = ss(Acl,Bdi,Cup,Ddiup);
%   PSu                     (ss Obj.) CL map d_i -> y
out_data.PSu = ss(Acl,Bdi,Cy,Ddiy);


% ***********************
%       
% OUTPUT DISTURBANCE d_o
%

%   Tdou                    (ss Obj.) CL map d_o -> u
out_data.Tdou = ss(Acl,Bdo,Cu,Ddou);
%   Tdoy                    (ss Obj.) CL map d_o -> y
out_data.Tdoy = ss(Acl,Bdo,Cy,Ddoy);
%   Tdoyp                   (ss Obj.) CL map d_o -> y_p
out_data.Tdoyp = ss(Acl,Bdo,Cyp,Ddoyp);
%   Tdoe                    (ss Obj.) CL map d_o -> e
out_data.Tdoe = ss(Acl,Bdo,Ce,Ddoe);


% % ***********************
% %       
% % INNER-LOOP NOISE n_i
% %
% 
% %   Tniu                    (ss Obj.) CL map n_i -> u
% out_data.Tniu = ss(Acl,Bni,Cu,Dniu);
% %   Tniy                    (ss Obj.) CL map n_i -> y
% out_data.Tniy = ss(Acl,Bni,Cy,Dniy);
% %   Tnie                    (ss Obj.) CL map n_i -> e
% out_data.Tnie = ss(Acl,Bni,Ce,Dnie);


% ***********************
%       
% PRE-FILTER MAPS
%

if has_pf
    
    % Closed-loop "A" matrix with pre-filter states
    Aclw = [    Acl             Br*Cw
                zeros(nw,ncl)   Aw      ]; 

    % Closed-loop "B" matrix with pre-filter states
    Brw = [ Br*Dw
            Bw      ];

    %   WKSe                    (ss Obj.) CL map r -> u
    out_data.WKSe = ss(Aclw,Brw,[Cu Dru*Cw],Dru*Dw);
    %   WTe                     (ss Obj.) CL map r -> y
    out_data.WTe = ss(Aclw,Brw,[Cy Dry*Cw],Dry*Dw);
    %   WSe                     (ss Obj.) CL map r -> e
    out_data.WSe = ss(Aclw,Brw,[Ce Dre*Cw],Dre*Dw);

end


% ***********************
%       
% CLOSED-LOOP "A" MATRIX
%

out_data.Acl = Acl;
