function out_data = f_CLTFM_io(in_data)
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
%   Ki                      (p x m State-Space Object) Inner-loop
%                           controller dynamical system. Order = n_i.
%   Ko                      (m x n_{x_r} State-Space Object) Outer-loop
%                           controller dynamical system. Order = n_o.
%   Mi                      (n_{x_r} x n Matrix) Matrix which reads out the
%                           states to be fed back to the inner loop. For
%                           example, if the system has order n = 5 and we
%                           desire to feed back states 2 and 3, then
%                           n_{x_r} = 2 and M_i would be:
%                               Mi = [  0 1 0 0 0
%                                       0 0 1 0 0   ].
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
%   KoSe                    (ss Obj.) CL map r -> u_o
%   Trxr                    (ss Obj.) CL map r -> x_r
%   Trui                    (ss Obj.) CL map r -> u_i
%
%   Tu                      (ss Obj.) CL map d_i -> u
%   Su                      (ss Obj.) CL map d_i -> u_p
%   PSu                     (ss Obj.) CL map d_i -> y
%   KoPSu                   (ss Obj.) CL map d_i -> u_o
%   Tdixr                   (ss Obj.) CL map d_i -> x_r
%   Tdiui                   (ss Obj.) CL map d_i -> u_i
%
%   Tdou                    (ss Obj.) CL map d_o -> u
%   Tdoy                    (ss Obj.) CL map d_o -> y
%   Tdoyp                   (ss Obj.) CL map d_o -> y_p
%   Tdoe                    (ss Obj.) CL map d_o -> e
%   Tdouo                   (ss Obj.) CL map d_o -> u_o
%   Tdoxr                   (ss Obj.) CL map d_o -> x_r
%   Tdoui                   (ss Obj.) CL map d_o -> u_i
%
%   Tniu                    (ss Obj.) CL map n_i -> u
%   Tniy                    (ss Obj.) CL map n_i -> y
%   Tnie                    (ss Obj.) CL map n_i -> e
%   Tniuo                   (ss Obj.) CL map n_i -> u_o
%   Tnixr                   (ss Obj.) CL map n_i -> x_r
%   Tniei                   (ss Obj.) CL map n_i -> e_i
%   Tniui                   (ss Obj.) CL map n_i -> u_i 
%
% (WITH PRE-FILTER ONLY)
%
%   WKSe                    (ss Obj.) CL map r -> u
%   WTe                     (ss Obj.) CL map r -> y
%   WSe                     (ss Obj.) CL map r -> e
%   WKoSe                   (ss Obj.) CL map r -> u_o
%   WTrxr                   (ss Obj.) CL map r -> x_r
%   WTrui                   (ss Obj.) CL map r -> u_i
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
%           x_i         \in R^{n + n_i + n_o}
%           x_o     ]       
%
% Where x_p \in R^{n} denotes the plant state, x_i \in R^{n_i} denotes the
% inner-loop controller state, and x_o \in R^{n_o} denotes the outer-loop
% controller state.
%
% If a reference command pre-filter W is used, then the state is
% partitioned as 
%
%   x = [   x_p
%           x_i         \in R^{n + n_i + n_o + n_w}
%           x_o 
%           x_w     ]  
%
% Where x_w \in R^{n_w} is the pre-filter state.
%
% ***** INPUTS CONSIDERED
%
% r                         (m-dim) Reference command signal.
% d_i                       (p-dim) Plant input disturbance signal.
% d_o                       (m-dim) Plant output disturbance signal.
% n_i                       (n_{x_r}-dim) noise to inner-loop controller.
%
% ***** OUTPUTS CONSIDERED
%
% e                         (m-dim) Error signal.
% u                         (m-dim) Control signal (pre-input disturbance).
% u_p                       (p-dim) Plant input signal.
% y                         (m-dim) Plant output + output disturbance
%                           signal.
% y_p                       (m-dim) Plant output (pre-output disturbance).
% x_r                       (n_{x_r}-dim) Number of states fed back from
%                           the plant to the inner-loop controller K_i.
% e_i                       (n_{x_r}-dim) Input to inner-loop controller
%                           K_i (inner-loop states + inner-loop noise n_i).
%
% ***** KEY RELATIONSHIPS:
% 
% y = y_p + d_o
% e = r - y
% u = u_o - u_i
% u_p = u + d_i
% x_r = M_i * x_p
% e_i = x_r + n_i
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

% Inner-loop controller K_i
Ki = in_data.Ki;

% Outer-loop controller K_o
Ko = in_data.Ko;

% Inner-loop state feedback matrix M_i
Mi = in_data.Mi;

% Check if inner-loop present
is_io = sum(abs(Mi(:))) > 0;

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
nxr = size(Mi,1);

% ***********************
%       
% INNER-LOOP CONTROLLER K_i
%

% State-space matrices
Ai = Ki.A;
Bi = Ki.B;
Ci = Ki.C;
Di = Ki.D;

% Dimensions
ni = size(Ai,1);

% ***********************
%       
% OUTER-LOOP CONTROLLER K_o
%

% State-space matrices
Ao = Ko.A;
Bo = Ko.B;
Co = Ko.C;
Do = Ko.D;

% Dimensions
no = size(Ao,1);

% Total order of closed-loop system (without pre-filter)
ncl = n + ni + no;

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
% State: [x_p^T x_i^T x_o^T]^T
%
% *************************************************************************
% *************************************************************************

A_Lu = [    Ap      zeros(n,ni)     zeros(n,no)
            Bi*Mi   Ai              zeros(ni,no)
            -Bo*Cp  zeros(no,ni)    Ao              ];
B_Lu = [    Bp
            zeros(ni,nu)
            -Bo*Dp          ];
C_Lu = [    -(Di*Mi+Do*Cp)  -Ci     Co  ];
D_Lu = -Do*Dp;

% Set output
out_data.Lu = ss(A_Lu,B_Lu,C_Lu,D_Lu);

% *************************************************************************
% *************************************************************************
%
% LOOP BROKEN AT ERROR L_e: e -> y 
%
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% FORM CLOSED-LOOP MAP u_o -> y
%
% *************************************************************************

% Inner open loop u -> u_i
% State: [x_p^T x_i^T]^T
Aoli = [    Ap      zeros(n,ni)
            Bi*Mi   Ai          ];
Boli = [    Bp
            zeros(ni,nu)        ];
Coli = [    Di*Mi   Ci  ];
Doli = zeros(nu);

% Closed-loop u_o -> y
% State: [x_p^T x_i^T]^T
Acli = Aoli - Boli * Coli;
B_Ti = Boli;
C_Ti = [    Cp-Dp*Di*Mi     -Dp*Ci  ];
D_Ti = Dp;

% *************************************************************************
%
% FORM OPEN-LOOP MAP L_e: e -> y
%
% State: [x_p^T x_i^T x_o^T]^T
%
% *************************************************************************

A_Le = [    Acli            B_Ti*Co
            zeros(no,n+ni)  Ao      ];
B_Le = [    B_Ti*Do
            Bo      ];
C_Le = [    C_Ti    D_Ti*Co     ];
D_Le = D_Ti*Do;

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

Ui = inv(eye(nu) + Do * Dp);
Uo = inv(eye(ny) + Dp * Do);

% % DEBUGGING: Make these identity matrices
% Ui = eye(nu);
% Uo = eye(ny);

% ***********************
%       
% CLOSED-LOOP "A" MATRIX
%   

Acl = [ Ap-Bp*Ui*(Di*Mi+Do*Cp)          -Bp*Ui*Ci       Bp*Ui*Co
        Bi*Mi                           Ai              zeros(ni,no)
        -Bo*(Cp-Dp*Ui*(Di*Mi+Do*Cp))    Bo*Dp*Ui*Ci     Ao-Bo*Dp*Ui*Co  ];


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

Br = [  Bp*Ui*Do
        zeros(ni,ny)
        Bo*Uo       ];

% ***********************
%       
% INPUT DISTURBANCE SIGNAL d_i
%

Bdi = [ Bp*Ui
        zeros(ni,nu)
        -Bo*Dp*Ui       ];

% ***********************
%       
% OUTPUT DISTURBANCE SIGNAL d_o
%

Bdo = [ -Bp*Ui*Do
        zeros(ni,ny)
        -Bo*Uo          ];

% ***********************
%       
% INNER-LOOP NOISE n_i
%

Bni = [ -Bp*Ui*Di
        Bi
        Bo*Dp*Ui*Di     ];


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
Cxpup = -Ui*(Di*Mi+Do*Cp);
% x_i -> u_p
Cxiup = -Ui*Ci;
% x_o -> u_p
Cxoup = Ui*Co;

% x -> u_p
Cup = [ Cxpup   Cxiup   Cxoup   ];


% ***********************
%       
% PLANT OUTPUT y_p
%

% x_p -> y_p
Cxpyp = Cp+Dp*Cxpup;
% x_i -> y_p
Cxiyp = Dp*Cxiup;
% x_o -> y_p
Cxoyp = Dp*Cxoup;

% x -> y_p
Cyp = [ Cxpyp   Cxiyp   Cxoyp   ];


% ***********************
%       
% PLANT OUTPUT + DISTURBANCE y
%

% x_p -> y
Cxpy = Cxpyp;
% x_i -> y
Cxiy = Cxiyp;
% x_o -> y
Cxoy = Cxoyp;

% x -> y
Cy = [ Cxpy     Cxiy    Cxoy   ];


% ***********************
%       
% ERROR e
%

% x_p -> e
Cxpe = -Cxpy;
% x_i -> e
Cxie = -Cxiy;
% x_o -> e
Cxoe = -Cxoy;

% x -> e
Ce = [ Cxpe     Cxie    Cxoe   ];


% ***********************
%       
% OUTER-LOOP CONTROL SIGNAL u_o
%

% x_p -> u_o
Cxpuo = Do*Cxpe;
% x_i -> u_o
Cxiuo = Do*Cxie;
% x_o -> u_o
Cxouo = Co+Do*Cxoe;

% x -> u_o
Cuo = [ Cxpuo    Cxiuo   Cxouo   ];


% ***********************
%       
% CONTROL SIGNAL u
%

% x_p -> u
Cxpu = Cxpup;
% x_i -> u
Cxiu = Cxiup;
% x_o -> u
Cxou = Cxoup;

% x -> u
Cu = [ Cxpu     Cxiu    Cxou   ];

% ***********************
%       
% INNER-LOOP FEEDBACK STATE x_r
%

% x_p -> x_r
Cxpxr = Mi;
% x_i -> x_r
Cxixr = zeros(nxr,ni);
% x_o -> x_r
Cxoxr = zeros(nxr,no);

% x -> x_r
Cxr = [ Cxpxr   Cxixr   Cxoxr   ];

% ***********************
%       
% INPUT TO INNER-LOOP CONTROLLER e_i
%

% x_p -> e_i
Cxpei = Cxpxr;
% x_i -> e_i
Cxiei = Cxixr;
% x_o -> e_i
Cxoei = Cxoxr;

% x -> e_i
Cei = [ Cxpei Cxiei Cxoei   ];

% ***********************
%       
% INNER-LOOP CONTROL SIGNAL u_i
%

% x_p -> u_i
Cxpui = Di*Mi;
% x_i -> u_i
Cxiui = Ci;
% x_o -> u_i
Cxoui = zeros(nu,no);

% x -> u_o
Cui = [ Cxpui    Cxiui   Cxoui   ];



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
Drup = Ui*Do;
% d_i -> u_p
Ddiup = Ui;
% d_o -> u_p
Ddoup = -Ui*Do;
% n_i -> u_p
Dniup = -Ui*Di;


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
% n_i -> y_p
Dniyp = Dp*Dniup;


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
% n_i -> y
Dniy = Dniyp;


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
% n_i -> e
Dnie = -Dniy;


% ***********************
%       
% OUTER-LOOP CONTROL SIGNAL u_o
%

% r -> u_o
Druo = Do*Dre;
% d_i -> u_o
Ddiuo = Do*Ddie;
% d_o -> u_o
Ddouo = Do*Ddoe;
% n_i -> u_o
Dniuo = Do*Dnie;



% ***********************
%       
% CONTROL SIGNAL u
%

% r -> u
Dru = Drup;
% d_i -> u
Ddiu = -Ui*Do*Dp;
% d_o -> u
Ddou = Ddoup;
% n_i -> u
Dniu = Dniup;

% ***********************
%       
% INNER-LOOP FEEDBACK STATE x_r
%

% r -> x_r
Drxr = zeros(nxr,ny);
% d_i -> x_r
Ddixr = zeros(nxr,nu);
% d_o -> x_r
Ddoxr = zeros(nxr,ny);
% n_i -> x_r
Dnixr = zeros(nxr);

% ***********************
%       
% INPUT TO INNER-LOOP CONTROLLER e_i
%

% r -> e_i
Drei = Drxr;
% d_i -> e_i
Ddiei = Ddixr;
% d_o -> e_i
Ddoei = Ddoxr;
% n_i -> e_i
Dniei = eye(nxr);

% ***********************
%       
% INNER-LOOP CONTROL SIGNAL u_i
%

% r -> u_i
Drui = Di*Drei;
% d_i -> u_i
Ddiui = Di*Ddiei;
% d_o -> u_i
Ddoui = Di*Ddoei;
% n_i -> u_i
Dniui = Di*Dniei;

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

%   K_i
out_data.Ki = Ki;
%   K_o
out_data.Ko = Ko;

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
%   KoSe                    (ss Obj.) CL map r -> u_o
out_data.KoSe = ss(Acl,Br,Cuo,Druo);
%   Trxr                    (ss Obj.) CL map r -> x_r
out_data.Trxr = ss(Acl,Br,Cxr,Drxr);
%   Trui                    (ss Obj.) CL map r -> u_i
out_data.Trui = ss(Acl,Br,Cui,Drui);

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
%   KoPSu                   (ss Obj.) CL map d_i -> u_o
out_data.KoPSu = ss(Acl,Bdi,Cuo,Ddiuo);
%   Tdixr                   (ss Obj.) CL map d_i -> x_r
out_data.Tdixr = ss(Acl,Bdi,Cxr,Ddixr);
%   Tdiui                   (ss Obj.) CL map d_i -> u_i
out_data.Tdiui = ss(Acl,Bdi,Cui,Ddiui);

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
%   Tdouo                   (ss Obj.) CL map d_o -> u_o
out_data.Tdouo = ss(Acl,Bdo,Cuo,Ddouo);
%   Tdoxr                   (ss Obj.) CL map d_o -> x_r
out_data.Tdoxr = ss(Acl,Bdo,Cxr,Ddoxr);
%   Tdoui                   (ss Obj.) CL map d_o -> u_i
out_data.Tdoui = ss(Acl,Bdo,Cui,Ddoui);

% ***********************
%       
% INNER-LOOP NOISE n_i
%

% Only add these if the inner loop is present
if is_io
    
    %   Tniu                    (ss Obj.) CL map n_i -> u
    out_data.Tniu = ss(Acl,Bni,Cu,Dniu);
    %   Tniy                    (ss Obj.) CL map n_i -> y
    out_data.Tniy = ss(Acl,Bni,Cy,Dniy);
    %   Tnie                    (ss Obj.) CL map n_i -> e
    out_data.Tnie = ss(Acl,Bni,Ce,Dnie);
    %   Tniuo                   (ss Obj.) CL map n_i -> u_o
    out_data.Tniuo = ss(Acl,Bni,Cuo,Dniuo);
    %   Tnixr                   (ss Obj.) CL map n_i -> x_r
    out_data.Tnixr = ss(Acl,Bni,Cxr,Dnixr);
    %   Tniei                   (ss Obj.) CL map n_i -> e_i
    out_data.Tniei = ss(Acl,Bni,Cei,Dniei);
    %   Tniui                   (ss Obj.) CL map n_i -> u_i 
    out_data.Tniui = ss(Acl,Bni,Cui,Dniui);

end


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
    %   WKoSe                   (ss Obj.) CL map r -> u_o
    out_data.WKoSe = ss(Aclw,Brw,[Cuo Druo*Cw],Druo*Dw);
    %   WTrxr                   (ss Obj.) CL map r -> x_r
    out_data.WTrxr = ss(Aclw,Brw,[Cxr Drxr*Cw],Drxr*Dw);
    %   WTrui                   (ss Obj.) CL map r -> u_i
    out_data.WTrui = ss(Aclw,Brw,[Cui Drui*Cw],Drui*Dw);

end

% ***********************
%       
% CLOSED-LOOP "A" MATRIX
%

out_data.Acl = Acl;

