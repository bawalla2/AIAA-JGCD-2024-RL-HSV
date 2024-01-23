function nt = eval_noise(t, m, noise)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE PROBING NOISE n(t)
%
% Brent Wallace  
%
% 2021-11-06
%
% This program, given a tag of a desired noise signal, input dimension m,
% and time t evaluates the probing noise n(t).
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% nt = eval_noise(t, m, noise)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% t         (Double) Time (sec)
% m         (Integer) System input dimension
% noise     (Struct) Contains properties of the pecific noise signal to be
%           evaluated. Has the following fields:
%   tag     (String) Tag of the noise signal to be evaluated
%
% NOTE: If the noise is of tag 'sum_sinusoids', additional fields are
% required (see description above the respective tag).
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% nt        (m-dim. vector) Evaluation of probing noise n(t)
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

switch noise.tag

    % ***********************
    %
    % ZERO NOISE SIGNAL
    %
    
    case '0'
        
        nt = zeros(m,1);

    % ***********************
    %
    % MULTIVARIABLE SUM OF SINUSOIDAL SIGNALS
    %
    
    case 'multivar_sum_sinusoid'   

        % Get settings
        cos1_sin0 = noise.cos1_sin0;
        Amat = noise.Amat;
        Tmat = noise.Tmat;       
        donoisevec = noise.donoisevec;
        nsin = size(Amat,2);

        % Initialize
        nt = zeros(m,1);
        
        for i = 1:m
            
            if donoisevec(i)
                for j = 1:nsin
                    if cos1_sin0(i,j)
                        tmp = cos(2*pi/Tmat(i,j) * t);
                    else
                        tmp = sin(2*pi/Tmat(i,j) * t);
                    end
    
                    nt(i) = nt(i) + Amat(i,j) * tmp;
                end
            end

        end

        
    % ***********************
    %
    % MULTIVARIABLE SINUSOIDAL SIGNAL
    %
    
    case 'multivar_sinusoid'   

        % Get settings
        cos1_sin0 = noise.cos1_sin0;
        bvec = noise.bvec;
        Avec = noise.Avec;
        Tvec = noise.Tvec;

        % Initialize
        nt = zeros(m,1);
        
        for i = 1:m
            
            if cos1_sin0(i)
                tmp = cos(2*pi/Tvec(i) * t);
            else
                tmp = sin(2*pi/Tvec(i) * t);
            end

            nt(i) = bvec(i) + Avec(i) * tmp;

        end


    % ***********************
    %
    % SUM OF SINES AND COSINES
    %
    % Requires the fields in the noise struct:
    %
    %   cos1_sin0       M-dim. binary vector, where M denotes the total
    %                   number of sinusoids in the noise signal. For the
    %                   ith sinusoid (1 <= i <= M) 1 = make cos(), 0 = make
    %                   sin().
    %   scalevec        M-dim. vector. For ith sinusoid, is the scalar
    %                   multiple to put in front of the ith sinusoid
    %   freqvec         M-dim. vector. For ith sinusoid, is the frequency
    %                   of the ith sinusoid
    %   
    % E.g., to make the noise n(t) = cos(t) - 2*sin(5t), we would have
    % cos1_sin0 = [1;0], scalevec = [1;-2], freqvec = [1;5].
    %   
    %   
    
    case 'sum_sinusoids'
        
        M = size(noise.cos1_sin0, 1);   % Number of sinusoids in the signal
        nt = zeros(m,1);
        
        for i = 1:M
            a = noise.scalevec(i);
            w = noise.freqvec(i);
            if noise.cos1_sin0(i)
                nt = nt + a * ones(m,1) * cos(w * t);
            else    
                nt = nt + a * ones(m,1) * sin(w * t);
            end 
        end

    % ***********************
    %
    % DECAYING SINUSOIDAL SIGNAL
    %
    
    case 'dacaying_sinusoid'   
        
        a = 5;              % Amplitude
        w = 1;              % Frequency
        alpha = 0.01;       % Decay rate
        
        nt = ones(m, 1) * a * cos(w * t) * exp(-alpha * t);
        


    % *********************************************************************
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    % *********************************************************************
    % *********************************************************************
    
    otherwise
        
        error('*** ERROR: NOISE TAG NOT RECOGNIZED ***');

end