	function H = DCM(Phi,Theta,Psi)
%	FLIGHT Earth-to-Body-Axis Direction-Cosine Matrix

%	November 11, 2018   
%	===============================================================
%	Copyright 2006-18 by ROBERT F. STENGEL.  All rights reserved.

%   Called by:
%       FLIGHT.m
%       EoM.m in FLIGHT.m

%	Euler Angles:
%		Phi,	Roll Angle, rad
%		Theta,	Pitch Angle, rad
%		Psi,	Yaw Angle, rad

	sinR = sin(Phi);
	cosR = cos(Phi);
	sinP = sin(Theta);
	cosP = cos(Theta);
	sinY = sin(Psi);
	cosY = cos(Psi);

	H(1,1) = cosP * cosY;
	H(1,2) = cosP * sinY;
	H(1,3) = -sinP;
	H(2,1) = sinR * sinP * cosY - cosR * sinY;
	H(2,2) = sinR * sinP * sinY + cosR * cosY;
	H(2,3) = sinR * cosP;
	H(3,1) = cosR * sinP * cosY + sinR * sinY;
	H(3,2) = cosR * sinP * sinY - sinR * cosY;
	H(3,3) = cosR * cosP;