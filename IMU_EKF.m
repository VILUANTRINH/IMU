%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extended Kalman filter function
% Input u_k_1: prior control signal
% Input y_meas: measurement data (gyroscope, 
%               accelerometer, magnetometer)
% Input x_hat_plus_k_1: prior estimation
% Input P_plus_k_1: prior covariance of estimation
%                   error
% Output x_hat_plus_k: posteriori estimation
% Output P_plus_k: posteriori covariance of estimation
%                  error
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_hat_plus_k, P_plus_k] = IMU_EKF(u_k_1, y_meas, x_hat_plus_k_1, P_plus_k_1)
	% Symbol variables declaration
	syms x1 x2 x3 x4 x5 x6 x7 w_rx w_ry w_rz ;
	syms t_rx t_ry t_rz T t1 t2 t3 delta_t;
    syms  v1 v2 v3 v4 v5 v6 v7 u  ;  % w_rx w_ry w_rz
	% Sampling time
	T=0.001;
	
	% Input and output funtions
	f1 = T*((-1/0.5)*x1 + (1/0.5)*w_rx) + x1;   
	f2 = T*((-1/0.5)*x2 + (1/0.5)*w_ry) + x2;
	f3 = T*((-1/0.5)*x3 + (1/0.5)*w_rz) + x3;
	f4 = T*(1/(2*sqrt(x4^2 + x5^2 + x6^2 + x7^2)))*(x3*x5 - x2*x6 + x1*x7)  + x4;
	f5 = T*(1/(2*sqrt(x4^2 + x5^2 + x6^2 + x7^2)))*(-x3*x4 + x1*x6 + x2*x7) + x5;
	f6 = T*(1/(2*sqrt(x4^2 + x5^2 + x6^2 + x7^2)))*(x2*x4 - x1*x5 + x3*x7)  + x6;
	f7 = T*(1/(2*sqrt(x4^2 + x5^2 + x6^2 + x7^2)))*(-x1*x4 - x2*x5 - x3*x6) + x7;
	h1 = x1 + v1;
	h2 = x2 + v2;
	h3 = x3 + v3;
	h4 = x4 + v4;
	h5 = x5 + v5;
	h6 = x6 + v6;
	h7 = x7 + v7;

	F = [                                  499/500,                                            0,                                            0,                                                                                             0,                                                                                             0,                                                                                             0,                                                                                            0;
	                                             0,                                      499/500,                                            0,                                                                                             0,                                                                                             0,                                                                                             0,                                                                                            0;
	                                             0,                                            0,                                      499/500,                                                                                             0,                                                                                             0,                                                                                             0,                                                                                            0;
	   x7/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)), -x6/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)),  x5/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)),                     1 - (x4*(x1*x7 - x2*x6 + x3*x5))/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)),  (x3*x4^2 + x3*x6^2 + x2*x5*x6 + x3*x7^2 - x1*x5*x7)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)), -(x2*x4^2 + x2*x5^2 + x3*x6*x5 + x2*x7^2 + x1*x6*x7)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)), (x1*x4^2 + x1*x5^2 - x3*x7*x5 + x1*x6^2 + x2*x7*x6)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2));
	   x6/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)),  x7/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)), -x4/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)), -(x3*x5^2 + x3*x6^2 + x1*x4*x6 + x3*x7^2 + x2*x4*x7)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)),                     1 - (x5*(x1*x6 - x3*x4 + x2*x7))/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)),  (x1*x4^2 + x3*x6*x4 + x1*x5^2 + x1*x7^2 - x2*x6*x7)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)), (x2*x4^2 + x3*x7*x4 + x2*x5^2 + x2*x6^2 - x1*x7*x6)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2));
	  -x5/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)),  x4/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)),  x7/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)),  (x2*x5^2 + x1*x4*x5 + x2*x6^2 + x2*x7^2 - x3*x4*x7)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)), -(x1*x4^2 + x2*x5*x4 + x1*x6^2 + x1*x7^2 + x3*x5*x7)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)),                     1 - (x6*(x2*x4 - x1*x5 + x3*x7))/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)), (x3*x4^2 - x2*x7*x4 + x3*x5^2 + x1*x7*x5 + x3*x6^2)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2));
	  -x4/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)), -x5/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)), -x6/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(1/2)), -(x1*x5^2 - x2*x4*x5 + x1*x6^2 - x3*x4*x6 + x1*x7^2)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)), -(x2*x4^2 - x1*x5*x4 + x2*x6^2 - x3*x5*x6 + x2*x7^2)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)), -(x3*x4^2 - x1*x6*x4 + x3*x5^2 - x2*x6*x5 + x3*x7^2)/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)),                    (x7*(x1*x4 + x2*x5 + x3*x6))/(2000*(x4^2 + x5^2 + x6^2 + x7^2)^(3/2)) + 1];
	 
	 
	L = [ 1/500,     0,     0,   0,   0,   0,   0 ;
	          0, 1/500,     0,   0,   0,   0,   0 ; 
	          0,     0, 1/500,   0,   0,   0,   0 ;
	          0,     0,     0,   1,   0,   0,   0 ;
	          0,     0,     0,   0,   1,   0,   0 ;
	          0,     0,     0,   0,   0,   1,   0 ;
	          0,     0,     0,   0,   0,   0,   1  ] ;
	 
	H = [ 1, 0, 0, 0, 0, 0, 0;
	      0, 1, 0, 0, 0, 0, 0;
	      0, 0, 1, 0, 0, 0, 0;
	      0, 0, 0, 1, 0, 0, 0;
	      0, 0, 0, 0, 1, 0, 0;
	      0, 0, 0, 0, 0, 1, 0;
	      0, 0, 0, 0, 0, 0, 1];
	 
	 
	M = [ 1, 0, 0, 0, 0, 0, 0;
	      0, 1, 0, 0, 0, 0, 0;
	      0, 0, 1, 0, 0, 0, 0;
	      0, 0, 0, 1, 0, 0, 0;
	      0, 0, 0, 0, 1, 0, 0;
	      0, 0, 0, 0, 0, 1, 0;
	      0, 0, 0, 0, 0, 0, 1];
	
	q11 = 1;
	q22 = 1;
	q33 = 1;

	Q_k=[ q11  0  0  0  0  0  0   ;
	       0  q22 0  0  0  0  0   ;
	       0   0 q33 0  0  0  0   ;
	       0   0  0  0  0  0  0   ; 
	       0   0  0  0  0  0  0   ;
	       0   0  0  0  0  0  0   ;
	       0   0  0  0  0  0  0  ];
	   
	R_k= [ 0.01      0      0     0     0     0     0 ;
	       0       0.01     0     0     0     0     0 ;
	       0         0    0.0001  0     0     0     0 ;
	       0         0      0   0.0001  0     0     0 ;
	       0         0      0     0  0.0001   0     0 ;
	       0         0      0     0     0   0.0001  0 ;
	       0         0      0     0     0     0   0.0001 ];
	
	I= [ 1 0 0 0 0 0 0 ;
	     0 1 0 0 0 0 0 ;
	     0 0 1 0 0 0 0 ; 
	     0 0 0 1 0 0 0 ;
	     0 0 0 0 1 0 0 ;
	     0 0 0 0 0 1 0 ;
	     0 0 0 0 0 0 1 ];      

	% (a) Compute the following partial derivative matrices:
	F = subs(F, [x1 x2 x3 x4 x5 x6 x7 u], [x_hat_plus_k_1(1) x_hat_plus_k_1(2) x_hat_plus_k_1(3) x_hat_plus_k_1(4) x_hat_plus_k_1(5) x_hat_plus_k_1(6) x_hat_plus_k_1(7) u_k_1]);
    F = double(F);
    
	% (b) Perform the time update of the state estimate and estimation-error covariance as follows: 
	P_minus_k = F*P_plus_k_1*F' + L*Q_k*L';
	
	x_hat_minus_k = zeros(7,1);
	x_hat_minus_k(1) = subs(f1, [x1 x2 x3 x4 x5 x6 x7 v1 w_rx], [x_hat_plus_k_1(1) x_hat_plus_k_1(2) x_hat_plus_k_1(3) x_hat_plus_k_1(4) x_hat_plus_k_1(5) x_hat_plus_k_1(6) x_hat_plus_k_1(7) 0 0]);
	x_hat_minus_k(2) = subs(f2, [x1 x2 x3 x4 x5 x6 x7 v2 w_ry], [x_hat_plus_k_1(1) x_hat_plus_k_1(2) x_hat_plus_k_1(3) x_hat_plus_k_1(4) x_hat_plus_k_1(5) x_hat_plus_k_1(6) x_hat_plus_k_1(7) 0 0]);
	x_hat_minus_k(3) = subs(f3, [x1 x2 x3 x4 x5 x6 x7 v3 w_rz], [x_hat_plus_k_1(1) x_hat_plus_k_1(2) x_hat_plus_k_1(3) x_hat_plus_k_1(4) x_hat_plus_k_1(5) x_hat_plus_k_1(6) x_hat_plus_k_1(7) 0 0]);
	x_hat_minus_k(4) = subs(f4, [x1 x2 x3 x4 x5 x6 x7 v4], [x_hat_plus_k_1(1) x_hat_plus_k_1(2) x_hat_plus_k_1(3) x_hat_plus_k_1(4) x_hat_plus_k_1(5) x_hat_plus_k_1(6) x_hat_plus_k_1(7) 0]);
	x_hat_minus_k(5) = subs(f5, [x1 x2 x3 x4 x5 x6 x7 v5], [x_hat_plus_k_1(1) x_hat_plus_k_1(2) x_hat_plus_k_1(3) x_hat_plus_k_1(4) x_hat_plus_k_1(5) x_hat_plus_k_1(6) x_hat_plus_k_1(7) 0]);
	x_hat_minus_k(6) = subs(f6, [x1 x2 x3 x4 x5 x6 x7 v6], [x_hat_plus_k_1(1) x_hat_plus_k_1(2) x_hat_plus_k_1(3) x_hat_plus_k_1(4) x_hat_plus_k_1(5) x_hat_plus_k_1(6) x_hat_plus_k_1(7) 0]);
	x_hat_minus_k(7) = subs(f7, [x1 x2 x3 x4 x5 x6 x7 v7], [x_hat_plus_k_1(1) x_hat_plus_k_1(2) x_hat_plus_k_1(3) x_hat_plus_k_1(4) x_hat_plus_k_1(5) x_hat_plus_k_1(6) x_hat_plus_k_1(7) 0]);
	
	% (c) Compute the following partial derivative matrices:

	% (d) Perform the measurement update of the state estimate and estimation error covariance as follows:   
	K = P_minus_k * H.' * inv(H*P_minus_k*H.' + M*R_k*M.');
	h_k = zeros(7,1);
	h_k(1) = subs(h1, [x1 x2 x3 x4 x5 x6 x7 v1], [x_hat_minus_k(1) x_hat_minus_k(2) x_hat_minus_k(3) x_hat_minus_k(4) x_hat_minus_k(5) x_hat_minus_k(6) x_hat_minus_k(7) 0]);
	h_k(2) = subs(h2, [x1 x2 x3 x4 x5 x6 x7 v2], [x_hat_minus_k(1) x_hat_minus_k(2) x_hat_minus_k(3) x_hat_minus_k(4) x_hat_minus_k(5) x_hat_minus_k(6) x_hat_minus_k(7) 0]);
	h_k(3) = subs(h3, [x1 x2 x3 x4 x5 x6 x7 v3], [x_hat_minus_k(1) x_hat_minus_k(2) x_hat_minus_k(3) x_hat_minus_k(4) x_hat_minus_k(5) x_hat_minus_k(6) x_hat_minus_k(7) 0]);
	h_k(4) = subs(h4, [x1 x2 x3 x4 x5 x6 x7 v4], [x_hat_minus_k(1) x_hat_minus_k(2) x_hat_minus_k(3) x_hat_minus_k(4) x_hat_minus_k(5) x_hat_minus_k(6) x_hat_minus_k(7) 0]);
	h_k(5) = subs(h5, [x1 x2 x3 x4 x5 x6 x7 v5], [x_hat_minus_k(1) x_hat_minus_k(2) x_hat_minus_k(3) x_hat_minus_k(4) x_hat_minus_k(5) x_hat_minus_k(6) x_hat_minus_k(7) 0]);
	h_k(6) = subs(h6, [x1 x2 x3 x4 x5 x6 x7 v6], [x_hat_minus_k(1) x_hat_minus_k(2) x_hat_minus_k(3) x_hat_minus_k(4) x_hat_minus_k(5) x_hat_minus_k(6) x_hat_minus_k(7) 0]);
	h_k(7) = subs(h7, [x1 x2 x3 x4 x5 x6 x7 v7], [x_hat_minus_k(1) x_hat_minus_k(2) x_hat_minus_k(3) x_hat_minus_k(4) x_hat_minus_k(5) x_hat_minus_k(6) x_hat_minus_k(7) 0]);

	P_plus_k= (I - K*H)*(P_minus_k);
	x_hat_plus_k = x_hat_minus_k + K*(y_meas - h_k);
    x_hat_plus_k = double(x_hat_plus_k);
   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



