%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Applied extended Kalman filter and Newton optimization
% algorithms in inertial measurement unit (IMU) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Algorithms' parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_hat_plus_k =   [0                       % gyrox
                  0                       % gyroy
                  0                       % gyroz
                  0                       % qa
                  0                       % qb
                  0                       % qc
                  1];                     % qd
P_plus_k = [100  0   0  0   0   0   0 ;
             0  100  0  0   0   0   0 ;
             0   0  100 0   0   0   0 ;
             0   0   0 100  0   0   0 ; 
             0   0   0  0  100  0   0 ;
             0   0   0  0   0  100  0 ;
             0   0   0  0   0   0  100];
epsilon = 0.001;
max_iteration = 20;

max_number_of_sample = 10000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Read_data_from_Arduino
arduino = serial('COM6', 'BaudRate', 9600);   
set(arduino,'DataBits', 8);
set(arduino,'StopBits', 1);
fopen(arduino);
s.ReadAsyncMode = 'continuous';
readasync(arduino);

k=1;

while (k<=max_number_of_sample)
    % meas
    frame_mes=fscanf(arduino,'%f');

    Gx(k)=frame_mes(1);
    Gy(k)=frame_mes(2);
    Gz(k)=frame_mes(3);
    Ax(k)=frame_mes(4);
    Ay(k)=frame_mes(5);
    Az(k)=frame_mes(6);
    Mx(k)=frame_mes(7);
    My(k)=frame_mes(8);
    Mz(k)=frame_mes(9);
    k=k+1 ; 
    disp(k) 
  
end

for k=1:max_number_of_sample
    % Newton
    quaternion = IMU_Newton(Ax(k), Ay(k), Az(k), Mx(k), My(k), Mz(k), epsilon, max_iteration);
    quaternion = double(quaternion) ;
    y_meas = [Gx(k); Gy(k); Gz(k); quaternion(1); quaternion(2); quaternion(3); quaternion(4)];
    x(k)=quaternion(1);
    y(k)=quaternion(2);
    z(k)=quaternion(3);
    w(k)=quaternion(4);

    % Kalman
    x_hat_plus_k_1 = x_hat_plus_k;
    P_plus_k_1 = P_plus_k;
    [x_hat_plus_k, P_plus_k] = IMU_EKF(0, y_meas, x_hat_plus_k_1, P_plus_k_1);
   
    Gx_estimated(k)     = x_hat_plus_k(1);
    Gy_estimated(k)     = x_hat_plus_k(2);
    Gz_estimated(k)     = x_hat_plus_k(3);
    Quat_a_estimated(k) = x_hat_plus_k(4); 
    Quat_b_estimated(k) = x_hat_plus_k(5);
    Quat_c_estimated(k) = x_hat_plus_k(6);
    Quat_d_estimated(k) = x_hat_plus_k(7);
    
    disp( k ) 
end

%% plot data %%
figure(1);
plot(Gx_estimated);
title('rad/s');
ylabel('phat');
hold on;

figure(2);
plot(Gy_estimated);
title('rad/s');
ylabel('qhat');
hold on;

figure(3);
plot(Gz_estimated);
title('rad/s');
ylabel('rhat');
hold on;

figure(4);
plot(Quat_a_estimated);
ylabel('Quata');
hold on;

figure(5);
plot(Quat_b_estimated);
ylabel('Quatb');
hold on;

figure(6);
plot(Quat_c_estimated);
ylabel('Quatc');
hold on;

figure(7);
plot(Quat_d_estimated);
ylabel('Quatd');
hold on;

  
  
 