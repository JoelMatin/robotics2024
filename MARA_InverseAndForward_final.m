%2023-2024
%Group 5 : David Moli, Joël Pacha Matin, Loubna El Baz, Omidreza Kamalian, Saad Rekiek

clc; clear 
% link lengths
l1 = 0.09122;
l2 = 0.1637;
l3 = 0.19998;
l4 = 0.1493;
l5 = 0.1607;
l6 = 0.1468;
l7 = 0.2449;

% End effector position and orientation set by user
x0 = 0;
y0 = 0;
z0 = 0.8;
phi = pi/6;
theta = 0;
psi = 0;

% DH parameters
d1=l1+l2; alpha1=pi/2; a1=0;
d2=0; alpha2=0; a2=l3;
d3=0; alpha3=0; a3=l4+l5;
d4=0; alpha4=-pi/2; a4=0;
d5=0; alpha5=pi/2; a5=0;
d6=l6+l7; alpha6=0; a6=0;

R = eul2rotm([phi theta psi], 'ZYX'); % get rotation matrix from roll-pitch-yaw angles
T06 = [R [x0; y0; z0]; 0 0 0 1];

% compute wrist center position
xW = x0 - d6 * T06(1, 3);
yW = y0 - d6 * T06(2, 3);
zW = z0 - d6 * T06(3, 3);

disp("the position of the wrist center is : ")
disp([xW; yW; zW]);

% Inverse Kinematics: 

%  compute theta1, theta3 and theta2
theta1_1 = atan2(yW, xW); theta1_2 = theta1_1 + pi;

r = sqrt(xW^2 + yW^2);    
s = zW - d1;
D=(s^2+r^2-a2^2-a3^2)/(2*a2*a3);
cos3 = D;
sin3_1 = sqrt(1 - D^2);    sin3_2 = -sin3_1;
theta3_1 = atan2(sin3_1, cos3);   theta3_2 = atan2(sin3_2, cos3);

theta2_1 = atan2(s, r) - atan2(a3 * sin3_1 , a2 + a3 * cos3);
theta2_2 = atan2(s, r) - atan2(a3 * sin3_2 , a2 + a3 * cos3);

% compute the T03 matrix, and then T36 matrix
T1 = dh_mat(theta1_1, d1, a1, alpha1);
T2 = dh_mat(theta2_1, d2, a2, alpha2);
T3 = dh_mat(theta3_1, d3, a3, alpha3);
T03 = T1 * T2 * T3;
T36 = transpose(T03) * T06;

disp('wrist position found with T03 :');
disp([T03(1,4), T03(2,4), T03(3,4)]);

% compute theta4, theta5, and theta6 from T36
theta5_1 = atan2(sqrt(T36(1, 3)^2 + T36(2, 3)^2), T36(3, 3));
theta5_2 = atan2(-sqrt(T36(1, 3)^2 + T36(2, 3)^2), T36(3, 3));
theta4 = atan2(T36(2, 3), T36(1, 3));
theta6 = atan2(T36(3, 2), -T36(3, 1));

% solutions all joint angles
S1 = [theta1_1, theta2_1, theta3_1, theta4, theta5_1, theta6];
S2 = [theta1_1, theta2_2, theta3_2, theta4, theta5_1, theta6];
S3 = [theta1_2, theta2_1, theta3_1, theta4, theta5_1, theta6];
S4 = [theta1_2, theta2_2, theta3_2, theta4, theta5_1, theta6];
S5 = [theta1_1, theta2_1, theta3_1, theta4, theta5_2, theta6];
S6 = [theta1_1, theta2_2, theta3_2, theta4, theta5_2, theta6];
S7 = [theta1_2, theta2_1, theta3_1, theta4, theta5_2, theta6];
S8 = [theta1_2, theta2_2, theta3_2, theta4, theta5_2, theta6];

% define the forward kinematics
T1 = dh_mat(S1(1), d1, a1, alpha1);
T2 = dh_mat(S1(2), d2, a2, alpha2); %!!! if you want to change the solution, you need to make the appropriate changes in lines 58-60
T3 = dh_mat(S1(3), d3, a3, alpha3);
T4 = dh_mat(S1(4), d4, a4, alpha4);
T5 = dh_mat(S1(5), d5, a5, alpha5);
T6 = dh_mat(S1(6), d6, a6, alpha6);

% compute the full transformation matrix and check
T06_sol = T1 * T2 * T3 * T4 * T5 * T6;

disp('Difference between original T06 and T06 from forward kinematics:');
disp(T06 - T06_sol);
disp('Position set by user : ');
disp([x0, y0, z0, phi, theta, psi]);
disp('Position found with inverse kinematics :');
disp(get_position(T06_sol));

function T = dh_mat(theta, d, a, alpha)
    % compute the DH transformation matrix given the dh parameetrs
    T = [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

function pos = get_position(T)
    % get position and orientation of the end effector based on T06 matrix
    x0 = T(1,4);
    y0 = T(2,4);
    z0 = T(3,4);
    theta = atan2(-T(3,2), sqrt(T(1,1)^2+T(2,1)^2)); %pitch y
    phi = atan2(T(2,1),T(1,1)); %roll z
    psi = atan2(T(3,2),T(3,3)); %yaw x
    pos = [x0, y0, z0, phi, theta, psi];
end