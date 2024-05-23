clc; clear all
syms theta4 theta5 theta6;
% Define link lengths
l1 = 0.09122;
l2 = 0.1637;
l3 = 0.19998;
l4 = 0.1493;
l5 = 0.1607;
l6 = 0.1468;
l7 = 0.2449;

% Define end effector position and orientation expressed with respect to
% the world coordinate system
x0 = 0;
y0 = 0;
z0 = 0.5;
alpha = pi/4;
beta = 0;
gamma = 0;

% Compute the desired transformation matrix T06
R = eul2rotm([gamma, beta, alpha], 'ZYZ');
T06 = [R [x0; y0; z0]; 0 0 0 1];

% Compute the wrist center position
d6 = l6+l7;
xW = x0 - d6 * T06(1, 3);
yW = y0 - d6 * T06(2, 3);
zW = z0 - d6 * T06(3, 3);

disp("the position of the wrist center is : ")
disp([xW; yW; zW]);

% Inverse Kinematics: Find theta1, theta2, and theta3
theta1 = atan2(yW, xW);

% Compute variables for theta2 and theta3
r = sqrt(xW^2 + yW^2);
s = zW - (l1+l2);
D=(s^2+r^2-l3^2-(l4+l5)^2)/(2*l3*(l4+l5));
sin3 = D;
cos3 = sqrt(1 - D^2);
theta3 = atan2(sin3, cos3);

% Find theta2 based on theta3
theta2 = atan2(r, s) - atan2(l3 + (l4+l5) * cos3, (l4+l5) * sin3);

d1=l1+l2; alpha1=pi/2; a1=0;
d2=0; alpha2=0; a2=l3;
d3=0; alpha3=0; a3=l4;


% Compute T03 using the DH parameters and the calculated angles
T1 = dh_transform(theta1, d1, a1, alpha1);
T2 = dh_transform(theta2, d2, a2, alpha2);
T3 = dh_transform(theta3, d3, a3, alpha3);


% Compute the T03 matrix
T03 = T1 * T2 * T3

% Calculate T36 from T06 and T03
T36 = inv(T03) * T06;

% Extract theta4, theta5, and theta6 from T36
theta5_1 = atan2(sqrt(T36(1, 3)^2 + T36(2, 3)^2), T36(3, 3));
theta4_1 = atan2(T36(2, 3), T36(1, 3));
theta6_1 = atan2(T36(3, 2), -T36(3, 1));

% Combine all joint angles for the first solution
S1 = [theta1, theta2, theta3, theta4_1, theta5_1, theta6_1];

a4=0; alpha4=-pi/2; d4=0;
a5=0; alpha5=pi/2; d5=0;
a6=0; alpha6=0; %d6=l6+l7

% sT4 = dh_transform(theta4, d4, a4, alpha4);
% sT5 = dh_transform(theta5, d5, a5, alpha5);
% sT6 = dh_transform(theta6, d6, a6, alpha6);
% sT36 = vpa(simplify(sT4*sT5*sT6),2);

% Define the forward kinematics for the first solution
T4 = dh_transform(S1(4), d4, a4, alpha4);
T5 = dh_transform(S1(5), d5, a5, alpha5);
T6 = dh_transform(S1(6), d6, a6, alpha6);

% Compute the full transformation matrix T06 using the forward kinematics
T06_check = T03 * T4 * T5 * T6;

% Print the difference between the original T06 and the one computed from the solution
disp('Difference between original T06 and T06 from forward kinematics:');
disp(T06 - T06_check);

function T = dh_transform(theta, d, a, alpha)
    % Compute the DH transformation matrix
    T = [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

% function R = eul2rotm(eul, sequence)
%     % Convert Euler angles to a rotation matrix
%     Rz1 = [cos(eul(1)), -sin(eul(1)), 0;
%            sin(eul(1)), cos(eul(1)), 0;
%            0, 0, 1];
%     Ry = [cos(eul(2)), 0, sin(eul(2));
%           0, 1, 0;
%           -sin(eul(2)), 0, cos(eul(2))];
%     Rz2 = [cos(eul(3)), -sin(eul(3)), 0;
%            sin(eul(3)), cos(eul(3)), 0;
%            0, 0, 1];
%     R = Rz1 * Ry * Rz2;
% end
