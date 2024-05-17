clc, clear all
syms pi_sym;
pi_sym = sym(pi);
l1 = 0.09122;
l2 = 0.1637;
l3 = 0.19998;
l4 = 0.1493;
l5 = 0.1607;
l6 = 0.1468;
l7 = 0.2449;
x0 = 0;
y0 = 0;
z0 = 0.5;
alpha = pi/2;
beta = 0;
gamma = 0;
%syms x0 y0 z0 alpha beta gamma
%syms theta1 theta2 theta3 theta4 theta5 theta6
% syms c1 s1 c2 s2 c3 s3 s4 c4 c5 s5 c6 s6
syms theta4 theta5 theta6

% The T matrix can be obtained given the position and the orientation of
% the end effector
T06= [cos(alpha)*cos(gamma)-sin(alpha)*cos(beta)*sin(gamma) -cos(alpha)*sin(gamma)-sin(alpha)*cos(beta)*cos(gamma) sin(alpha)*sin(beta) x0;
    sin(alpha)*cos(gamma)-cos(alpha)*cos(beta)*sin(gamma) -sin(alpha)*sin(gamma)-cos(alpha)*cos(beta)*cos(gamma) -cos(alpha)*sin(beta) y0;
    sin(beta)*sin(gamma) sin(beta)*cos(gamma) cos(beta) z0;
    0 0 0 1]
% chapter 3:inverse kinematics slides 9 and 10 (decoupling)
% the center of the wrist W is at the joint 5
xW = x0 - (l6+l7)*T06(1,3);
yW = y0 - (l6+l7)*T06(2,3); % pW = p0 - d6.z6  z6=axis 6 of the last frame R6
zW = z0 - (l6+l7)*T06(3,3);

%first part : find theta1, theta2 and theta3 given the position of W
theta1 = atan2(yW, xW)-pi_sym/2;

% to find theta2 and theta 3 -> chapter 3 :inverse kinematics, slides 4 &5
x = sqrt(xW^2+yW^2); 
y=zW-(l1+l2);
cos3 = (x^2+y^2-l3^2-(l4+l5)^2)/(2*l3*(l4+l5));
sin3_1 = sqrt(1-cos3^2); 
sin3_2 = -sin3_1;
%We are going to have two solutions
theta3_1 = atan2(sin3_1, cos3)-pi_sym/2;
theta3_2 = atan2(sin3_2, cos3)-pi_sym/2;

cos2_1 = 1/(l3^2+(l4+l5)^2+2*l3*(l4+l5)*cos3) * ( (l3+(l4+l5)*cos3)*x  + (l4+l5)*sin3_1*y );
sin2_1 = 1/(l3^2+(l4+l5)^2+2*l3*(l4+l5)*cos3) * ( -(l4+l5)*sin3_1*x + (l3+(l4+l5)*cos3)*y );
cos2_2 = 1/(l3^2+(l4+l5)^2+2*l3*(l4+l5)*cos3) * ( (l3+(l4+l5)*cos3)*x  - (l4+l5)*sin3_2*y );
sin2_2 = 1/(l3^2+(l4+l5)^2+2*l3*(l4+l5)*cos3) * ( (l4+l5)*sin3_2*x + (l3+(l4+l5)*cos3)*y );
theta2_1 = atan2(sin2_1 ,cos2_1)-pi_sym/2 ;
theta2_2 = atan2(sin2_2 ,cos2_2)-pi_sym/2;



c1 = cos(theta1+pi_sym/2); a1=0; d1=0.1637;
s1 = sin(theta1+pi_sym/2); a2=0.19998; d2=0;
c4 = cos(theta4); a3=0; d3=0.1493;
s4 = sin(theta4); a4=0; d4=0.31;
c5 = cos(theta5); a5=0; d5=0;
s5 = sin(theta5); a6=0; d6=3917;
c6 = cos(theta6); 
s6 = sin(theta6);


T1 = [c1 -s1*cos(pi_sym/2) s1*sin(pi_sym/2) a1*c1; s1 c1*cos(pi_sym/2) -c1*sin(pi_sym/2) a1*s1; 0 sin(pi_sym/2) cos(pi_sym/2) d1; 0 0 0 1];
T2_1 = [cos2_1 -sin2_1*cos(0) sin2_1*sin(0) a2*cos2_1; sin2_1 cos2_1*cos(0) -cos2_1*sin(0) a2*sin2_1; 0 sin(0) cos(0) d2; 0 0 0 1];
T3_1 = [cos3 -sin3_1*cos(pi_sym/2) sin3_1*sin(pi_sym/2) a3*cos3; sin3_1 cos3*cos(pi_sym/2) -cos3*sin(pi_sym/2) a3*sin3_1; 0 sin(pi_sym/2) cos(pi_sym/2) d3; 0 0 0 1];
T2_2 = [cos2_2 -sin2_2*cos(0) sin2_2*sin(0) a2*cos2_2; sin2_2 cos2_2*cos(pi_sym/2) -cos2_2*sin(pi_sym/2) a2*sin2_2; 0 sin(pi_sym/2) cos(pi_sym/2) d2; 0 0 0 1];
T3_2 = [cos3 -sin3_2*cos(pi_sym/2) sin3_2*sin(pi_sym/2) a3*cos3; sin3_2 cos3*cos(0) -cos3*sin(0) a3*sin3_2; 0 sin(0) cos(0) d3; 0 0 0 1];
T4 = [c4 -s4*cos(-pi_sym/2) s4*sin(-pi/2) a4*c4; s4 c4*cos(-pi_sym/2) -c4*sin(-pi/2) a4*s4; 0 sin(-pi/2) cos(-pi_sym/2) d4; 0 0 0 1];
T5 = [c5 -s5*cos(pi_sym/2) s5*sin(pi/2) a5*c5; s5 c5*cos(pi_sym/2) -c5*sin(pi/2) a5*s5; 0 sin(pi/2) cos(pi_sym/2) d5; 0 0 0 1];
T6 = [c6 -s6*cos(0) s6*sin(0) a6*c6; s6 c6*cos(0) -c6*sin(0) a6*s6; 0 sin(0) cos(0) d6; 0 0 0 1];


%The matrix T03 is found (normally we have possible matrices)
T03_1 = T3_1*T2_1*T1;
T03_2 = T3_2*T2_2*T1;

T36 = T6*T5*T4 %if you uncomment this line, the expression of T36(theta4,
%theta5, theta6) will be shown, once we find the matrix T36, the expression
%will help us find the angles theta4 theta5 and theta6

T36_1 = transpose(T03_1)*T06;
T36_2 = transpose(T03_2)*T06;

cos5_1 = T36_1(3,3);
sin5_1 = sqrt(T36_1(3,1)^2+T36_1(3,2)^2); sin5_2 = -sqrt(T36_1(3,1)^2+T36_1(3,2)^2); 
theta5_1 = atan2(sin5_1, cos5_1); theta5_2 = atan2(sin5_2, cos5_1);
cos5_2 = T36_2(3,3);
sin5_3 = sqrt(T36_2(3,1)^2+T36_2(3,1)^2); sin5_4 = -sqrt(T36_2(3,1)^2+T36_2(3,2)^2); 
theta5_3 = atan2(sin5_3, cos5_2); theta5_4 = atan2(sin5_4, cos5_2);

cos4_1 = T36_1(1,3)/sin5_1; cos4_2 = T36_1(1,3)/sin5_2;
sin4_1 = T36_1(2,3)/sin5_1; sin4_2 = T36_1(2,3)/sin5_2;
theta4_1 = atan2(sin4_1, cos4_1); theta4_2 = atan2(sin4_2, cos4_2);
cos4_3 = T36_2(1,3)/sin5_3; cos4_4 = T36_2(1,3)/sin5_4;
sin4_3 = T36_2(2,3)/sin5_3; sin4_4 = T36_2(2,3)/sin5_4;
theta4_3 = atan2(sin4_3, cos4_3); theta4_4 = atan2(sin4_4, cos4_4);

cos6_1 = -T36_1(3,1)/sin5_1; cos6_2 = -T36_1(3,1)/sin5_2; 
sin6_1 = T36_1(3,2)/sin5_1; sin6_2 = T36_1(3,2)/sin5_2; 
theta6_1 = atan2(sin6_1, cos6_1); theta6_2 = atan2(sin6_2, cos6_2);
cos6_3 = -T36_2(3,1)/sin5_3; cos6_4 = -T36_2(3,1)/sin5_4; 
sin6_3 = T36_2(3,2)/sin5_3; sin6_4 = T36_2(3,2)/sin5_4; 
theta6_3 = atan2(sin6_3, cos6_3); theta6_4 = atan2(sin6_4, cos6_4);

%the four different set of possible solutions 
S1 = [theta1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1];
S2 = [theta1 theta2_1 theta3_1 theta4_2 theta5_2 theta6_2];
S3 = [theta1 theta2_2 theta3_2 theta4_3 theta5_3 theta6_3];
S4 = [theta1 theta2_2 theta3_2 theta4_4 theta5_4 theta6_4];

T1 = [c1 -s1*cos(0) s1*sin(0) a1*c1; s1 c1*cos(0) -c1*sin(0) a1*s1; 0 sin(0) cos(0) d1; 0 0 0 1];
T2_1 = [cos2_1 -sin2_1*cos(-pi/2) sin2_1*sin(-pi/2) a2*cos2_1; sin2_1 cos2_1*cos(-pi/2) -cos2_1*sin(-pi/2) a2*sin2_1; 0 sin(-pi/2) cos(-pi/2) d2; 0 0 0 1];
T3_1 = [cos3 -sin3_1*cos(0) sin3_1*sin(0) a3*cos3; sin3_1 cos3*cos(0) -cos3*sin(0) a3*sin3_1; 0 sin(0) cos(0) d3; 0 0 0 1];
T2_2 = [cos2_2 -sin2_2*cos(-pi/2) sin2_2*sin(-pi/2) a2*cos2_2; sin2_2 cos2_2*cos(-pi/2) -cos2_2*sin(-pi/2) a2*sin2_2; 0 sin(-pi/2) cos(-pi/2) d2; 0 0 0 1];
T3_2 = [cos3 -sin3_2*cos(0) sin3_2*sin(0) a3*cos3; sin3_2 cos3*cos(0) -cos3*sin(0) a3*sin3_2; 0 sin(0) cos(0) d3; 0 0 0 1];
T4 = [c4 -s4*cos(pi_sym/2) s4*sin(pi/2) a4*c4; s4 c4*cos(pi_sym/2) -c4*sin(pi/2) a4*s4; 0 sin(pi/2) cos(pi_sym/2) d4; 0 0 0 1];
T5 = [c5 -s5*cos(-pi_sym/2) s5*sin(-pi/2) a5*c5; s5 c5*cos(-pi_sym/2) -c5*sin(-pi/2) a5*s5; 0 sin(-pi/2) cos(-pi_sym/2) d5; 0 0 0 1];
T6 = [c6 -s6*cos(pi_sym/2) s6*sin(pi/2) a6*c6; s6 c6*cos(pi_sym/2) -c6*sin(pi/2) a6*s6; 0 sin(pi/2) cos(pi_sym/2) d6; 0 0 0 1];

ST01 = [cos(S1(1)) -sin(S1(1))*cos(0) sin(S1(1))*sin(0) a1*cos(S1(1)) ; sin(S1(1)) cos(S1(1))*cos(0) -cos(S1(1))*sin(0) a1*sin(S1(1)); 0 sin(0) cos(0) d1; 0 0 0 1];
ST12 = [cos(S1(2)) -sin(S1(2))*cos(-pi_sym/2) sin(S1(2))*sin(-pi_sym/2) a2*cos(S1(2)) ; sin(S1(2)) cos(S1(2))*cos(-pi_sym/2) -cos(S1(2))*sin(-pi_sym/2) a2*sin(S1(2)); 0 sin(-pi_sym/2) cos(-pi_sym/2) d2; 0 0 0 1];
ST23 = [cos(S1(3)) -sin(S1(3))*cos(0) sin(S1(3))*sin(0) a3*cos(S1(3)) ; sin(S1(3)) cos(S1(3))*cos(0) -cos(S1(3))*sin(0) a3*sin(S1(3)); 0 sin(0) cos(0) d3; 0 0 0 1];
ST34 = [cos(S1(4)) -sin(S1(4))*cos(pi_sym/2) sin(S1(2))*sin(pi_sym/2) a4*cos(S1(4)) ; sin(S1(4)) cos(S1(4))*cos(pi_sym/2) -cos(S1(4))*sin(pi_sym/2) a4*sin(S1(4)); 0 sin(pi_sym/2) cos(pi_sym/2) d4; 0 0 0 1];
ST45 = [cos(S1(5)) -sin(S1(5))*cos(-pi_sym/2) sin(S1(5))*sin(-pi_sym/2) a5*cos(S1(5)) ; sin(S1(5)) cos(S1(5))*cos(-pi_sym/2) -cos(S1(5))*sin(-pi_sym/2) a5*sin(S1(5)); 0 sin(-pi_sym/2) cos(-pi_sym/2) d5; 0 0 0 1];
ST56 = [cos(S1(6)) -sin(S1(6))*cos(pi_sym/2) sin(S1(6))*sin(pi_sym/2) a6*cos(S1(6)) ; sin(S1(6)) cos(S1(6))*cos(pi_sym/2) -cos(S1(6))*sin(pi_sym/2) a6*sin(S1(6)); 0 sin(pi_sym/2) cos(pi_sym/2) d6; 0 0 0 1];

ST = vpa(simplify(ST56*ST45*ST34*ST23*ST12*ST01),2)


%rounded_up_ratio = ceil(numeric_ratio);
