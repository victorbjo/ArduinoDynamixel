%%  symbols
syms theta real
syms theta1 real
syms theta2 real
syms theta3 real

syms dtheta1 real %angular position
syms dtheta2 real
syms dtheta3 real

syms dtheta1 real %angular velocity
syms dtheta2 real
syms dtheta3 real

syms ddtheta1 real %angular acceleration
syms ddtheta2 real
syms ddtheta3 real
syms G real %gravity

%% rotation matrices
RB1 = [cos(theta1) -sin(theta1) 0
       sin(theta1) cos(theta1) 0
       0 0 1];

R12 = [cos(theta2) -sin(theta2) 0
       0 0 1
       -sin(theta2) -cos(theta2) 0];

R23 = [cos(theta3) -sin(theta3) 0
       sin(theta3) cos(theta3) 0
       0 0 1];
   



% manipulator properties
gSym = [0;0;G]; %gravity vector symbols
z = [0;0;1]; %z axis unit vector

%masse bodies
m1 = 0.22955;
m2 = 0.20776;
m3 = 0.23267;

%inertia tensors
J1 = [131530.27 3088.91 1397.85
      3088.91 95266.29 180.81
      1397.85 180.81 154361.67]*10^(-9)
 
J2 = [535568.72 -9722.76 -5566.78
      -9722.76 0.212521869*10^7 234962.10
      -5566.78 234962.10 393943.69] * 10^(-9)

J3 = [531069.96 1748.90 -11135.75
      1748.90 600306.89 -54475.88
      -11135.75 -54475.88 140564.13]* 10^(-9)

% position  vectors
s1 = [0
    0
    0.041];
s2 = [0.187
    0
    0];
s3 = [0.149
    0
    0];

%center of mass position vectors
sc1 = RB1 * s1

sc2 = RB1 * R12 * s2

sc3 = RB1 * R12 *R23 * s3

%% velocities
 omega1 = RB1*[0; 0; dtheta1]
 vc1 = [0;0;0];
 v1 = RB1*(cross(omega1,s1));
 
 omega2 = omega1 + R12*dtheta2*z
 vc2 = R12*(cross(omega1,sc2))
 v2 = R12*(v1+cross(omega2,s2))

 omega3 = omega2+R23*dtheta3*z
 vc3 = R23*(v2+cross(omega2,sc3))
 
 %% Lagrangian formulation
 T1 = vpa(1/2 * m1 * dot(vc1,vc1) + 1/2 *  dot(omega1,(J1*omega1)),2);
 V1 = vpa(-m1*dot(gSym,sc1),3);
 
 T2 = vpa(1/2 * m2 *dot(vc2,vc2) + 1/2 * dot(omega2,(J2*omega2)),2);
 V2 = vpa(-m2 * dot(gSym,sc2),3);
 
 T3 = vpa(1/2 * m3 * dot(vc3,vc3) + 1/2 * dot(omega3,(J3*omega3)),2);
 V3 = vpa(-m3 * dot(gSym,sc3),3);
 
 L = vpa(T1-V1 + T2-V2 + T3-V3,2)
 
 dT1_dtheta = diff(L,dtheta1);
 dT_dT1_dtheta = vpa(diff(dT1_dtheta,theta1)*dtheta1 + diff(dT1_dtheta,theta2)*dtheta2 + diff(dT1_dtheta,theta3)*dtheta3 + diff(dT1_dtheta,dtheta1)*ddtheta1 + diff(dT1_dtheta,dtheta2)*ddtheta2 + diff(dT1_dtheta,dtheta3)*ddtheta3,2);
 dT1_theta = vpa(diff(L,theta1),2);
 
 dT2_dtheta = diff(L,dtheta2);
 dT_dT2_dtheta = vpa(diff(dT2_dtheta,theta1)*dtheta1 + diff(dT2_dtheta,theta2)*dtheta2 + diff(dT2_dtheta,theta3)*dtheta3 + diff(dT2_dtheta,dtheta1)*ddtheta1 + diff(dT2_dtheta,dtheta2)*ddtheta2 + diff(dT2_dtheta,dtheta3)*ddtheta3,2);
 dT2_theta = vpa(diff(L,theta2),2);
 
 dT3_dtheta = diff (L,dtheta3);
 dT_dT3_dtheta = vpa(diff(dT3_dtheta,theta1)*dtheta1 + diff(dT3_dtheta,theta2)*dtheta2 + diff(dT3_dtheta,theta3)*dtheta3 + diff(dT3_dtheta,dtheta1)*ddtheta1 + diff(dT3_dtheta,dtheta2)*ddtheta2 + diff(dT3_dtheta,dtheta3)*ddtheta3,2);
 dT3_theta = vpa(diff(L,theta3),2);
 
 tau1 = vpa(simplify(dT_dT1_dtheta - dT1_theta),2);
 tau2 = vpa(simplify(dT_dT2_dtheta - dT2_theta),2);
 tau3 = vpa(simplify(dT_dT3_dtheta - dT3_theta),2);
 
 tau_syms=vpa([tau1;tau2;tau3],3);
 
 %% General larange equation

    % mass matrix
    M(1,1) = vpa(diff(tau1, ddtheta1),2);
    M(1,2) = vpa(diff(tau1, ddtheta2),2);
    M(1,3) = vpa(diff(tau1, ddtheta3),2);
    
    M(2,1) = vpa(diff(tau2, ddtheta1),2); 
    M(2,2) = vpa(diff(tau2, ddtheta2),2); 
    M(2,3) = vpa(diff(tau2, ddtheta3),2); 
    
    M(3,1) = vpa(simplify(diff(tau3, ddtheta1)),2); 
    M(3,2) = vpa(simplify(diff(tau3, ddtheta2)),2);
    M(3,3) = vpa(simplify(diff(tau3, ddtheta3)),2);
    
    MasseMatrix = vpa(M,2)  
    vel1 = 0;
    vel2 = 0;
    vel3 = 0;
    
    %viscous friction and coulumb vector
    V1 = subs(tau1,[dtheta1,dtheta2,dtheta3],[vel1,vel2,vel3]);
    V2 = subs(tau2,[dtheta1,dtheta2,dtheta3],[vel1,vel2,vel3]);
    V3 = subs(tau3,[dtheta1,dtheta2,dtheta3],[vel1,vel2,vel3]);
    
    V(1,1) = tau1 - V1;
    V(1,2) = tau2 - V2;
    V(1,3) = tau3 - V3;
    
    VF_CVector = vpa(V,3)
    
    
    %Gravity vector
    Gvector(1,1) = vpa(simplify(diff(tau1,G)),3)*G;
    Gvector(2,1) = vpa(simplify(diff(tau2,G)),3)*G;
    Gvector(3,1) = vpa(simplify(diff(tau3,G)),3)*G;
    
    GravityVector = vpa(Gvector)
    
    %Torque matrix
    ddthetaVector = [ddtheta1;ddtheta2;ddtheta3] %acceleration vector
    
    Tau = vpa(M*ddthetaVector + VF_CVector+GravityVector,2);
   %% Test
     % set angle velocity acceleration
    ang1 = deg2rad(0);
    ang2 = deg2rad(90);
    ang3 = deg2rad(0);
    
    g = 9.82; %gravity vector with numbers
    
    vel1 = 0;
    vel2 = 0;
    vel3 = 0;
    
    acc1 = 0;
    acc2 = 0;
    acc3 = 0;

    torque1 = subs(tau1,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g])
    torque2 = subs(tau2,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g])
    torque3 = subs(tau3,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g])
   
    MX106_1_current = vpa(0.033*torque1^2+0.5087*torque1,3)
    MX106_2_current = vpa(0.033*torque2^2+0.5087*torque2,3)
    MX64_current = vpa(-0.0387*torque3^2+1.0153*torque3,3)
    
   
    
    
    T = 3;
    N = 61;
    i = 0;
    acc2 = 0;
    
    theta0 = 0;
    thetaf12 = 90;
    tf = 3;
    a0 = theta0;
    a1 = 0;
    a2 = 3/(tf*tf)*(thetaf12-theta0); 
    a3 = -2/(tf*tf*tf)*(thetaf12-theta0);
    
    t=0:0.05:tf;
    theta=a0+a1*t+a2*t.^2+a3*t.^3;
    thetaPrik = a1+2*a2*t+3*a3*t.^2;
    thetaPrikPrik = 2*a2+6*a3*t;
 
figure(1)
clf
figure(1)
hold on
plot(t,theta, 'b')
plot(t,thetaPrik,'r')
plot(t,thetaPrikPrik,'m')
hold off
legend('Angular position', 'Angular velocity', 'Angular acceleration')
grid on; 
xlabel('time [sec]'); ylabel('Degrees');
    
    
for t = linspace(0,T,N)
    i = i + 1; time(i) = t;
    ang2 = deg2rad(theta(i));
    vel2 = thetaPrik(i);
    acc2 = thetaPrikPrik(i);
    
 %   torque1 = subs(tau1,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
    torque2(i) = subs(tau2,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
  %  torque3 = subs(tau3,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
    
 %   MX106_1_current = vpa(0.033*torque1^2+0.5087*torque1,3);
    MX106_2_current(i) = vpa(0.033*torque2(i)^2+0.5087*torque2(i),3);
 %   MX64_current(i) = vpa(-0.0387*torque3^2+1.0153*torque3,3);    
end

figure(2)
clf
figure(2)
hold on
plot(time, MX106_2_current, 'r')
plot(ForJimJam1.VarName6*0.001,ForJimJam1.VarName2*0.001, 'b')
plot(ForJimJam4.VarName6*0.001,ForJimJam4.VarName2*0.001, 'g')
hold off
legend('Calculated current', 'Measured current','calulatedMX64')
grid on; 
xlabel('time [sec]'); ylabel('current [A]');


%% test 2

    ang1 = deg2rad(0);
    ang2 = deg2rad(0);
    ang3 = deg2rad(0);
    
    g = 9.82; %gravity vector with numbers
    
    vel1 = 0;
    vel2 = 0;
    vel3 = 0;
    
    acc1 = 0;
    acc2 = 0;
    acc3 = 0;


    theta0 = 0;
    thetaf12 = 90;
    tf = 3.3;
    a0 = theta0;
    a1 = 0;
    a2 = 3/(tf*tf)*(thetaf12-theta0); 
    a3 = -2/(tf*tf*tf)*(thetaf12-theta0);
    
    t=0:0.05:tf;
    theta=a0+a1*t+a2*t.^2+a3*t.^3;
    thetaPrik = a1+2*a2*t+3*a3*t.^2;
    thetaPrikPrik = 2*a2+6*a3*t;

    
    T = 3.3;
    N = 61;
    i = 0;
    
figure(4)
clf
figure(4)
hold on
plot(t,theta, 'b')
plot(t,thetaPrik,'r')
plot(t,thetaPrikPrik,'m')
hold off
legend('Angular position', 'Angular velocity', 'Angular acceleration')
grid on; 
xlabel('time [sec]'); ylabel('Degrees');
    
for t = linspace(0,T,N)
    i = i + 1; time(i) = t;
    ang3 = deg2rad(theta(i));
    vel3 = thetaPrik(i);    
    acc3 = thetaPrikPrik(i);
    
   % torque1 = subs(tau1,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
    torque2(i) = subs(tau2,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
    torque3(i) = subs(tau3,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
    
   % MX106_1_current = vpa(0.033*torque1^2+0.5087*torque1,3);
    MX106_2_current(i) = vpa(0.033*torque2(i)^2+0.5087*torque2(i),3);
    MX64_current(i) = vpa(-0.0387*torque3(i)^2+1.0153*torque3(i),3);    
end

figure(3)
clf
figure(3)
hold on
plot(time, MX64_current, 'r')
plot(ForJimJam2.VarName6*0.001, ForJimJam2.VarName2*0.001, 'b')
%plot(ForJimJam1.VarName6*0.001,ForJimJam1.VarName2*0.001, 'b')
hold off
legend('Calculated current','Measured current')
grid on; 
xlabel('time [sec]'); ylabel('current [A]');



%% test3
    ang1 = deg2rad(0);
    ang2 = deg2rad(90);
    ang3 = deg2rad(0);
    
    vel1 = 0;
    vel2 = 0;
    vel3 = 0;
    
    acc1 = 0;
    acc2 = 0;
    acc3 = 0;
    
    T = 3;
    N = 61;
    i = 0;
    acc2 = 0;
    
    theta0 = 0;
    thetaf12 = 70;
    tf = 3;
    a0 = theta0;
    a1 = 0;
    a2 = 3/(tf*tf)*(thetaf12-theta0); 
    a3 = -2/(tf*tf*tf)*(thetaf12-theta0);
    
    t=0:0.05:tf;
    theta=a0+a1*t+a2*t.^2+a3*t.^3;
    thetaPrik = a1+2*a2*t+3*a3*t.^2;
    thetaPrikPrik = 2*a2+6*a3*t;
 
figure(6)
clf
figure(6)
hold on
plot(t,theta, 'b')
plot(t,thetaPrik,'r')
plot(t,thetaPrikPrik,'m')
hold off
legend('Angular position', 'Angular velocity', 'Angular acceleration')
grid on; 
xlabel('time [sec]'); ylabel('Degrees');
    
    
for t = linspace(0,T,N)
    i = i + 1; time(i) = t;
    ang2 = deg2rad(theta(i));
    vel2 = thetaPrik(i);
    acc2 = thetaPrikPrik(i);
    
 %   torque1 = subs(tau1,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
    torque2(i) = subs(tau2,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
  %  torque3 = subs(tau3,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
    
 %   MX106_1_current = vpa(0.033*torque1^2+0.5087*torque1,3);
    MX106_2_current(i) = vpa(0.033*torque2(i)^2+0.5087*torque2(i),3);
 %   MX64_current(i) = vpa(-0.0387*torque3^2+1.0153*torque3,3);    
end

figure(5)
clf
figure(5)
hold on
plot(time, MX106_2_current, 'r')
plot(ForJimJam5.VarName6*0.001,ForJimJam5.VarName2*0.001, 'b')
hold off
legend('Calculated current', 'Measured current')
grid on; 
xlabel('time [sec]'); ylabel('current [A]');

%% test4
time = 0;
MX106_2_current = 0;
    ang1 = deg2rad(0);
    ang2 = deg2rad(90);
    ang3 = deg2rad(0);
    
    vel1 = 0;
    vel2 = 0;
    vel3 = 0;
    
    acc1 = 0;
    acc2 = 0;
    acc3 = 0;
    
    T = 3.2;
    N = 31;
    i = 0;
    acc2 = 0;
    
    theta0 = 0;
    thetaf12 = 60;
    tf = 3.2;
    a0 = theta0;
    a1 = 0;
    a2 = 3/(tf*tf)*(thetaf12-theta0); 
    a3 = -2/(tf*tf*tf)*(thetaf12-theta0);
    
    t=0:0.05:tf;
    theta=a0+a1*t+a2*t.^2+a3*t.^3;
    thetaPrik = a1+2*a2*t+3*a3*t.^2;
    thetaPrikPrik = 2*a2+6*a3*t;
 
figure(8)
clf
figure(8)
hold on
plot(t,theta, 'b')
plot(t,thetaPrik,'r')
plot(t,thetaPrikPrik,'m')
hold off
legend('Angular position', 'Angular velocity', 'Angular acceleration')
grid on; 
xlabel('time [sec]'); ylabel('Degrees');
    
    
for t = linspace(0,T,N)
    i = i + 1; time(i) = t;
    ang2 = deg2rad(theta(i));
    vel2 = thetaPrik(i);
    acc2 = thetaPrikPrik(i);
    
 %   torque1 = subs(tau1,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
    torque2(i) = subs(tau2,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
  %  torque3 = subs(tau3,[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,g]);
    
 %   MX106_1_current = vpa(0.033*torque1^2+0.5087*torque1,3);
    MX106_2_current(i) = vpa(0.033*torque2(i)^2+0.5087*torque2(i),3);
 %   MX64_current(i) = vpa(-0.0387*torque3^2+1.0153*torque3,3);    
end

figure(7)
clf
figure(7)
hold on
plot(time, MX106_2_current, 'r')
plot(ForJimJam6.VarName6*0.001,ForJimJam6.VarName2*0.001, 'b')
hold off
legend('Calculated current', 'Measured current')
grid on; 
xlabel('time [sec]'); ylabel('current [A]');

