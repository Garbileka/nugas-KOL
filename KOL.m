% file name: KOL.m 
clear
clc
t = 0:0.001:10;
% DATA1=============
J = 0.01;
b = 0.00003;
K = 0.023;
R = 1;
L = 0.5;
A = [-b/J   K/J
     -K/L   -R/L];

B = [0
     1/L];
C = [1   0]; 
D = 0;
% sys = ss(A,B,C,D);
num=K;
den=[(J*L) ((J*R)+(L*b)) ((b*R)+K^2)];
open=tf(num,den);
closed= feedback(open,1)
%============PID===============
Kp = 150;  %
Ki = 150;  % took by try 
Kd = 0.4;  %
PID = tf([Kd Kp Ki],[1 0]); 
PIDsys = feedback(PID*open,1);
% ++++++++++++++++++++++++++++++++++% 
% Linear Quadratic Regulator design LQR 
% %+++++++++++++++++++++++++++++++++%
Q=[.2 0;0 0.028];
R=[.2];
[KK,S,e] =lqr(A,B,Q,R)
ZZ=(A-B*KK);
LQR=ss(ZZ,B,C,D);
damp(LQR)
[num1,den1]=ss2tf(ZZ,B,C,D,1);
G=tf(num1,den1)    %ALWAYS den=1

%++++++++++++++++++++++++++++++++++%
% Linear Quadratic Regulator design LQR Step             
%++++++++++++++++++++++++++++++++++%
figure(1)
step(closed,t),title('Closed Loop step response')
xlabel('Time','FontSize',11);
ylabel('P.U. speed','FontSize',11); 
figure(2)
step(PIDsys,t),title('PID step response')
xlabel('Time','FontSize',11);
ylabel('P.U. speed','FontSize',11);
figure(3)
step(LQR,t),title('LQR step response')
xlabel('Time','FontSize',11);
ylabel('P.U. speed','FontSize',11);
figure(4)
step(PIDsys,LQR,closed,t),title('step all')
xlabel('Time','FontSize',11);
ylabel('P.U. speed','FontSize',11); 
