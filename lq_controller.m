%% Linear Quadratic Optimal Control for ACC 
% Nam Anh Mai 
clear all
close all
clc
%% Car inputs
t = 0:0.1:5; % time interval
N = length(t);
t_hw = 2; % time head-way between lead and host vehicle
e0 = 1e-5;
X0 = [0; 0; 0];
al = abs(4*sin(t));
a = abs(3*sin(t));
u = [al;a]';
% x0 = [0;0;0];
vl = 30 + al.*t;
v = 40 + a.*t;
xl = vl.*t + 0.5*(al.^2);
x = v.*t + 0.5*(a.^2);
err = t_hw-(xl-x);

X = [xl-x; vl; v];
U = [al; a];
%% State-space system
A = [0 1 -1; 0 0 0; 0 0 0];
B = [0 0; 1 0; 0 1];
C = [-1 0 t_hw; 0 e0 0];
D = [0 0; 0 0];

sys = ss(A,B,C,D);
[Yo,t,Xo] = lsim(sys,U,t,X0);

[num1,den] = ss2tf(A,B,C,D,1); % find TF for open loop system
[num2,den] = ss2tf(A,B,C,D,2);
r = roots(den);
h1 = tf(num1(1,:),den);
h2 = tf(num2(1,:),den);

%% Step response for open loop system
% %Step response of leading car acceleration
% step(h1,t);
% figure;
% %Step response of host car acceleration
% step(h2,t);

%%
% ux = [zeros(1,length(t))];
% y1u1 = filter(num1(1,:),den,ux);
% stem(t,y1u1)
% step(sys,u,t)
% lsim(sys)

% Check controllability
Co = ctrb(A,B);
unco = length(A) - rank(Co); %number of uncontrollable states
%Check observability
Ob = obsv(A,C);
unob = length(A) - rank(Ob); %number of unobservable states

%% Solving Riccati equation
lambda = 1; % tuning parameter
R =  lambda*[1/e0 0; 0 1];
G = B*(R^(-1))*B';
Q = (C')*C;
% P = Riccati(A,G,Q); % semi-definitive solution
% K = (R^(-1))*(B')*P; % state feedback gain
K = lqr(sys,Q,R);

%% Closed-loop state feedback system
Ac = A - B*K;
Bc = B;
Cc = C;
Dc = D;
csys = ss(Ac,Bc,Cc,Dc);
Y = C*((-Ac)^(-1))*B;
[Yc,t,Xc] = lsim(csys,U,t,X0);
% Uf = -K*X;
figure;
subplot(211);
plot(t,Xo(:,1),'r',t,Xc(:,1),'g');
grid;
legend('Open-loop','Closed-loop');
subplot(212);
plot(t,Yo(:,1),'r',t,Yc(:,1),'g');
grid;
legend('Open-loop','Closed-loop');
