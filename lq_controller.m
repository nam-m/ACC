%% Linear Quadratic Optimal Control for ACC 
% Nam Anh Mai 
clear all
close all
clc
%% Car inputs
t = 0:0.1:20; % time interval
N = length(t);
t_hw = 2; % time head-way between lead and host vehicle
e0 = 1e-5;

X0 = [0; 0; 0];
al = abs(4*sin(t));
a = abs(3*sin(t));
u = [al;a]';
x0 = [0;0;0];
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
%Yo: open-loop system response
%Xo: open-loop input
%t: time vector
[Yo,t,Xo] = lsim(sys,U,t,X0);

% find TF for open loop system
[num1,den] = ss2tf(A,B,C,D,1); 
[num2,den] = ss2tf(A,B,C,D,2);
r = roots(den);
ho1 = tf(num1(1,:),den); 
ho2 = tf(num2(1,:),den);
% K = place(A,B,r);

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
[numc1,denc] = ss2tf(Ac,Bc,Cc,Dc,1); % find TF for open loop system
[numc2,denc] = ss2tf(Ac,Bc,Cc,Dc,2);
hc1 = tf(numc1(1,:),denc);
hc2 = tf(numc2(1,:),denc);
rc = roots(denc);

Y = C*((-Ac)^(-1))*B;
[Yc,t,Xc] = lsim(csys,U,t,X0);
% Uf = -K*X;

%% Plotting
figure;
subplot(311);
plot(t,Xo(:,1),'r',t,Xc(:,1),'b');
grid;
legend('Open-loop','Closed-loop');
subplot(312);
plot(t,Yo(:,1),'r',t,Yc(:,1),'b');
grid;
legend('Open-loop','Closed-loop');
% subplot(313);
% plot(t,Yo(:,2),'r',t,Yc(:,2),'b');
% grid;
% legend('Open-loop','Closed-loop');
