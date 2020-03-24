%% Linear Quadratic Optimal Control for ACC 
% Nam Anh Mai 

%% Car inputs
t = 0:0.1:5; % time interval
t_hw = 2; % time head-way between lead and host vehicle
e0 = 1e-5;

al = abs(4*sin(t));
a = abs(3*sin(t));
u = [al;a]';
x0 = [0;0;0];
xl = 30*t;
x = 20*t;
vl = abs(30*sin(t));
v = abs(20*sin(t));

err = t_hw-(xl-x);

%% State-space system
A = [0 1 -1; 0 0 0; 0 0 0];
B = [0 0; 1 0; 0 1];
C = [-1 0 t_hw; 0 e0 0];
D = [0 0; 0 0];

sys = ss(A,B,C,D);
% step(sys)
[num1,den] = ss2tf(A,B,C,D,1); % find TF for open loop system
[num2,den] = ss2tf(A,B,C,D,2);
% ux = [zeros(1,length(t))];
% y1u1 = filter(num1(1,:),den,ux);
% stem(t,y1u1)
% step(sys,u,t)
lsim(sys)

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
P = Riccati(A,G,Q); % semi-definitive solution

K = (R^(-1))*(B')*P; % state feedback gain
Ac = A - B*K;
Y = C*((-Ac)^(-1))*B;