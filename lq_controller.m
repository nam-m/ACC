%% Linear Quadratic Optimal Control for ACC 
% Nam Anh Mai 

%% Car inputs
xl = 30;
x = 20;
vl = 30;
v = 30;


%% State-space system
t_hw = 2; % time head-way between lead and host vehicle
e0 = 1e-5;
A = [0 1 -1; 0 0 0; 0 0 0];
B = [0 0; 1 0; 0 1];
C = [-1 0 t_hw; 0 e0 0];
D = 0;
sys = ss(A,B,C,D);

%% Solving Riccati equation
lambda = 1; % tuning parameter
R =  lambda*[1/e0 0; 0 1];
G = B*(R^(-1))*B';
Q = (C')*C;
P = Riccati(A,G,Q); % semi-definitive solution

K = (R^(-1))*(B')*P;