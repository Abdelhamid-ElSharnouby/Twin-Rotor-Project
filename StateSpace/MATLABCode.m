%Variables:
R = 8;
Lm = 860e-6;
btm = 2.348e-9;
Kv = 0.0202;
Ktm = 20.2e-3;
Jm = 172e-6;
Lb = 0.3;
mb = 75e-3;
g = 9.81;
l = 0.6;
Kt = 1e-6;
w0= 2;
bt = 10e-3;
J = 58.23e-3;
%State Space:
A = [-R/Lm,0,0,0,0,0,-Kv,0;
     0,-R/Lm,0,0,0,0,0,-Kv;
     0,0,0,0,0,0,1,0;
     0,0,0,0,0,0,0,1;
     0,0,0,0,0,1,0,0;
     0,0,0,0,-0.5*Lb*mb*g,0,(2*l*Kt*w0 - bt)/J,-(2*l*Kt*w0 - bt)/J;
     Ktm/Jm,0,0,0,0,0,-btm/Jm,0;
     0,Ktm/Jm,0,0,0,0,0,-btm/Jm];

B= [1/Lm 0;
    0 1/Lm;
    0 0; 0 0; 0 0; 0 0; 0 0; 0 0];

C= [0 0 0 0 1 0 0 0];

D= 0;

%System Response:
sys = ss(A, B, C, D);
t = 0:0.1:20000;
% Initialize input: 2 columns for 2 inputs
u = zeros(length(t), 2);

% pulse on input 1 from t = 0 to 1s
u(t >= 0 & t < 1, 1) = 0.02;

% pulse on input 2
u(t >= 0 & t < 1, 2) = 0.01;
[y, t_out] = lsim(sys, u, t);

% Plot
figure;
plot(t_out, y, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Theta');
title('Open-Loop Response to Step on Both Inputs');
grid on;