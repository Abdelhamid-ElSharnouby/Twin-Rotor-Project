%% Variables:
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
Kt = 100e-6;
w0= 2;
bt = 10e-3;
J = 58.23e-3;
ki=0;
kp=1;
kd=1e-100;
%% State Space:
A = [-R/Lm,0,0,0,0,0,0,-Kv/Lm;
     0,-R/Lm,0,0,0,0,-Kv/Lm,0;
     0,0,0,0,0,0,0,1;
     0,0,0,0,0,0,1,0;
     0,0,0,0,0,1,0,0;
     0,0,0,0,-(0.5*Lb*mb*g)/J,-bt/J,(2*l*Kt*w0)/J,-(2*l*Kt*w0)/J;
     0,Ktm/Jm,0,0,0,0,-btm/Jm,0;
     Ktm/Jm,0,0,0,0,0,0,-btm/Jm];

B= [1/Lm 0;
    0 1/Lm;
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    0 0];

C= [0 0 0 0 1 0 0 0];

D= zeros(1,2);

%% System Response:
[b,a] = ss2tf(A, B, C, D, 1);
G1= tf(b,a);
[b,a] = ss2tf(A, B, C, D, 2);
G2= tf(b,a); %MISO system, 2 tranfster functions included
G=[G1 G2]; %Compine system transfer functions

t = 0:0.1:100;
% Initialize input: 2 columns for 2 inputs
u = zeros(length(t), 2);

% pulse on input 1 from t = 0 to 1s
u(t >= 0 & t < 1, 1) = 0.01;

% pulse on input 2
u(t >= 0 & t < 1, 2) = 0.02;
[y, t_out] = lsim(G, u, t);

% Plot
figure;
plot(t_out, y, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Theta');
title('Open-Loop Response on Both Inputs');
grid on;
% Get response characteristics from the simulated output
info = stepinfo(y, t_out);

% Display Rise Time, Settling Time, Overshoot
fprintf('Rise Time: %.4f seconds\n', info.RiseTime);
fprintf('Settling Time: %.4f seconds\n', info.SettlingTime);
fprintf('Overshoot: %.2f%%\n', info.Overshoot);
% Convert original G1 to state-space
sys = ss(G1);

% Choose reduced order (try 4 as example)
reduced_order = 4;

% Perform balanced truncation model reduction
sysr = balred(sys, reduced_order);

% Convert reduced model back to transfer function (optional)
G1_reduced = tf(sysr);
% Convert original G1 to state-space
sys_2 = ss(G2);

% Perform balanced truncation model reduction
sysr_2 = balred(sys_2, reduced_order);

% Convert reduced model back to transfer function (optional)
G2_reduced = tf(sysr_2);

%% Compare step responses of original and reduced models
figure;
step( sys, sysr);
legend('Original G1','Reduced G1');
title('Step Response Comparison: Original vs Reduced Model');
grid on;
G_reduced = [G1_reduced G2_reduced];
[y, t_out] = lsim(G_reduced, u, t);
figure;
plot(t_out, y, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Theta');
title('Open-Loop Response on Both Inputs - Reduced');
grid on;

%% convert G to G_reduced
G1 = G1_reduced;
G2 = G2_reduced;
G  =  G_reduced;

%% Get coefficients
[num, den] = tfdata(G1, 'v');
disp('Numerator coefficients:');
disp(num);
disp('Denominator coefficients:');
disp(den);
[num_2, den_2] = tfdata(G2, 'v');
disp('Numerator coefficients:');
disp(num_2);
disp('Denominator coefficients:');
disp(den_2);
 %% Root Locus
 figure;
 rlocus(G1);     % Plots root locus
grid on;       % Adds damping and frequency lines
title('Root Locus of G1(s)');
figure;
 rlocus(G2);     % Plots root locus
grid on;       % Adds damping and frequency lines
title('Root Locus of G2(s)');