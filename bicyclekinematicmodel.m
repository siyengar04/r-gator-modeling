clear;


v = 5; %inital velocity, m/s
caf = 2.787e-4; %cornering stiffness of front tire
car = 2.787e-4; % ''     '' of rear tire
lf = 0.5; %cog to front axle, m
lr = 0.5; %cog to rear axle, m
m = 12; %bicycle weight, kg
theta = pi/2; %initial steering angle, radians


a11 = (-1/v)*((caf*lf^2)+(car*lr^2)/theta);
a12 = -1*((caf*lf)-(car*lr))/theta;
a21 = -1-(1/v^2)*((caf*lf)-(car*lr))/m;
a22 = (-1/v)*((caf+car)/m);

b11 = (caf*lf)/theta;
b12 = (1/v)*caf/m;

A = [a11, a12; 
    a21, a22];

B = [b11;
    b12;];

C = [1,0;0,1];

D = [0;0;];
syms s
% sys = ss(A,B,C,D);
% Define the simulation time span
t = 0:0.01:10; % Adjust the time span as needed
% 
% b = [3*2923731754657427553341434166813655040, - 3*2906843307627018433046389551878264303];
% a = [2*5444517870735015415413993718908291383296, 2*562479712195478469941544517403410432,  2*4755552311528293529155105972227];
% [W,X,Y,Z] = tf2ss(b,a);


phi = inv(s*eye(2)-A);
H = C*phi*B+D

[b,a] = ss2tf(A,B,C,D)
newsys = tf(b(1,:),a)
newsys2 = tf(b(2,:),a)

b1 = b(1,:);
b2 = b(2,:);


% Define the input signal 'delta' (for example, a step input)
delta = rand(size(t)); % You can modify this input as needed

% Simulate the response of 'newsys' (psidot) to 'delta'
y1 = lsim(newsys, delta, t);

% Simulate the response of 'newsys2' (slip angle) to 'delta'
y2 = lsim(newsys2, delta, t);

% Plot the responses
figure;
subplot(2, 1, 1);
plot(t, y1);
title('Response of newsys (psidot) to Input delta');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(2, 1, 2);
plot(t, y2);
title('Response of newsys2 (slip angle) to Input delta');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;