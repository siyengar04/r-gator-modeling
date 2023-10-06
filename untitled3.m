l = 1;
lH = .5;
lV = 0.5;
rhoM = 2.5;
rho = 2.5;
deltaA = 5;
v = 5;
iL = 3.0; 

delta = deltaH*(1/iL);
tan(deltaA) = l/sqrt((rhoM)^2-(lH^2));
deltaA = l/rhoM;
psidot = v/rho;
vV = [v*cos(beta);v*sin(beta);0];
vA = [-v(psidotV + betadot)*sin(beta); v(psidotV+betadot)*cos(beta);0];
aN = v*(psidotV + betadot);
rhoK = v/(psidotV + betadot);
aY = v^2/rhoK;
vVv = [v*cos(beta);v*sin(beta)+lV*psidotV;0];
vVh = [v*cos(beta);v*sin(beta)-lHpsidotV;0];
vKv = [vV*cos(delta-alphaV);vV*sin(delta-alphaV);0];
alphaV = delta - beta - lV*(psidotV)/v;
alphaH = -beta+lH*psidotV/v;
forceVY = cAlphaV*alphaV;
forceHY = cAlphah*alphaH; 
psiV = arctan(alphaV/sV);
forceVXstat = cSV*sV;
forceVYstat = cAlphaV*alphaV;
forceHXstat = cSH*sH;
forceHYstat = cAlphaH*alphaH;



%variables in xdot
xdotdotV = (1/m)*(forceVX+forceHX-forceWX);
ydotdotV = (1/m)*(forceVY+forceHY-forceWX);
psidotdotV = (1/thetaZZ)*(lV*forceVY-lH*forceHY);
rhodotdotV = (1/thetaV)*(mAcapV-mBV*sign(rhodotV)-r*forceVX);
rhodotdotH = (1/thetaH)*(mAH-mBH*sign(rhodotH)-r*forceHX);
forcedotVX = (cVX*abs(r*rhodotV))/cSV*(forceVXstat-forceVX);
forcedotVY = (cVY*abs(r*rhodotV))/cAlphaH*(forceVYstat-forceVY);
forcedotHX = (cHX*abs(r*rhodotH))/cSH*(forceHXstat-forceHX);
forcedotHY = (cHY*abs(r*rhodotH))/cAlphaH*(forceHYstat-forceHY);




% Define the state-space matrices
A = [0.8, 0.2, 0.1; 0.1, 0.7, 0.2; 0.2, 0.1, 0.9]; % 3x3 matrix
B = [0.1; 0.2; 0.3]; % 3x1 matrix
C = [1, 0, 0]; % 1x3 matrix
D = 0; % Scalar (direct transmission)

% Create a state-space model
sys = ss(A, B, C, D);

% Initial state vector (12 elements)
xdot = [xdotV, ydotV, phidotV, xdotdotV, ydotdotV, psidotdotV, rhodotdotV, rhodotdotH, forcedotVX, forcedotVY, forcedotHX, forcedotHY]'; % 12x1 matrix

% Input vector (you can define this based on your control input)
u = [deltaH, pF, pB, G]'; % Scalar (for simplicity)

% Simulate the system over time
num_time_steps = 10;

% Initialize arrays to store state and output trajectories
x_traj = zeros(4, num_time_steps);
y_traj = zeros(1, num_time_steps);

% Iterate over time steps
x_t = x0;
for t = 1:num_time_steps
    % Update the state using the state-space model
    [y_t, x_t] = lsim(sys, u, [], x_t);
    
    % Store the state and output at this time step
    x_traj(:, t) = x_t';
    y_traj(t) = y_t;
end

% Display the results
disp('State Trajectory:');
disp(x_traj);
disp('Output Trajectory:');
disp(y_traj);