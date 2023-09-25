theta = 0.1;
v = 1;
car = 867;
caf = 867;
lr = 1;
lf = 1;
m = 12;
A = [(-1/v)*((caf*lf^2+car*lr^2)/theta) -1*(caf*lf-car*lr)/theta; -1-(1/v^2)*((caf*lf-car*lr)/m) (-1/v)*(caf+car)/m];
B = [(caf*lf)/theta; (1/v)*(caf/m)];
C = [0 1];
D = 0;

sys = ss(A,B,C,D);
