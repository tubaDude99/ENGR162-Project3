d = 0.0000025; % particle diameter
mSmog = 0.000000066; % average particle mass
pSmog = 6 * mSmog / (pi * d^3); % average particle density
pAir = 1.293; % air density
g = 9.8; % gravitational constant
vVert = -.5; % vertical/perpendicular particle velocity
v = .00001516; % kinematic viscosity of air at 20 deg C
Re = abs(vVert*d/v);
Cd = 24 / Re;
eV = .000000000000000000160217663; % fundamental charge
q = 50 * eV; % smog particle charge
s = 1; % plate charge density
e0 = .00000000000885418782; % electrostatic constant
c = 0.000036; % smog concentration
H = .1; % furthest distance from plate
D = H/2; % particle distance from plate

FG = mSmog * g; % Gravity
FB = pi * pAir * g * (d^3) / 6; % Buoyancy Force
FDrag = pAir * Cd * (d^2) * pi * (vVert^2) / 8; % Drag
FEPlate = q * s / (2 * pi * e0); % Electrostatic Force from the charged plate
%FECloud = (q^2) * c * (2*D - H) / (2 * e0); % Electrostatic Force from the cloud of particles

disp("Gravity: " + FG);
disp("Buoyancy: " + FB);
disp("Drag: " + FDrag);
disp("Plate ESF: " + FEPlate);
%disp("Cloud ESF: " + FECloud);
disp("Net Force: " + (FB - FG + FDrag - FEPlate));
