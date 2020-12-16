function q = inverseKinematics(xc, yc, zc)

% Robot parameters
h = 53;
r = 30.309;
L2 = 170.384;
L3 = 136.307;
L4 = 86.0;
c = 40.0;

% Robot offset
xc = xc - 100;
yc = yc - 290;
zc = zc - 12;

% z error offset
d = sqrt(xc^2 + yc^2);
zc = zc - d*0.05;

% xy error offset
% xc = xc + abs(yc)*0.05;
% yc = yc - 5;

% Calculate q1
q1 = atan2(yc,xc);

% Calculate q3
D = (L2^2 + L3^2 - (d-r)^2 - (zc+c+L4-h)^2) / (2 * L2 * L3);
q3 = atan2(sqrt(1-D^2), D);

% Calculate q2
a = atan2(L3*sin(q3), L2-L3*cos(q3));
b = atan2(zc+c+L4-h, xc-r);
q2 = a + b;

% Calculate q4
q4 = 3*pi/2 - a - b - q3;

% Convert to degrees
q1 = rad2deg(q1);
q2 = rad2deg(q2);
q3 = rad2deg(q3);
q4 = rad2deg(q4);

% Add offset
q1 = -q1 + 240;
q4 = q4-2;

%Cap 
if q2>122
    q2 = 122;
end

% Concatenate
q = [q1 q2 q3 q4];

end