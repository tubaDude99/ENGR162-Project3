clear;
clc;
tic;

q = .5; % Volumetric flow, m^3 / s
c0 = 36; % Pollution concentration, ppm
e = 1; % Filtration efficiency
D = 1.98 * 10^-5; % Diffusion coefficient of N2 in N2

xStart = -5;
xStop = 5;
xStep = .2;
xLength = (xStop - xStart) / xStep + 1;

rStart = -1;
rStop = 1;
rStep = .2;
rLength = (rStop - rStart) / rStep + 1;

tStart = 1;
tStop = 36000;
tStep = 1;
tLength = (tStop - tStart) / tStep + 1;

c1 = c0 * (1 - e);
c = zeros(2, xLength, rLength, rLength);
c = c + c0;
d = zeros(xLength, rLength, rLength, 3);

disp("Program execution begun with " + (rLength^3 * tLength) + " iterations");

for tIndex = 1:tLength
    c(1, ceil(xLength/2), ceil(rLength/2), ceil(rLength/2)) = c1;
    for xIndex = 2:xLength-1
        for yIndex = 2:rLength-1
            for zIndex = 2:rLength-1
                d(xIndex,yIndex,zIndex,1) = (c(1,xIndex+1,yIndex,zIndex) - c(1,xIndex-1,yIndex,zIndex)) / (2 * rStep);
                d(xIndex,yIndex,zIndex,2) = (c(1,xIndex,yIndex+1,zIndex) - c(1,xIndex,yIndex-1,zIndex)) / (2 * rStep);
                d(xIndex,yIndex,zIndex,3) = (c(1,xIndex,yIndex,zIndex+1) - c(1,xIndex,yIndex,zIndex-1)) / (2 * rStep);
            end
        end
    end
    for xIndex = 3:xLength-2
        for yIndex = 3:rLength-2
            for zIndex = 3:rLength-2
                dx = (d(xIndex+1,yIndex,zIndex) - d(xIndex-1,yIndex,zIndex)) / (2 * rStep);
                dy = (d(xIndex,yIndex+1,zIndex) - d(xIndex,yIndex-1,zIndex)) / (2 * rStep);
                dz = (d(xIndex,yIndex,zIndex+1) - d(xIndex,yIndex,zIndex)-1) / (2 * rStep);
                dc = -abs(dx) - abs(dy) - abs(dz);
                c(2,xIndex,yIndex,zIndex) = c(1,xIndex,yIndex,zIndex) + D*dc*tStep;
            end
        end
    end
    c(1,:,:,:) = c(2,:,:,:);
end
toc

figure;
x = xStart:xStep:xStop;
plot(x,c(1,:,ceil(rLength/2),ceil(rLength/2)));