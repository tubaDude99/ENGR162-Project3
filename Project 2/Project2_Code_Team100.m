% Activity: Project 2
% File: Proj2_Code_Team100.m
% Date: 2 April 2023
% By:   Nathan Battle
%       battle6
%       Mazen Sheppard
%       sheppar7
%       Christopher Baron
%       baronc
%       Vishu Pancholi
%       vpanchol
% Section: 4
% Team: 100
%
% ELECTRONIC SIGNATURE
% Nathan Battle
% Mazen Sheppard
% Chistopher Baron
% Vishuddha Pancholi
%
% The electronic signature above indicates that the program
% submitted for evaluation is my individual work. I have
% a general understanding of all aspects of its development
% and execution.
%
% Simulates the diffusion of clean air around the smog-free tower

clear;
clc;
tic;

if input("Do you want to enter values? ", "s") == "no"
    q = 8.33; % Volumetric flow, m^3 / s
    c0 = 36; % Pollution concentration, ppm
    e = 1; % Filtration efficiency
    D = 1.98 * 10^-5; % Diffusion coefficient of N2 in N2
    threshold = .9; % Effective threshold in portion of pollution remaining
    hkArea = 1110000000; % Area of Hong Kong, m^2
    towerCost = 1000; % Cost per tower, $
else
    q = input("Enter the volumetric flow in m^3/s: ");
    c0 = input("Enter the ambient pollution, in ppm: ");
    e = input("Enter the filtration efficiency: ");
    D = 1.98 * 10^-5; % Diffusion coefficient of N2 in N2
    threshold = input("Enter your reduction threshold, in ppm: ");
    hkArea = 1110000000; % Area of Hong Kong, m^2
    towerCost = input("Enter the cost per tower, in $: ");
end

xStart = -3;
xStop = 3;
xStep = .05;
xLength = (xStop - xStart) / xStep + 1;

rStart = -1;
rStop = 1;
rStep = .05;
rLength = (rStop - rStart) / rStep + 1;

tStart = 1;
tStop = 36000;
tStep = 1;
tLength = (tStop - tStart) / tStep + 1;

c1 = c0 * (1 - e);
c = zeros(2, xLength, xLength, rLength);
c = c + c0;
d = zeros(xLength, xLength, rLength, 3);

disp("Program execution begun with " + (xLength * rLength^2 * tLength) + " iterations");

xMid = ceil(xLength/2);
rMid = ceil(rLength/2);

outflowR = round((q * tStep)^(1/3) / (rStep * 2));

for tIndex = 1:tLength
    c(1, xMid-outflowR:xMid+outflowR, rMid-outflowR:rMid+outflowR, rMid-outflowR:rMid+outflowR) = c1;
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
                dx = (d(xIndex+1,yIndex,zIndex,1) - d(xIndex-1,yIndex,zIndex,1)) / (2 * rStep);
                dy = (d(xIndex,yIndex+1,zIndex,2) - d(xIndex,yIndex-1,zIndex,2)) / (2 * rStep);
                dz = (d(xIndex,yIndex,zIndex+1,3) - d(xIndex,yIndex,zIndex-1,3)) / (2 * rStep);
                dc = dx + dy + dz;
                c(2,xIndex,yIndex,zIndex) = c(1,xIndex,yIndex,zIndex) + D*dc*tStep;
            end
        end
    end
    c(1,:,:,:) = c(2,:,:,:);
end
toc

figure;
x = xStart:xStep:xStop;
plot(x,c(1,:,rMid,rMid));
output = [transpose(x) permute(c(1,:,rMid,rMid), [2 3 4 1])];

%xVals = zeros(xLength,xLength);
%yVals = zeros(xLength,xLength);
%cVals = zeros(xLength,xLength);

%for i = 1:length(x)
%    for j = 1:length(x)
%        xVals(i,j) = output(i,1);
%        yVals(i,j) = output(j,1);
%        cVals(i,j) = c(1,i,j,rMid);
%    end
%end

%figure;
%surf(xVals,yVals,cVals);

for i = 1:length(output)
    if output(i,2) <= c0 * threshold
        safeDist = abs(output(i,1));
        break
    end
end
safeArea = safeDist^2 * pi;
towerCount = round(hkArea / safeArea);
disp("Effective radius: " + safeDist + " m");
disp("Coverage: " + safeArea + " m^2");
disp("It would take " + towerCount + " towers to cover all of Hong Kong, and the project would cost $" + (towerCount * towerCost));