clear;
clc;

% k = 1.380649 * 10^-23; % Boltzmann constant
% T = 293; % Absolute temperature (K)
% n = .00001516; % Viscocity of air
% d = 0.0000025; % particle diameter
% r = d/2; % particle radius
% f = 6 * pi * n * r; % Frictional coefficient
% D = k * T / f; % Diffusion coefficient

c0 = 1;
D = 1.98 * 10^-5; % Diffusion coefficient of N2 in N2

xStart = -1;
xStop = 1;
xStep = .02;
tStart = 20;
tStop = 4000;
tStep = 20;

xVals = zeros((xStop-xStart)/xStep + 1, (tStop-tStart)/tStep + 1);
tVals = zeros((xStop-xStart)/xStep + 1, (tStop-tStart)/tStep + 1);
cVals = zeros((xStop-xStart)/xStep + 1, (tStop-tStart)/tStep + 1);

xList = [xStart:xStep:-.1 -.09:.01:.09 .1:xStep:xStop];
tList = tStart:tStep:tStop;

for xIndex = 1:length(xList)
    for tIndex = 1:length(tList)
        xVals(xIndex, tIndex) = xList(xIndex);
        tVals(xIndex, tIndex) = tList(tIndex);
        cVals(xIndex, tIndex) = 0;
        for tPast = 1:tIndex
            cVals(xIndex, tIndex) = cVals(xIndex, tIndex) + concentration(tList(tPast),xList(xIndex),c0*tStep/tStop,D);
        end
    end
end

figure;
surf(xVals, tVals, cVals);
figure;
subplot(2,2,1);
plot(xVals(:,end),cVals(:,end));
subplot(2,2,2);
plot(tVals(ceil(length(xList)/2),:),cVals(ceil(length(xList)/2),:));
subplot(2,2,4);
plot(tVals(ceil(length(xList)/2),:),exp(cVals(ceil(length(xList)/2),:)));

function c = concentration(t, x, C, D)
    c = (C / sqrt(4*pi*D*t)) * exp(x^2 / (-4*D*t));
end