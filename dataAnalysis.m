clc
clear all

%overallData = load_mvnx('PD021-ON_walking_gait_report.mvnx');
% load PD021-ON_walking_gait_report.mat


%%%% Subject 46 %%%%%%%
%%% walking%%%%%%%%%%%%%%%%%%%%
load PD046-ON_walking.mat

overallData = tree;
startL = 1250;
endL = 2950;
leftHeelContact = overallData.footContact(1).footContacts(startL:endL);
leftToeContact = overallData.footContact(2).footContacts(startL:endL);
rightHeelContact = overallData.footContact(3).footContacts(startL:endL);
rightToeContact = overallData.footContact(4).footContacts(startL:endL);
lHeelContact = diff(leftHeelContact);
lToeContact = diff(leftToeContact);
rHeelContact = diff(rightHeelContact);
rToeContact = diff(rightToeContact);

leftFootData = overallData.segmentData(22);
leftToeData = overallData.segmentData(23);
rightFootData = overallData.segmentData(18);
rightToeData = overallData.segmentData(19);

leftFootPosition  = leftFootData.position(startL:endL,:);
leftFootOrient_Eul = quat2eul(leftFootData.orientation(startL:endL,:),'XYZ');

leftToePosition  = leftToeData.position(startL:endL,:);
leftToeOrient_Eul = quat2eul(leftToeData.orientation(startL:endL,:),'XYZ');

% leftFootDistance = sqrt(leftFootData.position(:,1).^2 + leftFootData.position(:,2).^2);
% leftToeDistance =  sqrt(leftToeData.position(:,1).^2 + leftToeData.position(:,2).^2);

rightFootPosition  = rightFootData.position(startL:endL,:);
rightFootOrient_Eul = quat2eul(rightFootData.orientation(startL:endL,:),'XYZ');

rightToePosition  = rightToeData.position(startL:endL,:);
rightToeOrient_Eul = quat2eul(rightToeData.orientation(startL:endL,:),'XYZ');

% rightFootDistance = sqrt(rightFootData.position(:,1).^2 + rightFootData.position(:,2).^2);
% rightToeDistance =  sqrt(rightToeData.position(:,1).^2 + rightToeData.position(:,2).^2);

[lHeelPeaks,lHeelIndexes] = findpeaks(lHeelContact,'MinPeakDistance',10);
% [lToePeaks,lToeIndexes] = findpeaks(lToeContact);

[rHeelPeaks,rHeelIndexes] = findpeaks(rHeelContact,'MinPeakDistance',10);
% [rToePeaks,rToeIndexes] = findpeaks(rToeContact);

[lFootPeaks,lFootIndexes] = findpeaks(leftFootPosition(:,3),'MinPeakHeight',max(leftFootPosition(:,3))/1.3,'MinPeakDistance',30);
[rFootPeaks,rFootIndexes] = findpeaks(rightFootPosition(:,3),'MinPeakHeight',max(rightFootPosition(:,3))/1.3,'MinPeakDistance',30);

% [rFootPeaks,rFootIndexes] = findpeaks(rightFootPosition(:,3),'Thres',1.05);



% 
% figure(2)
% plot(leftFootPosition(:,3));hold on;
% plot(lFootIndexes,lFootPeaks,'o','MarkerSize',12)
% plot(lHeelContact,'black');plot(lHeelIndexes,lHeelPeaks,'x','MarkerSize',12);

lHeelPeaksN = [];
lHeelIndexesN = [];
lFootPeaksN = [];
lFootIndexesN = [];

rHeelPeaksN = [];
rHeelIndexesN = [];
rFootPeaksN = [];
rFootIndexesN = [];

if(length(lHeelIndexes)~= length(lFootIndexes))
    if( length(lFootIndexes) > length(lHeelIndexes))
        for i=1:length(lFootIndexes)
            k = lFootIndexes(i);
            for j=1:length(lHeelIndexes)
                if( abs(k - lHeelIndexes(j)) <= 30)
                    lFootIndexesN = [lFootIndexesN;lFootIndexes(i)];
                    lFootPeaksN = [lFootPeaksN;lFootPeaks(i)];
                    break;
                end
            end
        end
    else 
        for i=1:length(lHeelIndexes)
            k = lHeelIndexes(i);
            for j=1:length(lFootIndexes)
                if(abs(k-lFootIndexes(j))<=30)
                    lHeelIndexesN = [lHeelIndexesN;lHeelIndexes(i)];
                    lHeelPeaksN = [lHeelPeaksN;lHeelPeaks(i)];
                    break;
                end
            end
        end
    end
end

if(length(rHeelIndexes)~= length(rFootIndexes))
    if( length(rFootIndexes) > length(rHeelIndexes))
        for i=1:length(rFootIndexes)
            k = rFootIndexes(i);
            for j=1:length(rHeelIndexes)
                if( abs(k - rHeelIndexes(j)) <= 30)
                    rFootIndexesN = [rFootIndexesN;rFootIndexes(i)];
                    rFootPeaksN = [rFootPeaksN;rFootPeaks(i)];
                    break;
                end
            end
        end
    else 
        for i=1:length(rHeelIndexes)
            k = rHeelIndexes(i);
            for j=1:length(rFootIndexes)
                if(abs(k-rFootIndexes(j))<=30)
                    rHeelIndexesN = [rHeelIndexesN;rHeelIndexes(i)];
                    rHeelPeaksN = [rHeelPeaksN;rHeelPeaks(i)];
                    break;
                end
            end
        end
    end
end


if(isempty(lHeelIndexesN))
    lHeelIndexesN = lHeelIndexes;
end
if(isempty(lHeelPeaksN))
    lHeelPeaksN = lHeelPeaks;
end
if(isempty(lFootIndexesN))
    lFootIndexesN = lFootIndexes;
end
if(isempty(lFootPeaksN))
    lFootPeaksN = lFootPeaks;
end

if(isempty(rHeelIndexesN))
    rHeelIndexesN = rHeelIndexes;
end
if(isempty(rHeelPeaksN))
    rHeelPeaksN = rHeelPeaks;
end
if(isempty(rFootIndexesN))
    rFootIndexesN = rFootIndexes;
end
if(isempty(rFootPeaksN))
    rFootPeaksN = rFootPeaks;
end


% subplot(2,1,2);
% plot(rightFootPosition(:,3));hold on;
% plot(rFootIndexes,rFootPeaks,'o','MarkerSize',12)
% plot(rHeelContact,'black');plot(rHeelIndexes,rHeelPeaks,'x','MarkerSize',12);
% subplot(2,1,1)
% plot(leftFootPosition(:,3));hold on;
% plot(lFootIndexesN,lFootPeaksN,'o','MarkerSize',12)
% plot(lHeelContact,'black');plot(lHeelIndexes,lHeelPeaks,'x','MarkerSize',12);
% 




%%%% determine if left leg or right leg starts gait%%%%%%%%
%%%% also determine if movement is gait or not%%%%%%%%%%%%%


%%%% Metrics to calculate
%% Step: R,L
%% Step length : R,L
%% Stride length: R,L
%% Gait time
%% Double support phase
%% walking speed

%% Dual tasking


%% Stance phase
%% Foot progression angle
%% Others based on literature
strideLengthL = [];
strideLengthR = [];
stepLengthL = [];   %%% right heel strike - left heel strike
stepLengthR = [];   %%% left heel strike - right heel strike

stepL = length(lHeelIndexesN);
stepR = length(rHeelIndexesN);

%%% Stride length L
for i=1:length(lHeelIndexesN)-1
    
    a = leftFootPosition(lHeelIndexesN(i+1),1) - leftFootPosition(lHeelIndexesN(i),1);
    strideLengthL = [strideLengthL;a];
end

%%% Stride Length R
for i=1:length(rHeelIndexesN)-1
    a = rightFootPosition(rHeelIndexesN(i+1),1) - rightFootPosition(rHeelIndexesN(i),1);
    strideLengthR = [strideLengthR;a];
end

%%% Step length R,L

if(lHeelIndexesN(1)<rHeelIndexesN(1))
    disp(" Left foot first")
    
    %%% step length L
    for i=1:length(lHeelIndexesN)
        a = rightFootPosition(rHeelIndexesN(i),1)-leftFootPosition(lHeelIndexesN(i),1);
        stepLengthL = [stepLengthL;a];
    end
    
    %%% step length R
    for i=1:length(rHeelIndexesN)-1
        a = leftFootPosition(lHeelIndexesN(i+1),1) - rightFootPosition(rHeelIndexesN(i),1);
        stepLengthR = [stepLengthR;a];
    end
else
    disp(" Right foot first")
    %%% step length L
    for i=1:length(lHeelIndexesN)-1
        a = rightFootPosition(rHeelIndexesN(i+1),1)-leftFootPosition(lHeelIndexesN(i),1);
        stepLengthL = [stepLengthL;a];
    end
    
    %%% step length R
    for i=1:length(rHeelIndexesN)
        a = leftFootPosition(lHeelIndexesN(i),1) - rightFootPosition(rHeelIndexesN(i),1);
        stepLengthR = [stepLengthR;a];
    end
end
    
%%% Gait cycle%%%%%%%%%

gaitCycleLeft = [];
gaitCycleRight = [];

for i=1:length(lHeelIndexesN)-1
    a = (lHeelIndexesN(i+1)-lHeelIndexesN(i))/60;
    gaitCycleLeft = [gaitCycleLeft;a];
end

for i=1:length(rHeelIndexesN)-1
    a = (rHeelIndexesN(i+1)-rHeelIndexesN(i))/60;
    gaitCycleRight = [gaitCycleRight;a];
end
 

%%% Gait Speed%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
gaitSpeedLeft = (strideLengthL./gaitCycleLeft);
gaitSpeedRight = (strideLengthR./gaitCycleRight);


%% Swing phase%%%%%%%%%%%5

    
    

figure(1)
subplot(2,1,1)
plot(leftFootPosition(:,3));hold on;
plot(lFootIndexesN,lFootPeaksN,'o','MarkerSize',12)
plot(lHeelContact,'black');plot(lHeelIndexesN,lHeelPeaksN,'x','MarkerSize',12);

subplot(2,1,2);
plot(rightFootPosition(:,3));hold on;
plot(rFootIndexes,rFootPeaks,'o','MarkerSize',12)
plot(rHeelContact,'black');plot(rHeelIndexes,rHeelPeaks,'x','MarkerSize',12);



figure(2)
start = rHeelIndexesN(1);
endl = rHeelIndexesN(2);
plot(lToeContact(start:endl));hold on;plot(lHeelContact(start:endl),'r');plot(rToeContact(start:endl),'cyan')


doubleSupportL = [];
doubleSupportR = [];


    
for i=1:length(rHeelIndexesN)-1
    startTime = rHeelIndexesN(i);
    endTime = rHeelIndexesN(i+1);
    indexLToeOff = -100;
    for i=startTime:endTime
        if(lToeContact(i)==-1)
            indexLToeOff = i;
            break;
        end
    end
    double1 = indexLToeOff - startTime;
    indexLHeel = -100;
    for i=startTime:endTime
        if(lHeelContact(i)==1)
            indexLHeel = i;
        end
    end
    indexRToeOff = -100;
    for i=startTime:endTime
        if(rToeContact(i)==-1)
            indexRToeOff = i;
        end
    end
    double2 = indexRToeOff - indexLHeel;
    double = (double1 + double2)/60;
    doubleSupportR = [doubleSupportR;double];
end

for i=1:length(lHeelIndexesN) -1
    startTime = lHeelIndexesN(i);
    endTime = lHeelIndexesN(i+1);
    indexRToeOff = -100;
    for i=startTime:endTime
        if(rToeContact(i)==-1)
            indexRToeOff = i;
            break;
        end
    end
    double1 = indexRToeOff - startTime;
    indexRHeel = -100;
    for i=startTime:endTime
        if(rHeelContact(i)==1)
            indexRHeel = i;
        end
    end
    indexLToeOff = -100;
    for i=startTime:endTime
        if(lToeContact(i)==-1)
            indexLToeOff = i;
        end
    end
    double2 = indexLToeOff - indexRHeel;
    double = (double1 + double2)/60;
    doubleSupportL = [doubleSupportL;double];
end
        
disp("Mean double support L")
mean(doubleSupportL)
disp("std double support L")
std(doubleSupportL)
disp("Mean percentage of gait cycle")
mean((doubleSupportL./gaitCycleLeft)*100)
disp("std percantage of gait cycle")
std((doubleSupportL./gaitCycleLeft)*100)
        

disp("Mean double support R")
mean(doubleSupportR)
disp("std double support R")
std(doubleSupportR)
disp("Mean percentage of gait cycle")
mean((doubleSupportR./gaitCycleRight)*100)
disp("std percantage of gait cycle")
std((doubleSupportR./gaitCycleRight)*100)








