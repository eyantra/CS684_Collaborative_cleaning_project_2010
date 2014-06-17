% /**
%  * @mainpage package Collaborative_Cleaner
%  * @author Group 11: Chinmay Vaishampayan(09305918), Jitendra Sahu(09305059), Ashutosh Patel(09305003)
%  */
% 
% /********************************************************************************
% 
%    Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in               -*- c -*-
%    All rights reserved.
%  
%    Redistribution and use in source and binary forms, with or without
%    modification, are permitted provided that the following conditions are met:
% 
%    * Redistributions of source code must retain the above copyright
%      notice, this list of conditions and the following disclaimer.
% 
%    * Redistributions in binary form must reproduce the above copyright
%      notice, this list of conditions and the following disclaimer in
%      the documentation and/or other materials provided with the
%      distribution.
% 
%    * Neither the name of the copyright holders nor the names of
%      contributors may be used to endorse or promote products derived
%      from this software without specific prior written permission.
% 
%    * Source code can be used for academic purpose. 
% 	 For commercial use permission form the author needs to be taken.
% 
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
%   BUSINESS
%   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THEfprintf(1, '\n\nStart Cleaning/traversing\n');
%   POSSIBILITY OF SUCH DAMAGE. 
% 
%   Software released under Creative Commence cc by-nc-sa licence.
%   For legal information refer to: 
%   http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode
% 
% ********************************************************************************/

% 's' = stop
% 'r' = rotate
% 'a' = send angle
% 'd' = send distance
% 'q' = reset prevI to 0
% 'n' = to send number next, send n
% 'f' = forward
% 'h' = halt
% 'm' = set motor angle to something
% 'p' = phase
% '2/4/5/6/8' = navigation
% '1/3' = soft left/right
% '7/9' = hard left right

function cleaner()
    global z; % zigbee serial object
    global rotate; % enable disable rotation
    global vid; % video object
    global frontAngle;
    global angle90;
    global prevAngle; %to know what was the prevAngle, when setMotorAngle function was used
    
    angle90 = 85;
    prevAngle = -1;
    frontAngle = 90;
    
    % initialize video object
    vid = videoinput('winvideo', 1);
    % Set the properties of the video object
    set(vid, 'FramesPerTrigger', Inf);
    set(vid, 'ReturnedColorspace', 'rgb')
    vid.FrameGrabInterval = 5;
    
    % make a serial object
    z = serial('COM17', 'BaudRate', 9600);
    rotate = 1;

    %start the video aquisition here
    start(vid)
    pause(1);
    
    % reset prevI
    fopen(z);
    sendChar('q');
    setMotorDegree(0);
    fclose(z);
    pause(1);
    fopen(z);
    
    % get current location
    [redA1, redD1, blueA1, blueD1, greenA1, greenD1] = locate();
     
    % get length of sides of rectangular area
    [lenRB, rbA, rbB] = solveTriangle(redD1, blueD1, blueA1-redA1);
    % rb means triangle between red/blue
    pToSideRB = findPerpendicular(redD1, blueD1, blueA1-redA1); %perpendicular to RB side
    pa = 90 - rbA; % angle which perpendicular on c makes with b
        
%     [lenBG, bgA, bgB] = solveTriangle(blueD1, greenD1, greenA1-blueA1);
    lenBG = 40;
    fprintf(1, '\nSides of rectangle: %d, %d\n', lenRB, lenBG);
    fprintf(1, '\nPerpendicular to side Red-Blue: %d\n', pToSideRB);
    % rotate towards RB line
    rotateAngle = int8(pa - (double(blueA1) - 90));
    fprintf(1, 'Rotation angle to reach side red-blue: %d', rotateAngle);
    if (rotateAngle > 0) % rotate left
        turn(rotateAngle-5, 'l');
    else % rotate right
        turn(-rotateAngle-5, 'r');
    end
    
    % move to rb line
    move(pToSideRB-2, 'f');
    pause(1);
    turn(angle90, 'r');
    pause(1);
    turn(angle90, 'r');
    
    fprintf(1, '\n\nAligning to Red-Blue line.\n');
    AlignRedBlue();
    fprintf(1, 'Aligned.\n');
    % get start line
    [startPoint, parts] = getStartPoint(1, lenRB);   

    fprintf(1, '\nGo to start point\n');
    goToPoint(startPoint);
    fprintf(1, '\n\nStart Cleaning/traversing\n');
    clean(parts, lenBG);
    fprintf(1, '\n\nCleaning Completed.\n');    
    stop(vid);
%     Flush all the image data stored in the memory buffer.
    flushdata(vid);
    fclose(z);
    % Clear all variables
    clear all
return

% get dimensions of area to be cleaned, distance/angle from poles
function [redA1, redD1, blueA1, blueD1, greenA1, greenD1] = locate()
% phase 1 is locate
    global z;
    global vid;
    fclose(z);
    fopen(z);
    sendChar('n');
    sendNumber(1);
    sendChar('p');
    pause(1);
    fclose(z);
    fopen(z);
    
    % assumption is left most is red, then blue, then green
    [redA1, redD1] = PositionFromRedPole();
    fprintf(1, '\nRed Pole Detected:\n\tDistance: %d\n\tAngle: %d', redD1, redA1);
    pause(2);
    [blueA1, blueD1] = PositionFromBluePole();
    fprintf(1, '\nBlue Pole Detected:\n\tDistance: %d\n\tAngle: %d', blueD1, blueA1);
    pause(2);
    [greenD1, greenA1] = PositionFromGreenPole();
    fprintf(1, '\nGreen Pole Detected:\n\tDistance: %d\n\tAngle: %d', greenD1, greenA1);
    stop(vid);
    flushdata(vid);
    start(vid);
    pause(2);
    end
return

% get distance/angle from red pole
function [redAngle, redDistance] = PositionFromRedPole()
    global rotate;
    global vid;
    done = 0;
    distance = 1;
    angle = 500;
    while (done ~= 2)
        
        % Get the snapshot of the current frame
        data = getsnapshot(vid);

        if (rotate == 1) 
            startRotation();
        else
            stopRotation();
        end
            
        
        % Now to track red objects in real time
        % we have to subtract the red component 
        % from the grayscale image to extract the red components in the image.
        diff_im = imsubtract(data(:,:,1), rgb2gray(data));

        % Convert the resulting grayscale image into a binary image.
        diff_im = im2bw(diff_im,0.18);

        % Remove all those pixels less than 300px
        diff_im = bwareaopen(diff_im,300);

        % Label all the connected components in the image.
        bw = bwlabel(diff_im, 8);

        % Here we do the image blob analysis.
        % We get a set of properties for each labeled region.
        stats = regionprops(bw, 'basic');
        % Display the image
        imshow(data)
    %     largestStat = stat(1);
        %This is a loop to bound the red objects in a rectangular box.
        for object = 1:length(stats)
            bb = stats(object).BoundingBox;
            area = stats(object).Area;
            if area > 10000
                x = bb(1);
                w = bb(3);
                xCenter = 320;
                precision = 5;
                if ((x < (xCenter-precision)) && ((x+w) > (xCenter+precision)))
                    rectangle('Position',bb,'EdgeColor','r','LineWidth',2);
                    stopRotation();
                    flushReadBuffer();
                    sendDistanceEnable();
                    while (distance<10 || distance>82)
                        distance = uint8(readNumber());
                        done = 1;
                    end
                    pause(1);
                    sendAngleEnable();
                    flushReadBuffer();
                    while (angle > 180)
                        angle = uint8(readNumber());
                        done = 2;
                    end
                end
            end
        end
    end
    rotate = 1;
    redDistance = double(distance);
    redAngle = double(angle);
return

% get distance/angle from blue pole
function [blueAngle, blueDistance] = PositionFromBluePole()
    global rotate;
    global vid;
    done = 0; % done=0 menas no data recvd, done=1 means distance recvd and done=2 means angle also recvd
    distance = 1;
    angle = 500;
    while (done ~= 2)
        
        % Get the snapshot of the current frame
        data = getsnapshot(vid);

        if (rotate == 1) 
            startRotation();
        else
            stopRotation();
        end
            
        
        % Now to track red objects in real time
        % we have to subtract the red component 
        % from the grayscale image to extract the red components in the image.
        diff_im = imsubtract(data(:,:,3), rgb2gray(data));

        % Convert the resulting grayscale image into a binary image.
        diff_im = im2bw(diff_im,0.18);

        % Remove all those pixels less than 300px
        diff_im = bwareaopen(diff_im,300);

        % Label all the connected components in the image.
        bw = bwlabel(diff_im, 8);

        % Here we do the image blob analysis.
        % We get a set of properties for each labeled region.
        stats = regionprops(bw, 'basic');
        % Display the image
        imshow(data)
    %     largestStat = stat(1);
        %This is a loop to bound the red objects in a rectangular box.
        for object = 1:length(stats)
            bb = stats(object).BoundingBox;
            area = stats(object).Area;
            if area > 10000
                x = bb(1);
                w = bb(3);
                xCenter = 320;
                precision = 5;
                if ((x < (xCenter-precision)) && ((x+w) > (xCenter+precision)))
                    rectangle('Position',bb,'EdgeColor','r','LineWidth',2);
                    stopRotation();
                    flushReadBuffer();
                    sendDistanceEnable();
                    while (distance<10 || distance>82)
                        distance = uint8(readNumber());
                        done = 1;
                    end
                    pause(1);
                    sendAngleEnable();
                    flushReadBuffer();                     
                    while (angle > 180)
                        angle = uint8(readNumber());
                        done = 2;
                    end
                end
            end
        end
    end
    rotate = 1;
    blueDistance = double(distance);
    blueAngle = double(angle);
    % Both the loops end here.
return

% get distance/angle from green pole
function [greenAngle, greenDistance] = PositionFromGreenPole()
    global rotate;
    global vid;
    done = 0;
    distance = 1;
    angle = 500;
    while (done ~= 2)
        if (rotate == 1)
            startRotation();
        else
            stopRotation();
        end

        data = getsnapshot(vid);
        dim = size(data); %stores dimension of RGB matrix
        %Color segmentation code
        temp = zeros(dim(1), dim(2));
        for i=1:dim(1)
            for j=1:dim(2)
                if (data(i, j, 1) <= 110 && data(i, j, 2) < 255 &&  ...
                    data(i, j, 2) > 120 && data(i, j, 3) <=140)
                    temp(i, j) = 255;
                end
            end
        end
        
        diff_im = bwareaopen(temp, 80);

        bw = bwlabel(diff_im, 8);
        stats = regionprops(bw, 'basic');
        imshow(data);
        for object = 1:length(stats)
            bb = stats(object).BoundingBox;
            area = stats(object).Area;
            if area > 10000
                x = bb(1);
                w = bb(3);
                xCenter = 320;
                precision = 5;
                if ((x < (xCenter-precision)) && ((x+w) > (xCenter+precision)))
                    rectangle('Position',bb,'EdgeColor','r','LineWidth',2);
                    stopRotation();
                    flushReadBuffer();
                    sendDistanceEnable();
                    while (distance<10 || distance>82)
                        distance = uint8(readNumber());
                        done = 1;
                    end
                    pause(1);
                    flushReadBuffer();
                    sendAngleEnable();
                    while (angle > 180)
                        angle = uint8(readNumber());
                        done = 2;
                    end
                end
            end
        end
    end
    rotate = 1;
    greenDistance = double(distance);
    greenAngle = double(angle);
return

% check if red pole is there at specified angle or not.
function isRedPole = DetectRedPole(angle)
    global vid;
    isRedPole = 0;
    % check if angle was passed
    if ((nargin<1) || isempty(angle))
        angle = 0;
    end
    setMotorDegree(angle);

    % Get the snapshot of the current frame
    data = getsnapshot(vid);

    % Now to track red objects in real time
    % we have to subtract the red component 
    % from the grayscale image to extract the red components in the image.
    diff_im = imsubtract(data(:,:,1), rgb2gray(data));

    % Convert the resulting grayscale image into a binary image.
    diff_im = im2bw(diff_im,0.18);

    % Remove all those pixels less than 300px
    diff_im = bwareaopen(diff_im,300);

    % Label all the connected components in the image.
    bw = bwlabel(diff_im, 8);

    % Here we do the image blob analysis.
    % We get a set of properties for each labeled region.
    stats = regionprops(bw, 'basic');
    % Display the image
    imshow(data)
    %This is a loop to bound the red objects in a rectangular box.
    for object = 1:length(stats)
        bb = stats(object).BoundingBox;
        area = stats(object).Area;
        if area > 15000
            x = bb(1);
            w = bb(3);
            xCenter = 320;
            precision = 5;
            if ((x < (xCenter-precision)) && ((x+w) > (xCenter+precision)))
                rectangle('Position',bb,'EdgeColor','r','LineWidth',2);
                isRedPole = 1;
            else
                isRedPole = 0;
            end
        end
    end
return

% check if blue pole is there at specified angle or not.
function isBluePole = DetectBluePole(angle)
    global vid;
    isBluePole = 0;
    % check if angle was passed
    if ((nargin<1) || isempty(angle))
        angle = 0;
    end
    setMotorDegree(angle);
    
    % Get the snapshot of the current frame
    data = getsnapshot(vid);

    % Now to track red objects in real time
    % we have to subtract the red component 
    % from the grayscale image to extract the red components in the image.
    diff_im = imsubtract(data(:,:,3), rgb2gray(data));

    % Convert the resulting grayscale image into a binary image.
    diff_im = im2bw(diff_im,0.18);

    % Remove all those pixels less than 300px
    diff_im = bwareaopen(diff_im,300);

    % Label all the connected components in the image.
    bw = bwlabel(diff_im, 8);

    % Here we do the image blob analysis.
    % We get a set of properties for each labeled region.
    stats = regionprops(bw, 'basic');
%         statsLarge = find(stats.Area>15000);
    % Display the image
    imshow(data)
%     largestStat = stat(1);
    %This is a loop to bound the red objects in a rectangular box.
    for object = 1:length(stats)
        bb = stats(object).BoundingBox;
        area = stats(object).Area;
        if area > 15000
            x = bb(1);
            w = bb(3);
            xCenter = 320;
            precision = 5;
            if ((x < (xCenter-precision)) && ((x+w) > (xCenter+precision)))
                rectangle('Position',bb,'EdgeColor','r','LineWidth',2);
                isBluePole = 1;
            else
                isBluePole = 0;
            end
        end
    end
return

% check if green pole is there at specified angle or not.
function isGreenPole = DetectGreenPole(angle)
    global vid;
    isGreenPole = 0;
    % check if angle was passed
    if ((nargin<1) || isempty(angle))
        angle = 0;
    end
    setMotorDegree(angle);

    data = getsnapshot(vid);
    dim = size(data); %stores dimension of RGB matrix
    %Color segmentation code
    temp = zeros(dim(1), dim(2));
    for i=1:dim(1)
        for j=1:dim(2)
            if (data(i, j, 1) <= 110 && data(i, j, 2) < 255 &&  ...
                data(i, j, 2) > 120 && data(i, j, 3) <=140)
                temp(i, j) = 255;
            end
        end
    end

    diff_im = bwareaopen(temp, 80);

    bw = bwlabel(diff_im, 8);
    stats = regionprops(bw, 'basic');
    imshow(data);
    for object = 1:length(stats)
        bb = stats(object).BoundingBox;
        area = stats(object).Area;
        if area > 15000
            x = bb(1);
            w = bb(3);
            xCenter = 320;
            precision = 5;
            if ((x < (xCenter-precision)) && ((x+w) > (xCenter+precision)))
                rectangle('Position',bb,'EdgeColor','r','LineWidth',2);
                isGreenPole = 1;
            else
                isGreenPole = 0;
            end
        end
    end
return

% align the bot on red/blue line, with blue pole on left/red on right
function AlignRedBlue()
    stopRotation();
    setMotorDegree(0);    
    aligned = 0;
    while (aligned==0)
        bluePoleDetected = 0;
        redPoleDetected = 0;
        continueWithBlue=0;
        while (bluePoleDetected==0)
            for i = 0:2:90
                blueAngle=i;
                bluePoleDetected = DetectBluePole(i);
                if (90-i <= 2 && bluePoleDetected==0)
                    turn(10, 'l');
                end
                if (bluePoleDetected == 1)
                    if (blueAngle > 12)
                        turn(blueAngle - 12, 'r');
                        continueWithBlue=1;
                    end 
                    fprintf(1, '\nAngle of Blue Pole: %d', blueAngle);
                    break;
                end
                pause(.3);
            end
        end
        if (continueWithBlue == 1)
            continue;
        end
        pause(1);
        continueWithBlue = 0;
        while (redPoleDetected==0)
            for i = 130:2:180
                redAngle=i;
                redPoleDetected = DetectRedPole(i);
                if ((180-i <= 2) && redPoleDetected==0)
                    continueWithBlue=1;
                    move(5, 'b');
                end
                if (redPoleDetected == 1)
                    fprintf(1, '\nAngle of Red Pole: %d', redAngle);
                    break;
                end
                pause(.3);
            end
        end
        if (continueWithBlue == 1)
            continue;
        end
        if ((blueAngle < 10) && (redAngle>170))
            aligned = 1;
            % if not aligned, turn it gradually
        elseif ((90-blueAngle) - (redAngle-90) > 5)
            %turn towards red (right)
            turn(5, 'l');
            pause(1);
            move(3, 'f');
        elseif ((90-blueAngle) - (redAngle-90) < -5)
            % turn towards blue (left)
            turn(5, 'r');
            pause(1);
            move(3, 'f');
        else
            move(2, 'f');
        end
    end 
return

% align the bot on green/blue line, with blue pole on left/green on right
function AlignGreenBlue()
    stopRotation();
    setMotorDegree(0);    
    aligned = 0;
    while (aligned==0)
        bluePoleDetected = 0;
        greenPoleDetected = 0;
        continueWithBlue=0;
        while (bluePoleDetected==0)
            for i = 0:2:90
                blueAngle=i;
                bluePoleDetected = DetectBluePole(i);
                if (90-i <= 2 && bluePoleDetected==0)
                    turn(10, 'l');
                end
                if (bluePoleDetected == 1)
                    if (blueAngle > 12)
                        turn(blueAngle - 15, 'r');
                        continueWithBlue=1;
                    end % checked for 18, and turned wrt 15 because if we rotate for less than 3, then it doesn't really count
                    fprintf(1, '\nAngle of Blue Pole: %d', blueAngle);
                    break;
                end
                pause(.3);
            end
        end
        if (continueWithBlue == 1)
            continue;
        end
        pause(1);
        continueWithBlue = 0;
        while (greenPoleDetected==0)
            for i = 130:2:180
                greenAngle=i;
                greenPoleDetected = DetectGreenPole(i);
                if ((180-i <= 2) && greenPoleDetected==0)
                    continueWithBlue=1;
                    move(5, 'b');
                end
                if (greenPoleDetected == 1)
                    fprintf(1, '\nAngle of Green Pole: %d', greenAngle);
                    break;
                end
                pause(.3);
            end
        end
        if (continueWithBlue == 1)
            continue;
        end
        if ((blueAngle < 10) && (greenAngle>170))
            aligned = 1;
            % if not aligned, turn it gradually
        elseif ((90-blueAngle) - (greenAngle-90) > 5)
            %turn towards green (right)
            turn(5, 'l');
            pause(1);
            move(3, 'f');
        elseif ((90-blueAngle) - (green-90) < -5)
            % turn towards blue (left)
            turn(5, 'r');
            pause(1);
            move(3, 'f');
        else
            move(2, 'f');
        end
    end 
return

% start motor rotation
function startRotation()
    global rotate;
    global z;
    global prevAngle;
    fprintf(z, '%c', 'r');
    rotate = 1;
    prevAngle = -1; % reset prevAngle of motor because we don't know now what will be the angle.
return

% stop motor rotation
function stopRotation()
    global rotate;
    global z;
    global prevAngle;
    fprintf(z, '%c', 's');
    rotate = 0;
    prevAngle = -1; % reset prevAngle of motor because we don't know now what will be the angle.
return

% instruct bot to send distance, when bot is is waiting for this signal
function sendDistanceEnable()
    global z;
    fprintf(z, '%c', 'd');
return

% instruct bot to send angle
function sendAngleEnable()
    global z;
    fprintf(z, '%c', 'a');
return

% instruct bot to send the distance immediately
function distance = getDistance()
    global z;
    flushReadBuffer();    
    fclose(z);
    fopen(z);
    sendChar('v');
    distance = int8(readNumber());
return

% send a number to bot
function sendNumber(x)
    global z;
    fwrite(z, x);
return

% send a character to bot
function sendChar(x)
    global z;
    fprintf(z, '%c', x);
return

% flush/empty read buffer of communication object
function flushReadBuffer()
    global z;
    if (z.BytesAvailable > 0 )
        fread(z, z.BytesAvailable); 
    else
        fclose(z);
        fopen(z);
    end
return

% recv a character from bot
% function recv = readChar()
%     global z;
%     recv = fscanf(z, '%c', 1);
% return

% read a number sent from bot
function [recv] = readNumber()
    global z;
    recv = fread(z, 1);
return

% turn the bot by specified angle, in specified side ('l'=left, 'r'=right)
function turn(angle, side)
    sendChar('n');
    sendNumber(angle);
    if (side == 'l') % L-left
        sendChar('7');
    elseif (side == 'r') % R-right
        sendChar('9');
    end
    if (angle<100)
        pause(2);
    else
        pause(3);
    end
return

% move the bot through some distance, f/b=forward/backward
function move(distance, direction)
    sendChar('n');
    sendNumber(distance);
    if (direction == 'f') % L-left
        sendChar('8');
    elseif (direction == 'b') % R-right
        sendChar('2');
    end
    waitTime = distance/40 + 1;
    if (distance < 5)
        waitTime = 0.2;
    end
    pause(waitTime);
return

% set motor to specified degree
function setMotorDegree(angle)
    global prevAngle;
    sendChar('n');
    sendNumber(angle);
    sendChar('m');
    pause(.3);
    
    if ((prevAngle == -1) || (prevAngle-angle > 5) || (prevAngle-angle < -5))
        pause(1);
    end
    prevAngle = angle;
return

% solve sides/angles of triangle based on 2 sides/1 angle
function [c, A, B] = solveTriangle(a, b, C)
      
    a = double(a);
    b = double(b);
    C = double(C);
    
    c = sqrt((a*a) + (b*b) - double(2*a*b*cosd(C)));
       
    A = asind((a*sind(C))/c);
    B = 180 - A - C; 
return

function p = findPerpendicular(a, b, C)
    a = double(a);
    b = double(b);
    C = double(C);
    [c, A, B] = solveTriangle(a, b, C);
    p = a * sind(B);
    p = double(p);
return

function [bots, parts] = divideArea(len)
    parts = floor(len/18); % 18 approx dia of bot
    bots = floor(parts/4); %4 parts per bot
    if (bots <= 1)
        bots = 2;
    end
return

function [startPoint, parts] = getStartPoint(botNumber, len)
    [bots, parts] = divideArea(len);
    startPoint = botNumber * len / bots;
    if (startPoint>len)
        startPoint=len;
    end
return

% go at a point at a distance specified from ble pole
function goToPoint(distanceFromBlue)
    global z;
    global angle90;
    fprintf(1, '\nGo to a point at a distance %d cm from blue pole\n', distanceFromBlue);
    setMotorDegree(88);
    turn(angle90, 'l');
    pause(1);
    distance = getDistance();
    while (abs(distance-distanceFromBlue)>5)
        setMotorDegree(88);
        fclose(z);
        fopen(z);
        distance = getDistance();
        if (distance < distanceFromBlue)
            move(1, 'b');
        elseif (distance > distanceFromBlue)
            move(1, 'f');
        else
            break;
        end
        
    end
    turn(angle90, 'r');
    pause(2);
return

% parts or no. of strips to cover (18cm) and length of each strip
function clean(parts, len)
    curPart = 1;
    atRB=1; % if atRB is true then bot is on RB line otherwise BG line
    global angle90;
    while (curPart <= parts)
        move(len, 'f');
        pause(3);
        atRB = ~atRB;
        if (curPart ~= parts)
            if (atRB)
                turn(angle90, 'r');
                move(13, 'f');
                pause(1);
                turn(angle90, 'r');
                AlignRedBlue();
            else
                turn(angle90, 'l');
                move(13, 'f');
                pause(1);
                turn(angle90, 'l');
                AlignGreenBlue();
            end
        end
        curPart = curPart + 1;
    end
return
