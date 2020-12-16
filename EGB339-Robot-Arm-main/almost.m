%% Solution for EGB339 prac for 2020

 function [init_xy, dest_xy] = EGB339_prac_exam_group_16(dobot_sim, init_positions, dest_positions, use_vision)

% close all; clear; clc
% sync = false;
% dobot_sim = coppeliaRobot('Dobot');
% dobot_sim.startSim(sync);

if use_vision == true
   
   
    % do computer vision on initial_positions and destination_positions arguments which are images
   
    % add your code here which should include move robot arm and capture image from sim %%
   
    % you will need to calculate the xy positions and return them at the end of this section of code
    % shapes_xy needs to be
    % init_shape_xy and dest_xy need to be of size 3 x 2 and an example is shown below
    camangle = 45;
    camtarget = [0 0 -camangle camangle 0];
    dobot_sim.setJointPositions(camtarget);
    pause(4)
    workspace = dobot_sim.getImage();
   
    img = workspace;
    figure(); imshow(img)
   
    B = img(:, :, 3);
    RG = img - B;
    b = img - RG;
    segment = b(:, :, 3);
    diff = segment - b;
    greydiff = rgb2gray(diff);
    binary = imbinarize(greydiff);
    imshow(binary)
    s = regionprops('table', binary, 'Centroid', 'MajorAxisLength', 'MinorAxisLength');
    centroids = cat(1, s.Centroid);
   
    left = centroids(1,1) - 20;
    top = centroids(1,2) - 20 ;
    right = centroids(end,1) + 20;
    bottom = centroids(end,2) + 20;
   
    %     centmin = min(centroids) - 20;
    %     centmax = max(centroids) + 20;
   
    workspace(1:left,:) = 0;
    workspace(right:end,:) = 0;
   
    workspace(1:top,:) = 0;
    workspace(bottom:end,:) = 0;
   
    whole = cat(1, s.MajorAxisLength);
    [ ~ , Z ] = max(whole);
    startpoint = centroids(Z , :);
    changex = centroids(:,1) - startpoint(1);
    changey = centroids(:,2) - startpoint(2);
    change  = sqrt((changex).^2 + (changey).^2);
    withrespect = change / max(change);
    pos = withrespect;
   
    val0 = 0;
    val1 = 1;
    val2 = 2;
    val3 = 3;
    val4 = 4;
    val5 = 5;
   
    for k = 1:length(centroids)
       
        if withrespect(k) < 0.4
            pos(k) = val1;
        end
       
        if withrespect(k) > 0.4 && withrespect(k) < 0.55
            pos(k) = val2;
        end
       
        if withrespect(k) > 0.55 && withrespect(k) < 0.76
            pos(k) = val3;
        end
       
        if withrespect(k) > 0.76 && withrespect(k) < 0.85
            pos(k) = val4;
        end
       
        if withrespect(k) == 1
            pos(k) = val5;
        end
       
        if k == Z
            pos(k) = val0;
        end
    end
   
    sum1 = 0;
    sum2 = 0;
    sum3 = 0;
   
    for k = 1:length(centroids)
       
        if pos(k) == 1
            if sum1 == 1
                pos(k) = 2.5;
            else
                sum1 = sum1 + 1;
            end
        end
       
        if pos(k) == 3
            if sum2 == 1
                pos(k) = 7;
            else
                sum2 = sum2 + 1;
            end
        end
       
        if pos(k) == 4
            if sum3 == 1
                pos(k) = 6;
            else
                sum3 = sum3 + 1;
            end
        end
    end
   
    [~,label] = sort(pos);
   
    coordinates = centroids;
   
    for k = 1:length(centroids)
        coordinates(k, 1) = centroids(label(k), 1);
        coordinates(k, 2) = centroids(label(k), 2);
    end
   
    P = coordinates';
   
    Q = [20,    290;
        345,   560;
        182.5, 290;
        182.5, 560;
        345,   290;
        345,    20;
        20,    560;
        182.5,  20;
        20, 20     ];
   
    Q = Q';
   
    H = simple_homography(P,Q);
   
    shape = strings([3,1]);
   
    areas = strings([3,1]);
   
    color = strings([3,1]);
   
    centroid = double([3,1]);
   
    count = [0 0 0];
   
    img = workspace;
    figure(); imshow(img)
   
    for ii = 1:2
       
        R = img(:,:,1);
        G = img(:,:,2);
        B = img(:,:,3);
       
        r = img - B - G;
        g = img - B - R;
        b = img - R - G;
       
        segmentred = r(:, :, 1);
        segmentgreen = g(:, :, 2);
       
        diffred = segmentred - b;
        diffgreen = segmentgreen - b;
       
        greydiffred = rgb2gray(diffred);
        greydiffgreen = rgb2gray(diffgreen);
       
        binaryred = imbinarize(greydiffred);
        binarygreen = imbinarize(greydiffgreen);
       
        imshow(binaryred)
        imshow(binarygreen)
       
       
       
        if ii == 1
            mask = binaryred;
        else
            mask = binarygreen;
        end
       
        s = regionprops('table', mask, 'Centroid', 'Area', 'Circularity'); % regionprops to find centroids
        centroids = cat(1, s.Centroid); % identifying centroids
        circularity = cat(1, s.Circularity); % identifying circularity
        area = cat(1, s.Area); % identifying centroids
       
        shapes_rand = [centroids, circularity, area];
        if ii == 1
            shapes_red = shapes_rand;
            shapes_red = num2cell(shapes_red);
            numR = 6;
            shapes_red(:, numR) = {'Red'};
            shapes_green = [];
        else
            shapes_green = shapes_rand;
            shapes_green = num2cell(shapes_green);
            numG = 6;
            shapes_green(:, numG) = {'Green'};
        end
       
        shapes_workspace = [shapes_red; shapes_green];
        x = size(shapes_workspace,1);
       
        circ_val = 0.9; % circle circularity
        tri_val = 0.7; % triangle circularity
        sq_val1 = 0.7; % square circularity value 1
        sq_val2 = 0.87; % square circularity value 2
       
        for jj = 1:x
            if cellfun(@(x) x > circ_val, shapes_workspace(jj, 3))
                shapes_workspace(jj, 7) = {'Circle'};
                if cellfun(@(x) x > 2000, shapes_workspace(jj, 4))
                    shapes_workspace(jj, 5) = {'Large'};
                else
                    shapes_workspace(jj, 5) = {'Small'};
                end
            end
            if cellfun(@(x) x > sq_val1 && x < sq_val2, shapes_workspace(jj, 3))
                shapes_workspace(jj, 7) = {'Square'};
                if cellfun(@(x) x > 1500, shapes_workspace(jj, 4))
                    shapes_workspace(jj, 5) = {'Large'};
                else
                    shapes_workspace(jj, 5) = {'Small'};
                end
            end
            if cellfun(@(x) x < tri_val, shapes_workspace(jj, 3))
                shapes_workspace(jj, 7) = {'Triangle'};
                if cellfun(@(x) x > 1000, shapes_workspace(jj, 4))
                    shapes_workspace(jj, 5) = {'Large'};
                else
                    shapes_workspace(jj, 5) = {'Small'};
                end
            end
        end
       
        if ii == 2
           
            shape_coordinates = shapes_workspace(:,1:2);
           
            shape_coordinates = cell2mat(shape_coordinates);
            shape_coordinates(:,3) = 1;
           
            for z = 1:length(shapes_workspace)
                P1 = shape_coordinates(z,:)';
                positions = H * P1;
                positions = positions(1:2)/positions(3);
                xy(z,1:2) = positions;
            end
           
        end
    end
   
     init_img = imread('TestSheet1b.jpg');
     dest_img = imread('TestSheet8.jpg');
   
    for kk = 1:2
       
        if kk == 1
            imgg = init_positions;
        else
            imgg = dest_positions;
        end
       
        R = imgg(:,:,1);
        G = imgg(:,:,2);
        B = imgg(:,:,3);
       
        r = imgg - B - G;
        g = imgg - B - R;
        b = imgg - R - G;
       
        segmentred = r(:, :, 1);
        segmentgreen = g(:, :, 2);
       
        diffred = segmentred - b;
        diffgreen = segmentgreen - b;
       
        greydiffred = rgb2gray(diffred);
        greydiffgreen = rgb2gray(diffgreen);
       
        binaryred = imbinarize(greydiffred);
        binarygreen = imbinarize(greydiffgreen);
       
       
        s = regionprops('table', binaryred, 'Centroid', 'Area', 'Circularity'); % regionprops to find centroids
        centroids_r = cat(1, s.Centroid); % identifying centroids
        circularity_r = cat(1, s.Circularity); % identifying circularity
        area_r = cat(1, s.Area); % identifying centroids
       
        s2 = regionprops('table', binarygreen, 'Centroid', 'Area', 'Circularity'); % regionprops to find centroids
        centroids_g = cat(1, s2.Centroid); % identifying centroids
        circularity_g = cat(1, s2.Circularity); % identifying circularity
        area_g = cat(1, s2.Area); % identifying centroids
       
        shapes_green = [centroids_g, circularity_g, area_g];
        disp(area_g)
        shapes_green = num2cell(shapes_green);
        numG = 6;
        shapes_green(:, numG) = {'Green'};
       
        shapes_red = [centroids_r, circularity_r, area_r];
        disp(area_r)
        shapes_red = num2cell(shapes_red);
        numR = 6;
        shapes_red(:, numR) = {'Red'};
       
        shapes = [shapes_red; shapes_green];
        x = size(shapes,1);
       
        circ_val = 0.9; % circle circularity
        tri_val = 0.7; % triangle circularity
        sq_val1 = 0.7; % square circularity value 1
        sq_val2 = 0.87; % square circularity value 2
       
        for jj = 1:x
            if cellfun(@(x) x > circ_val, shapes(jj, 3))
                shapes(jj, 7) = {'Circle'};
                if cellfun(@(x) x > 46000, shapes(jj, 4))
                    shapes(jj, 5) = {'Large'};
                else
                    shapes(jj, 5) = {'Small'};
                end
            end
            if cellfun(@(x) x > sq_val1 && x < sq_val2, shapes(jj, 3))
                shapes(jj, 7) = {'Square'};
                if cellfun(@(x) x > 37000, shapes(jj, 4))
                    shapes(jj, 5) = {'Large'};
                else
                    shapes(jj, 5) = {'Small'};
                end
            end
            if cellfun(@(x) x < tri_val, shapes(jj, 3))
                shapes(jj, 7) = {'Triangle'};
                if cellfun(@(x) x > 35000, shapes(jj, 4))
                    shapes(jj, 5) = {'Large'};
                else
                    shapes(jj, 5) = {'Small'};
                end
            end
        end
       
        if kk == 1
            shapes_init = shapes;
        else
            shapes_dest = shapes;
        end
    end
   
    shapes_init(:,1:4) = [];
    shapes_dest(:,1:4) = [];
   
    %     shapes_init = char(shapes_init);
    %     shapes_dest = char(shapes_dest);
    %
    %     shapes_workspace1 = shapes_init(1,:);
    %     shapes_workspace2 = shapes_init(2,:);
    %     shapes_workspace3 = shapes_init(3,:);
    %
    %     for aa = 1:size(shapes_workspace,1)
    %         if isequal(shapes_workspace(aa,5:7),shapes_workspace1)
    %             disp('1');
    %         end
    %         if isequal(shapes_workspace(aa,5:7),shapes_workspace2)
    %             disp('2');
    %         end
    %         if isequal(shapes_workspace(aa,5:7), shapes_workspace3)
    %             disp('3');
    %         end
    %     end
   
    %     shapes_workspace1 = shapes_workspace(1:3,5:7);
    %     shapes_workspace2 = shapes_workspace(4:6,5:7);
    %     shapes_workspace3 = shapes_workspace(7:9,5:7);
    %     shapes_workspace4 = shapes_workspace(10:12,5:7);
    %
    %     if shapes_init == shapes_workspace1
    %         disp('hi');
    %     end
   
    shapes_workspace(:,1) = cellstr(num2str(xy(:,1)));
    shapes_workspace(:,2) = cellstr(num2str(xy(:,2)));
   
    %     shapes_workspace = cell2mat(shapes_workspace);
   
    init_positions = cell(3,2);
    dest_positions = cell(3,2);
   
    for mm = 1:size(shapes_init,1)
        for nn = 1:size(shapes_workspace,1)
            if isequal(char(cell2mat(shapes_init(mm,1))),char(cell2mat(shapes_workspace(nn,5)))) && ...
                    isequal(char(cell2mat(shapes_init(mm,2))),char(cell2mat(shapes_workspace(nn,6)))) && ...
                    isequal(char(cell2mat(shapes_init(mm,3))),char(cell2mat(shapes_workspace(nn,7))))
                init_positions(mm,1) = shapes_workspace(nn,1);
                init_positions(mm,2) = shapes_workspace(nn,2);
            end
        end
    end
   
    for mm = 1:size(shapes_dest,1)
        for nn = 1:size(shapes_workspace,1)
            if isequal(char(cell2mat(shapes_dest(mm,1))),char(cell2mat(shapes_workspace(nn,5)))) && ...
                    isequal(char(cell2mat(shapes_dest(mm,2))),char(cell2mat(shapes_workspace(nn,6)))) && ...
                    isequal(char(cell2mat(shapes_dest(mm,3))),char(cell2mat(shapes_workspace(nn,7))))
                dest_positions(mm,1) = shapes_workspace(nn,1);
                dest_positions(mm,2) = shapes_workspace(nn,2);
            end
        end
    end
   
    init_xy = cell2mat(init_positions);
    dest_xy = cell2mat(dest_positions);
   
   
else
   
    i = 1;
    while (i < 4)
        %% Variables for testing vs submitting
        % Given that we are given the coordinates (Vision == False)
       
        % For submission
       
        init_xy = init_positions;
        dest_xy = dest_positions;
       
        %% Execute Pick and Place
       
        % Declare the constants
        link1 = 138;
        link2 = 135;
        link3 = 147;
        link4 = 060;
        link5 = 075;
       
        % Given Coordinates
        X = init_xy(i,1);
        Y = init_xy(i,2);
        %Z = 138;
       
        %To find the robot coodinates from the measured
        Xr = X - 80;
        Yr = Y - 290;
       
        %%-------------------------Reach Target-------------------------%%
       
        % This section iterates through each start and end coordinates
        % to pick up each object and place at the desired location
        %init_xy = init_positions;
        %dest_xy = dest_positions;
       
        % Theta 1
        theta1 = atan2d(Yr , Xr);
       
        % Hovering over the target
        ReachTarget = [theta1 0 0 0  0]
        % Robot does step 1
        dobot_sim.setJointPositions(ReachTarget);
       
        pause(4);
       
        %         % Making sure the joints are reached all the time
        %         while(1)
        %             Current = dobot_sim.getJointPositions();
        %             % Current(1)
        %             if(Current(1) == ReachTarget(1)) % I'd say let's leave it like this, since its millimeters. 0.0.. won't really matter noh?
        %                 break;
        %             else
        %                 dobot_sim.setJointPositions(ReachTarget);
        %             end
        %         end
       
        %%------------------------Reach Cylinder------------------------%%
        % Height of implemented (Z) => Cylinder
        Z = 60;
       
        %Theta 3
        vertical = Z + link5 - link1;
        horizontal = sqrt(Xr^2 + Yr^2)  - link4;
        lengthB = sqrt(vertical^2 + horizontal^2);
        A = (link2^2 + link3^2 - lengthB^2) / (2 * link2 * link3);
        theta3 = 90 - atan2d(sqrt(1 - A) , A);
       
        %Theta 2
        %thetaB = asind(  (link3 *(sin(90 - theta3)))/lengthB) ;
        G = (link2^2 + lengthB^2 - link3^2)/(2*link2*lengthB);
        thetaB = atan2d(sqrt(1-G^2), G);
        thetaA = atan2d(vertical, horizontal);
        theta2 = 90 - (thetaA + thetaB);
       
        %Theta 4
        theta4 = -theta2 - theta3;
       
        %Theta 5
        theta5 = 0;
       
        % Reach cylinder
        ReachCylinder = [theta1 theta2 theta3 theta4 theta5]
        % Robot does step 2
        dobot_sim.setJointPositions(ReachCylinder);
        pause (4)
       
        % Robot turns on suction
        dobot_sim.setSuctionCup(true);
       
        pause(4)
        %%------------------------Pick Cylinder-------------------------%%
        % Pick cylinder
        PickCylinder = [theta1 0 0 0 0]
        % Robot does step 2
        dobot_sim.setJointPositions(PickCylinder);
       
       
       
        pause(4);
       
       
        %%----------------------Reach Destination-----------------------%%
        % Implementing IK for the second time
        % Move to new location function
        Xend = dest_xy(i,1);
        Yend = dest_xy(i,2);
        Zend = 60; %
       
        Xr = Xend - 80;
        Yr = Yend - 290;
       
        % Theta 1
        theta1 = atan2d(Yr , Xr);
       
        % Theta 3
        vertical = Z + link5 - link1;
        horizontal = sqrt(Xr^2 + Yr^2)  - link4;
        lengthB = sqrt(vertical^2 + horizontal^2);
        A = (link2^2 + link3^2 - lengthB^2) / (2 * link2 * link3);
        theta3 = 90 - atan2d(sqrt(1- A) , A);
       
        % Theta 2
        G = (link2^2 + lengthB^2 - link3^2)/(2*link2*lengthB);
        thetaB = atan2d(sqrt(1-G^2), G);
        thetaA = atan2d(vertical, horizontal);
        theta2 = 90 - (thetaA + thetaB);
       
        % Theta 4
        theta4 = -theta2 - theta3;
       
        % Hovering over the destination
        ReachDestination = [theta1 0 0 0 0]
        dobot_sim.setJointPositions(ReachDestination);
       
        pause(10);
       
        %%------------------------Place Cylinder-------------------------%%
       
       
        % Drop down and place cylinder
        PlaceCylinder = [theta1 theta2 theta3 theta4 theta5]
       
        % Robot keep the cyclinder
        dobot_sim.setJointPositions(PlaceCylinder);
       
        pause(4);
        % Turn suction off
        dobot_sim.setSuctionCup(false);
       
        pause(2);
        %%--------------------------Finish Drop--------------------------%%
       
        %
        FinishDrop = [theta1 0 0 0 0]
       
        dobot_sim.setJointPositions(FinishDrop);
       
        pause(4);
       
       
       
       
       
       
        i = i + 1;
       
    end
end



end
