function P_world = RedSmartiesDetector(image, K_ext)
    % Examples
    %image = imread([pwd, '\images\Color threshold.png']);
    %K_ext = [0, 1, 0, 0.138; 0, 0, -1, 0; -1, 0, 0, 0.143; 0, 0, 0, 1];
    
    % Defining the red color
    lowercolor = [223, 60, 71] / 255;
    uppercolor = [18, 255, 255] / 255;
    
    % Finding the red smarties
    [~, coord] = filterColorAndGetCoordinates(image, lowercolor, uppercolor);
    % Uses clustering to define the red smarties pixel as an object
    P_coord = getClusterCenters(coord, 10, 200);
    
    % This is the intrinsic matrix for the camera. This is taken from the above
    % function from the Camera Calibrator App
    K_int = [729.0639, 0, 336.3515; 0, 728.0372, 246.7135; 0, 0, 1];
    
    % Show the image with the canter of mass points and principal point
    figure;
    imshow(image);
    hold on;
    plot(P_coord(:, 2), P_coord(:, 1), 'o', 'color', 'b',LineWidth=3);
    hold off;

    % Pre-allocating the sizes and values of the vectors
    P_pixel = [P_coord(:, 2)'; P_coord(:, 1)'; ones(1, size(P_coord, 1))];
    P_cam = zeros(4, 1);
    P_world = zeros(4, 1);
    
    % Finding the world coordinates from the pixel coordinates
    for index = 1:size(P_pixel, 2)
        % Finding the points in the camera coordinate
        P_cam(1:3, index) = (K_int \ P_pixel(:, index)) * (K_ext(3, 4) - 0.005);
        P_cam(4, index) = 1;

        % The coordinates in the extrinsic matrix does not point in the
        % same direction due to the assignment
        P_cam([1, 3], index) = P_cam([3, 1], index);
        P_cam(2, index) = -P_cam(2, index);
        
        % Finding the points in the world coordinate
        P_world(:, index) = K_ext * P_cam(:, index);
        P_world(:, index) = P_world(:, index) / P_world(4, index);
    end

    function [result, coordinates] = filterColorAndGetCoordinates(image, lowerColor, upperColor)
        % Convert the image to HSV color space
        hsvImage = rgb2hsv(image);
    
        % Create a binary mask to filter out colors outside the specified range
        mask = (hsvImage(:,:,1) >= lowerColor(1) | hsvImage(:,:,1) <= upperColor(1)) & ...
               (hsvImage(:,:,2) >= lowerColor(2) & hsvImage(:,:,2) <= upperColor(2)) & ...
               (hsvImage(:,:,3) >= lowerColor(3) & hsvImage(:,:,3) <= upperColor(3));
        
        % Use opening and closing to isolate the objects - added to avoid
        % noises and filling out the gaps. The radius of the disk is set to 5.
        se = strel('disk', 5);
        mask = imopen(mask, se);
        mask = imclose(mask, se);
    
        % Apply the mask to the original image
        result = bsxfun(@times, image, uint8(mask));
    
        % Find the colored coordinates (not blacked out)
        [rows, cols] = find(mask);
        coordinates = [rows, cols];
    end
    
    function clusterCenters = getClusterCenters(coordinates, eps, minSamples)
        % Perform DBSCAN clustering on the coordinates
        idx = dbscan(coordinates, eps, minSamples);
        
        % Find unique labels, excluding noise points (-1)
        uniqueLabels = unique(idx);
        uniqueLabels(uniqueLabels == -1) = [];
        clusterCenters = zeros(size(uniqueLabels, 1), 2);
    
        for label = 1:size(uniqueLabels, 1)
            clusterPoints = coordinates(idx == label, :);
    
            % Calculate the mean of cluster points to find the center
            center = mean(clusterPoints, 1);
            clusterCenters(label, :) = center;
        end
    end
end
