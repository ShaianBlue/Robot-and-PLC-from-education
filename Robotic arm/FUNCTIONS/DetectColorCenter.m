function [centers, avgRGB, label] = DetectColorCenter(image)
    % Ensure the input image is in uint8 format
    if ~isa(image, 'uint8')
        image = im2uint8(image);
    end

    % Pad the image with a white border
    borderSize = 5;
    paddedImage = padarray(image, [borderSize, borderSize], 255); % 255 represents white

    % Filter the padded image
    fk = fspecial('average', [borderSize, borderSize]);
    for i = 1:10
        filteredImage = imfilter(paddedImage, fk);
        paddedImage = filteredImage;
    end

    % Crop the filtered image to its original size
    croppedImage = filteredImage(borderSize + 1:end - borderSize, borderSize + 1:end - borderSize, :);

    % Convert the grayscale image to binary using adaptive thresholding

    imagegrad = std(double(croppedImage),0,3);
    imagegrad = uint8(imagegrad/max(imagegrad,[],'all')*255);
    threshold = graythresh(imagegrad);
    binaryImg = ~imbinarize(imagegrad, threshold);

    % for i = 1:3
    %     threshold = graythresh(croppedImage(:,:,i));
    %     binaryImg(:,:,i) = imbinarize(croppedImage(:,:,i), threshold);
    % end
    % 
    % binaryImg = all(binaryImg,3);

    % Apply erosion and dilation to remove noise

    % se = strel('disk', 10); % Define the structuring element (adjust the size as needed)
    % binaryImg = imdilate(binaryImg, se); % Dilation
    % binaryImg = imerode(binaryImg, se); % Erosion

    % Find contours in the binary image
    contours = bwboundaries(binaryImg);

    % Initialize storage for average RGB values and centers
    avgRGB = zeros(length(contours), 3);
    centers = zeros(length(contours), 2);

    for i = 1:length(contours)
        if sum(size(contours{i}))>1500
            continue
        end
        
        % Calculate the region mask for each contour
        boundary = contours{i};
        mask = poly2mask(boundary(:, 2), boundary(:, 1), size(binaryImg, 1), size(binaryImg, 2));

        % Extract the RGB channels within the masked region
        maskedR = croppedImage(:, :, 1) .* uint8(mask);
        maskedG = croppedImage(:, :, 2) .* uint8(mask);
        maskedB = croppedImage(:, :, 3) .* uint8(mask);

        % Compute the average RGB values
        avgR = mean(maskedR(mask > 0));
        avgG = mean(maskedG(mask > 0));
        avgB = mean(maskedB(mask > 0));

        avgRGB(i, :) = [avgR, avgG, avgB];

        % Calculate region properties
        s = regionprops(mask, 'Centroid', 'MajorAxisLength', 'MinorAxisLength');

        % Check if any centroids were found
        if ~isempty(s)
            centers(i, :) = s(1).Centroid;
        end
    end

    zerolist = all(centers == 0,2);
    avgRGB(zerolist, :) = [];
    centers(zerolist, :) = [];
    % coor = round(centers);
    % whitelist = diag(binaryImg(coor(:,2),coor(:,1))) == 1;
    % avgRGB(whitelist, :) = [];
    % centers(whitelist, :) = [];

    % Initialize label array
    label = zeros(size(centers, 1), 1);
    v = 1:length(centers);
    i = 1;
    while ~isempty(v)
        list = mean(abs(avgRGB(v, :) - avgRGB(v(1), :)), 2) < 20; % Adjust threshold as needed
        label(v(list)) = i;
        v(list) = [];
        i = i + 1;
    end

    % Display the detected circles on the original image
    imshow(croppedImage)
    hold on;
    for i = 1:length(centers)
        plot(centers(i, 1), centers(i, 2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
        text(centers(i, 1) + 5, centers(i, 2) - 5, sprintf('(%d,%d)', round(centers(i, 1)), round(centers(i, 2))), 'Color', 'blue', 'FontSize', 8);
    end
    hold off;
end
