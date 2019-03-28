%% Load the training data 
office = load('office1.mat');
office = office.pcl_train;
%% Uncomment to load the test file
% office = load('office2.mat');
% office = office.pcl_test;
%%

info = {};

for i = 1:length(office) % Reading the 40 point-clouds
i
    rgb = office{i}.Color; % Extracting the colour data
    point = office{i}.Location; % Extracting the xyz data
    
    % show original image and point cloud
    figure(1)
    pc = pointCloud(point, 'Color', rgb);
    pcshow(pc)
    figure(2)
    imshow(imag2d(rgb))
    
    %1.3 remove flying pixels
    fy = rangesearch(point,point,0.02);
    for j = 1:(640*480)
        si = size(fy{j});
        if  si(2) <11
            point(j,:) = NaN;
            rgb(j,:) = NaN;
        end
    end 
    
    
    
    %1.4 deformed data near the edges of the image
    for j = 4:476
        point(j*640+1:j*640+5,:) = NaN;
        rgb(j*640+1:j*640+5,:) = 0;
        point(j*640-4:j*640,:) = NaN; 
        rgb(j*640-4:j*640,:) = 0; 
    end
    point(1:640*5,:) = NaN;
    rgb(1:640*5,:) = 0;
    point(640*475+1:640*480,:) = NaN;
    rgb(640*475+1:640*480,:) = 0;  
    
    
%     %1.1 irrelevant points far away
    z_threshold = find(point(:,3) > 3.5);
%     temp =  3.5 ./ point(z_threshold,3);
%     point(z_threshold,1) = point(z_threshold,1) .* temp;
%     point(z_threshold,2) = point(z_threshold,2) .* temp;
    point(z_threshold,3) = 3.5;
    % rgb(z_threshold,:) = 0;
    %x_threshold = find(point(:,1) > 3.5 | point(:,1) < -3.5);
    %point(x_threshold,1) = 3.5;
    %y_threshold = find(point(:,2) > 3.5 | point(:,2) < -3.5);
    %point(y_threshold,2) = 3.5;
     
    
    %1.2 a person in one frame
    % remove professor Bob
    if (i == 27)               % remove professor Bob
        for ii = 1:480
            point((ii-1)*640+110:(ii-1)*640+320,:) = NaN;
            rgb((ii-1)*640+110:(ii-1)*640+320,:) = 0;
        end
    end     
    
    %1.5 noisy ripples
    
    % 2 
    pc = pointCloud(point, 'Color', rgb); % Creating a point-cloud variable
    new_image = imag2d(rgb); % Shows the 2D images
    grey_image = rgb2gray(new_image);
    [frames, des] = vl_sift(single(grey_image));

    info.rgb{i} = rgb;
    info.point{i} = point;
    info.frames{i} = frames;
    info.des{i} = des;
    
    bim = imbinarize(grey_image,0);
    % filename = strcat("binary/",num2str(i), '.png');
    % imwrite(im2double(bim), filename);
    
    % show cleaned point cloud
    figure(3) 
    pcshow(pc)
    % show binary image
    figure(4);
    imshow(bim);
    % show SIFT
    figure(5);
    image(new_image);
    h = vl_plotframe(frames);
    set(h,'color','red','linewidth',1);
    
    
    pause
end
save info.mat info