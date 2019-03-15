clear
clc
%% Load the training data 
office = load('office1.mat');
office = office.pcl_train;
%% Uncomment to load the test file
% office = load('office2.mat');
% office = office.pcl_test;
%%
for i = 1:length(office)        % Reading the 40 point-clouds
i 
    rgb = office{i}.Color;      % Extracting the colour data
    point = office{i}.Location; % Extracting the xyz data
    %% step 1:
    % remove irrelevant points outside the window
    cut1 = find(point(:,3)>3.5); % find out all point that z bigger than a threshold
    point(cut1,:) = NaN;         % remove this point
%    rgb(cut1,:) = 0;

    % remove professor Bob
    if (i == 27)               % remove professor Bob
        for ii = 1:480
            point((ii-1)*640+110:(ii-1)*640+320,:) = NaN;
%           rgb((ii-1)*640+110:(ii-1)*640+320,:) = 0;
        end
    end
   
    % remove flying pixels
    flyingPixels = rangesearch(point,point,0.02);  % give a search range
    counter = 0;
    for ii = 1:307200
        idx = size(flyingPixels{ii});    
        if ((1<idx(2))&&(idx(2)<=10))              % give the number of point
            point(ii,:) = NaN;
            counter = counter + 1;
        end
    end
    counter
    
    % remove data near edges
    point(1:640*5,:) = NaN;                     % remove the top edge
    point(640*476:640*480,:) = NaN;             % remove the bottom edge
    for ii = 1:475
        point(ii*640+1:ii*640+5,:) = NaN;       % remove the left edge
        point((ii+1)*640-4:(ii+1)*640,:) = NaN; % remove the right edge
    end
    
    %% step 2:
    
    %% plot
    pc = pointCloud(point, 'Color', rgb); % Creating a point-cloud variable
    figure(1)
    pcshow(pc)
    figure(2)
    imag2d(rgb) % Shows the 2D images
    pause
end
