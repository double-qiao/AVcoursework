info = load('info.mat');
for i = 2:length(info.info.rgb)
    i-1
    % find match
    matches = vl_ubcmatch(info.info.des{i-1}, info.info.des{i});
    
%     % draw match sift image
%     figure(1)
%     subplot(1,2,1)
%     img = imag2d(info.info.rgb{i-1});
%     image(img);
%     h1   = vl_plotframe(info.info.frames{i-1}(:,matches(1,:))) ; set(h1,'color','green','linewidth',1) ;
%     k = setdiff(info.info.frames{i-1}', info.info.frames{i-1}(:,matches(1,:))', 'row');
%     h2   = vl_plotframe(k') ; set(h2,'color','red','linewidth',1) ;
%     box off;
%     axis off;
%     
%     subplot(1,2,2)
%     img = imag2d(info.info.rgb{i});
%     image(img);
%     h3   = vl_plotframe(info.info.frames{i}(:,matches(2,:))) ; set(h3,'color','green','linewidth',1) ;
%     k = setdiff(info.info.frames{i}', info.info.frames{i}(:,matches(2,:))', 'row');
%     h4   = vl_plotframe(k') ; set(h4,'color','red','linewidth',1) ;
%     box off;
%     axis off;
%     filename = strcat(num2str(i-1),"_",num2str(i), '.jpg');    
%     saveas(1, filename);
    
    % caculate rotation
    sizes = 30;
    index = randperm(size(matches,2), sizes); % random select 30 match points
    %find image1
    location1 = floor(info.info.frames{i-1}(1:2, matches(1, index))); % the axis of selected points in frist image
    p = [];
    for j = 1:sizes
        temp = (location1(2,j)-1)*640 + location1(1,j);  % index of selected point j
        p = cat(1, p, info.info.point{i-1}(temp, :));
    end
    %for image2
    location2 = floor(info.info.frames{i}(1:2, matches(2, index))); % the axis of selected points in frist image
    q = [];
    for j = 1:sizes
        temp = (location2(2,j)-1)*640 + location2(1,j);  % index of selected point j
        q = cat(1, q, info.info.point{i}(temp, :));
    end
    
    % delete nan rows
    temp = cat(1, find(isnan(p(:,1))), find(isnan(q(:,1))));
    p(temp, :) = [];
    q(temp, :) = [];
    
    %caculate rotation
    H = p' * q;
    [U, S, V] = svd(H);
    R = V * U';
    p0 = mean(p,1)';
    q0 = mean(q,1)';
    T = q0 - R * p0;
    R
    T
    pause
end
