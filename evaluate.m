pc = pcread('pc1_40(without27)_copy.ply');
% figure(4)
% pcshow(pc)

% find three planes by cluster
clstr_idx = kmeans(pc.Location, 3);
clster1 = find(clstr_idx == 1);
pc1 = pointCloud(pc.Location(clster1, :), 'Color', pc.Color(clster1, :));
% figure(1);
% pcshow(pc1);
clster2 = find(clstr_idx == 2);
pc2 = pointCloud(pc.Location(clster2, :), 'Color', pc.Color(clster2, :));
% figure(2);
% pcshow(pc2);
clster3 = find(clstr_idx == 3);
pc3 = pointCloud(pc.Location(clster3, :), 'Color', pc.Color(clster3, :));
% figure(3);
% pcshow(pc3);

% fit three planes, ax+by+cz+d=0
maxdist = 0.1;
[model1,in1,out1] = pcfitplane(pc1,maxdist);
plane1 = select(pc1, in1);
figure(1);
pcshow(plane1);
[model2,in2,out2] = pcfitplane(pc2,maxdist);
plane2 = select(pc2, in2);
figure(2);
pcshow(plane2);
[model3,in3,out3] = pcfitplane(pc3,maxdist);
plane3 = select(pc3, in3);
figure(3);
pcshow(plane3);

% caculate angles of each planes
angle12 = acos(dot(model1.Normal, model2.Normal) / (norm(model1.Normal) ...
    * norm(model2.Normal)));
angle23 = acos(dot(model2.Normal, model3.Normal) / (norm(model2.Normal) ...
    * norm(model3.Normal)));
angle13 = acos(dot(model1.Normal, model3.Normal) / (norm(model1.Normal) ...
    * norm(model3.Normal)));

% caculate distance of each planes
point1 = mean(plane1.Location,1);
point2 = mean(plane2.Location,1);
point3 = mean(plane3.Location,1);
dist12 = dist(point1,point2);
dist13 = dist(point1,point3);
dist23 = dist(point2,point3);