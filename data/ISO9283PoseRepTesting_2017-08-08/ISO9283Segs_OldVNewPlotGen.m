%% Adjust segts_orig back to origin
segts_orig_start = segts_orig{1}(1,2:4);
segts_orig_adj = segts_orig;
for n = 1:length(segts_orig)
    segts_orig_adj{n}(:,2:4) = segts_orig_adj{n}(:,2:4)-segts_orig_start;
end

%% Generate plot

figure(1);
colors = get(gca,'ColorOrder');
hold on;
grid on
axis image
axis equal
xlabel('X, mm');
ylabel('Y, mm');
zlabel('Z, mm');
view(-45,30);

for n = 1:length(segts_orig_adj)
    p1 = plot3(segts_orig_adj{1,n}(:,2),segts_orig_adj{1,n}(:,3),segts_orig_adj{1,n}(:,4),'Color',[colors(1,:)]);
end

for n = 1:length(segts_new)
    p2 = plot3(segts_new{1,n}(:,2),segts_new{1,n}(:,3),segts_new{1,n}(:,4),'Color',[colors(2,:)]);
end

legend([p1 p2],'2016 Test','2017 Test');
title({'ISO 9283 Pose Repeatability Data:';'Previous vs. Current Results'});