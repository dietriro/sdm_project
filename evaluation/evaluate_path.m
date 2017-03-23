


path_bad = [[9 2]; [7 2]; [6 1]; [5 0]; [4 3]; [3 3]; [2 1]; [0 0]];
path_good = [[9 2]; [8 6]; [7 7]; [6 8]; [3 8]; [1 7]; [0 0]];
path_best = [[9 2]; [8 6]; [7 8]; [3 8]; [1 7]; [0 0]];

% f1 = figure;
figure('units','normalized','position',[.1 .1 .6 .9])
hold on

axis([0 10 0 10])
set(gca,'FontSize',14)

map = [[0 0 0 0 0 0 0 0 0 0];
 [0 1 1 1 1 1 0 0 0 0];
 [0 0 0 0 1 1 1 0 0 0];
 [1 1 1 0 1 1 1 1 0 0];
 [0 0 0 0 1 1 1 1 0 0];
 [0 1 1 1 1 1 1 1 0 0];
 [0 0 0 1 1 1 1 0 0 0];
 [1 1 0 1 1 1 0 0 0 0];
 [0 0 0 0 0 0 0 0 0 0];
 [0 0 0 0 0 0 0 0 0 0]];

occ = [];

for x = 1:10
    plot([x, x], [0, 10], 'black')
    plot([0, 10], [x, x], 'black')
    for y = 1:10
        if map(x,y) > 0
            px = [x-1, x, x, x-1];
            py = [y-1, y-1, y, y];
            fill(px,py,[0.4 0.4 0.4])
            occ = [occ; x-1, y-1];
        end
    end
end

xlabel('X')
ylabel('Y')



p1 = plot(path_good(:,1), path_good(:,2), '-go', 'LineWidth', 2, 'MarkerSize', 10)
p2 = plot(path_bad(:,1), path_bad(:,2), '-ro', 'LineWidth', 2, 'MarkerSize', 10)
p3 = plot(path_best(:,1), path_best(:,2), '--bo', 'LineWidth', 2, 'MarkerSize', 10)


legend([p2; p1; p3], [string('ARC-Theta* (max-rate=1.9)'); string('ARC-Theta* (max-rate=0.8)'); string('Adapted ARC-Theta* (C=1)')], 'FontSize',14)











