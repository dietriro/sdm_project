


dx = [];
dy = [];
ox = [];
oy = [];

for x = 1:186
    for y = 1:130
   
        hold on
        if map(x, y) > 253
            ox = [ox, x];
            oy = [oy, y];
%         elseif map(x, y) > 0
%             dx = [dx, x];
%             dy = [dy, y];
        end
        
    end
end

% plot(dx, dy, 'bx')
plot(ox, oy, 'rx')

