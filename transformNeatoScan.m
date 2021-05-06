function [new_x, new_y] = transformNeatoScan(x, y, origin_x, origin_y)

transformer = [1 0 origin_x; 0 1 origin_y; 0 0 1];

positions = [x; y; ones(size(x))];

new_positions = transformer * positions;

new_x = new_positions(1,:);
new_y = new_positions(2,:);
end

