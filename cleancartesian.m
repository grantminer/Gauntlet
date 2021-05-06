function [x, y] = cleancartesian(r, theta)

indices = find(r);

theta_clean = theta(indices);
r_clean = r(indices);

x = r_clean.*cos(theta_clean)-0.084;
y = r_clean.*sin(theta_clean);
end