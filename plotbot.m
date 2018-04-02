function plotrobot(x, y, theta, color, filled, fillcolor, scale)

persistent h;

po = [ 0.5  0; ...
      -0.5  0.35; ...
      -0.5 -0.35]';

R = [cos(theta), -sin(theta);
     sin(theta),  cos(theta)];
 
p = bsxfun(@plus, R*po, [x; y]);


if ~isempty(h)
    try
        delete(h);
    end
end
h(1) = line(p(1,[1:3,1]), p(2,[1:3,1]), 'Color', color);
h(2) = fill(p(1,:),p(2,:),fillcolor);
