
function h = drawRobot(x,y,r)
d = r*2;
px = x-r;
py = y-r;
h = rectangle('Position',[px py d d],'Curvature',[1,1],'EdgeColor','k','FaceColor','w');
end