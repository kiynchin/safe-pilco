function drawEnvironment()
global env
rectangle('Position',[env.left_wall,env.bottom_wall,env.width,env.height], ...
    'EdgeColor','k','FaceColor',[0.8,0.7,0.53]);

obstacles = env.obstacles;
for i = length(obstacles)
    hold on
    drawEllipse(obstacles{i});
    hold off
end
end