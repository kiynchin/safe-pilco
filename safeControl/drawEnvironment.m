env.left_wall = 0;
env.right_wall = 100;
env.top_wall = 50;
env.bottom_wall = 0;
env.width = env.right_wall-env.left_wall;
env.height = env.top_wall-env.bottom_wall;



rectangle('Position',[env.left_wall,env.bottom_wall,env.width,env.height], ...
    'EdgeColor','k','FaceColor',[0.5,0.8,0.5]);
