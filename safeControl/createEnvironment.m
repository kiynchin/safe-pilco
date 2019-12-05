function env = createEnvironment()

global env
env.left_wall = 0;
env.right_wall = 100;
env.top_wall = 50;
env.bottom_wall = 0;
env.width = env.right_wall-env.left_wall;
env.height = env.top_wall-env.bottom_wall;
env.obstacles = {};

obst1.x = 50; obst1.y = 25; obst1.a = 30; obst1.b = 10;
env.obstacles{end+1} = obst1;
end