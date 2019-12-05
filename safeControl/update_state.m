function update_state(dq,dt)
global q
q.x = q.x+dq(1)*dt;
q.y = q.y+dq(2)*dt;
q.x_vel = q.x_vel+dq(3)*dt;
q.y_vel = q.y_vel+dq(4)*dt;
end