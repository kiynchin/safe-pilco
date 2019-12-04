function drawEllipse(params)
a=params.a; % horizontal radius
b=params.b; % vertical radius
x0=params.x; % x0,y0 ellipse centre coordinates
y0=params.y;
t=-pi:0.01:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
plot(x,y)

end