clear,clc,close all
syms r t

x = r*cos(t);
y = r*sin(t);

diff(diff(x,t),t)
diff(diff(y,t),t)

