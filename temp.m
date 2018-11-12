clear,clc,close all
syms r t b d

x = r*cos(t);
y = r*sin(t);

diff(diff(x,t),t)
diff(diff(y,t),t)

tr=[b b b b;
	0 -b 0 b;
	-b 0 b 0;
	-d d -d d];

inv(tr)