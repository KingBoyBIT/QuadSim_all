function dstate = statefunc(input)
global m g l I_1 I_2 I_3 K_1 K_2 K_3 K_4 K_5 K_6

state = input(1:12);
x = state(1);
y = state(2);
z = state(3);
d_x = state(4);
d_y = state(5);
d_z = state(6);
phi = state(7);
theta = state(8);
psi = state(9);
d_phi = state(10);
d_theta = state(11);
d_psi = state(12);
control = input(13:16);
u_1 = control(1);
u_2 = control(2);
u_3 = control(3);
u_4 = control(4);
dd_x=u_1*(cos(phi)* sin(theta)* cos( psi)+sin(phi)*sin(psi))-K_1*d_x/m;
dd_y=u_1*(cos(phi)* sin(theta)* sin( psi)-sin(phi)*cos(psi))-K_2*d_y/m;
dd_z=u_1*(cos(phi)* cos(theta)) - g-K_3*d_z/m;
dd_phi=u_2-l*K_4*d_phi/I_1;
dd_theta=u_3-l*K_5*d_theta/I_2;
dd_psi=u_4-K_6*d_psi/I_3;

dstate = [
	d_x;d_y;d_z;
	dd_x;dd_y;dd_z;
	d_phi;d_theta;d_psi;
	dd_phi;dd_theta;dd_psi];
end