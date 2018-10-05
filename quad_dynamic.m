function s=quad_dynamic(s,omega,p,dt)

x=s(1);y=s(2);z=s(3);
vx=s(4);vy=s(5);vz=s(6);
phi=s(7);theta=s(8);psi=s(9);
vphi=s(10);vtheta=s(11);vpsi=s(12);
tr=[p.b p.b p.b p.b;0 -p.b 0 p.b;-p.b 0 p.b 0;-p.d p.d -p.d p.d];

omega2=(omega).^2;
Omega=-omega(1)+omega(2)-omega(3)+omega(4);
u=tr*omega2;

vx=vx+dt*(u(1)*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))/p.m-p.k1*vx/p.m);
x=x+dt*vx;
vy=vy+dt*(u(1)*(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))/p.m-p.k2*vy/p.m);
y=y+dt*vy;
vz=vz+dt*(u(1)*(cos(theta)*cos(phi))/p.m-p.g-p.k3*vz/p.m);
z=z+dt*vz;
vphi=vphi+dt*((p.Iy-p.Iz)/p.Ix*vtheta*vpsi-p.Jr*vtheta*Omega+u(2)*p.l/p.Ix-p.k4*vphi*p.l/p.Ix);
phi=phi+dt*vphi;
vtheta=vtheta+dt*((p.Iz-p.Ix)/p.Iy*vphi*vpsi+p.Jr*vphi*Omega+u(3)*p.l/p.Iy-p.k5*vtheta*p.l/p.Iy);
theta=theta+dt*vtheta;
vpsi=vpsi+dt*((p.Ix-p.Iy)/p.Iz*vphi*vtheta+u(4)*1/p.Iz-p.k6*vpsi/p.Iz);
psi=psi+dt*vpsi;

s=[x;y;z;vx;vy;vz;phi;theta;psi;vphi;vtheta;vpsi];
end
