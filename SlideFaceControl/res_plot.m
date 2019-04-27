figure(1)
plot3(rx,ry,rz,'-r')
hold on
grid on
plot3(rx1,ry1,rz1,'--b')

save('state.txt','state','-ascii');
save('state_des.txt','state_des','-ascii');
save('u_control.txt','u_control','-ascii');
save('time.txt','t','-ascii');