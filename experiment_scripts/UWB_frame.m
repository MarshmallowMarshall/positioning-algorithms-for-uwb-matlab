A0_2d = [4.22, -0.09];          
A1_2d = [3.72, 1.71]; 
A2_2d = [0, 1.5];
A3_2d = [0, 0];

figure(1);
hold on
plot(A0_2d(1),A0_2d(2),'ro');
plot(A1_2d(1),A1_2d(2),'ro');
plot(A2_2d(1),A2_2d(2),'ro');
plot(A3_2d(1),A3_2d(2),'ro');

grid on;
axis([-0.5,5,-1,2]);axis equal;
set(gca,'XTick',-0.5:0.5:5);
set(gca,'YTick',-1:0.5:4);
