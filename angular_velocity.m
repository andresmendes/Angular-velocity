%% Angular velocity
% Simulation and animation of a rotating cube illustrating angular velocity
% vector.
%
%%

clear ; close all ; clc

%% Cube

% Cube edge length [m]
L = 0.5; 

% Side 1 (Front)
c1 = [ L/2 -L/2  L/2];
c2 = [ L/2 -L/2 -L/2];
c3 = [ L/2  L/2 -L/2];
c4 = [ L/2  L/2  L/2];

side1 = [c1 ; c2 ; c3 ; c4 ; c1];

% Side 2 (Rear)
c5 = [-L/2 -L/2  L/2];
c6 = [-L/2 -L/2 -L/2];
c7 = [-L/2  L/2 -L/2];
c8 = [-L/2  L/2  L/2];

side2 = [c5 ; c6 ; c7 ; c8 ; c5];

% Connections
c15 = [c1 ; c5];
c26 = [c2 ; c6];
c37 = [c3 ; c7];
c48 = [c4 ; c8];

figure
hold on ; grid on ; box on ; axis equal
set(gca,'xlim',[-L L],'ylim',[-L L],'zlim',[-L L],'CameraPosition',[5.2490 4.6237 2.7053])
% Side 1
plot3(side1(:,1),side1(:,2),side1(:,3),'r')
% Side 2
plot3(side2(:,1),side2(:,2),side2(:,3),'b')
% Connections
plot3(c15(:,1),c15(:,2),c15(:,3),'g')
plot3(c26(:,1),c26(:,2),c26(:,3),'g')
plot3(c37(:,1),c37(:,2),c37(:,3),'g')
plot3(c48(:,1),c48(:,2),c48(:,3),'g')

xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

%% Simulation

% Video
tF      = 15;                   % Final time                    [s]
fR      = 60;                   % Frame rate                    [fps]
dt      = 1/fR;                 % Time resolution               [s]
time    = linspace(0,tF,tF*fR); % Time                          [s]

w       = [1 0 0];

corners = [c1 ; c2 ; c3 ; c4  ; c5 ; c6 ; c7 ; c8];

% Preallocating
x = zeros(length(time),8);
y = zeros(length(time),8);
z = zeros(length(time),8);

for i=1:size(corners,1)

    states_1_init = corners(i,:);
    [time,states_1] = ode45(@(t,states) dot_kinematics(t,states,w),time,states_1_init); 

    % Retrieving states
    % Columns -> corners
    % Rows -> Time
    x(:,i) = states_1(:,1);
    y(:,i) = states_1(:,2);
    z(:,i) = states_1(:,3); 
    
end

plot3(x,y,z)


%% Animation

% Camera position setup:
raio   = 10;                               % Raio de rotação da câmera
th  = linspace(0,pi/2,length(time));    % Ângulo da varredura da câmera
X   = raio*cos(th);
Y   = raio*sin(th);

figure
% set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% set(gcf,'Position',[50 50 854 480])   % YouTube: 480p
set(gcf,'Position',[50 50 640 640])     % Social

fact = 1.4;

hold on ; grid on ; box on ; axis equal
set(gca,'FontName','Verdana','FontSize',18)
% set(gca,'Position',[0.2 0.2 0.6 0.6])
set(gca,'xlim',[-fact*L fact*L],'ylim',[-fact*L fact*L],'zlim',[-fact*L fact*L])
set(gca,'XTick',[],'YTick',[],'ZTick',[])
set(gca,'CameraPosition',[5.2490 4.6237 2.7053])

an = annotation('textbox', [0.32 0.9, 0.5, 0.1], 'string', 'Angular velocity','FitBoxToText','on');
an.FontName     = 'Verdana';
an.FontSize     = 18;
an.LineStyle    = 'none';
an.FontWeight   = 'Bold';

% Create and open video writer object
v = VideoWriter('angular_velocity.mp4','MPEG-4');
v.Quality   = 100;
v.FrameRate = fR;
open(v);

for i=1:length(time)
    
    cla

    % Update camera position
    set(gca,'CameraPosition',[X(i)    Y(i)    1])
   
    % Side 1 (Front)
    plot3([x(i,1:4) x(i,1)] , [y(i,1:4) y(i,1)] , [z(i,1:4) z(i,1)],'k','LineWidth',3)
    % Side 2 (Rear)
    plot3([x(i,5:8) x(i,5)] , [y(i,5:8) y(i,5)] , [z(i,5:8) z(i,5)],'k','LineWidth',3)
    % Connections
    plot3([x(i,1) x(i,5)] , [y(i,1) y(i,5)] , [z(i,1) z(i,5)],'k','LineWidth',3)
    plot3([x(i,2) x(i,6)] , [y(i,2) y(i,6)] , [z(i,2) z(i,6)],'k','LineWidth',3)
    plot3([x(i,3) x(i,7)] , [y(i,3) y(i,7)] , [z(i,3) z(i,7)],'k','LineWidth',3)
    plot3([x(i,4) x(i,8)] , [y(i,4) y(i,8)] , [z(i,4) z(i,8)],'k','LineWidth',3)

    % Angular velocity
    plot_vector(0.4*w);
    
%     xlabel('x [m]')
%     ylabel('y [m]')
%     zlabel('z [m]')
    
    frame = getframe(gcf);
    writeVideo(v,frame);

end

close(v);

function dstates = dot_kinematics(~,states,w)

    % Position
    x = states(1);
    y = states(2);
    z = states(3);

    % Angular velocity
    wx = w(1);
    wy = w(2);
    wz = w(3);

    % Dot kinematics
    dx = wy*z - wz*y;
    dy = wz*x - wx*z;
    dz = wx*y - wy*x;
    
    dstates = [dx ; dy ; dz];

end

function plot_vector(v_original)

    % Orientation vector
    % Cartesian coordinates

    
    
    v_x_coord = v_original(1);
    v_y_coord = v_original(2);
    v_z_coord = v_original(3);

    % ABS vector
    v_abs = sqrt(v_x_coord^2 + v_y_coord^2 + v_z_coord^2);

    % Unit vector
    v_x_unit = v_x_coord / v_abs;
    v_y_unit = v_y_coord / v_abs;
    v_z_unit = v_z_coord / v_abs;

    v_unit = [v_x_unit v_y_unit v_z_unit];

    %% Cone
    R = 0.05;       % Radius
    H = 0.2;        % Height
    N = 50;         % Number of points
    [xCy1 , yCy1 , zCy1] = cylinder([0 R], N);

    % Updating arrow to point upward
    zCy1 = -H*zCy1;

    % Cone point at zero and vertical (k)
    x = xCy1(2,:);
    y = yCy1(2,:);
    z = zCy1(2,:);

    v1 = [ x ; y ; z];

    %% Rotations 

    % Rotation 1 (j)
    th_j = atan(sqrt(v_x_unit^2 + v_y_unit^2) / v_z_unit);

    % Rotation matrix (j)
    r_j = [ cos(th_j)        0       sin(th_j)      ;
                0           1           0           ;
            -sin(th_j)       0       cos(th_j)      ];

    % Preallocating
    v2 = zeros(3,size(v1,2));
    
    for i=1:size(v1,2)
        v2(:,i) = r_j*v1(:,i);
    end
    
    xCy2 = xCy1;
    yCy2 = yCy1;
    zCy2 = zCy1;

    xCy2(2,:) = v2(1,:);
    yCy2(2,:) = v2(2,:);
    zCy2(2,:) = v2(3,:);

    % Rotation 2 (k)
    th_k = atan(v_y_unit / v_x_unit);

    % Rotation matrix (k)
    r_k = [ cos(th_k)     -sin(th_k)      0   ;
            sin(th_k)      cos(th_k)      0   ;
            0               0           1   ];

    % Preallocating
    v3 = zeros(3,size(v1,2));
    
    for i=1:size(v2,2)
        v3(:,i) = r_k*v2(:,i);
    end
    
    xCy3 = xCy2;
    yCy3 = yCy2;
    zCy3 = zCy2;

    xCy3(2,:) = v3(1,:);
    yCy3(2,:) = v3(2,:);
    zCy3(2,:) = v3(3,:);

    % Plot
    % Origin
    plot([0 0 0],[0 0 0],'go','MarkerFaceColor','g','MarkerSize',10)
    % Rod
    plot3([0 v_x_coord],[0 v_y_coord],[0 v_z_coord],'g','LineWidth',3) 

    m = surf(xCy3+(v_unit(1)*(H+v_abs)), yCy3+ (v_unit(2)*(H+v_abs)), zCy3+(v_unit(3)*(H+v_abs)));
    set(m,'edgealpha',0.1,'facecolor',[0 1 0])
%     set(m,'edgealpha',0,'facecolor',[1 0 0],'facealpha',0.2)

end
