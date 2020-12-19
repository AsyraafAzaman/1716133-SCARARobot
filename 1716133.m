%% DH parameters
d1 = 5;
%d3 = 2.5;
a1 = 6;
a2 = 6;

%% Link Description

% Link([theta, d, a, alpha, R/P])
L(1) = Link([0,d1,a1,0,0],'standard');%theta 1
L(2) = Link([0,0,a2,pi,0],'standard');%theta 2
L(3) = Link([0,0,0,0,1],'standard');%d3%prismatic
L(3).qlim = [0,10];    
L(4) = Link([0,0,0,0,0],'standard');%theta 4

%% SerialLink to make robot

robot = SerialLink(L,'name','SekaRA Robot');
%   robot.plot([0 0 0 0],'workspace',[-15 15 -15 15 -15 15]);
% robot.teach

%% get path

prompt = 'Choose the path flow whether in Continous,C or Discrete,D ,(C=1 or D=0) \nuser input : ';
a = input(prompt);
matrix = getmatrix(a);

%% Transformation Matrix

qinit = [0 0 0 0];%initial position x = 12, y = 0, z = 5

T1 = transl(matrix(1,1),matrix(1,2),matrix(1,3));
T2 = transl(matrix(2,1),matrix(2,2),matrix(2,3));
T3 = transl(matrix(3,1),matrix(3,2),matrix(3,3));
T4 = transl(matrix(4,1),matrix(4,2),matrix(4,3));


%% use inverse kinematics to get joint angle

q1 = robot.ikine(T1,qinit,[1,1,1,0,0,0]);
q2 = robot.ikine(T2,q1,[1,1,1,0,0,0]);
q3 = robot.ikine(T3,q2,[1,1,1,0,0,0]);
q4 = robot.ikine(T4,q3,[1,1,1,0,0,0]);

%% use forwardkinematics to find the x,y,z ( to verify the joinnt angle that got from the inverese kinematics )

q1_T1 = robot.fkine(q1);
q2_T2 = robot.fkine(q2);
q3_T3 = robot.fkine(q3);
q4_T4 = robot.fkine(q4);

%% Animation

t =(0: .5:3)'; %time vector for where one transformation took 3 seconds + pause(0.02) before undergo next transformations;

% The trajectory of this scara is :
% qinit -> q1 -> q2 -> q3 -> q4 -> q1 -> qinit


% First Transformation = initial position to 1st position 
transformation1 = jtraj(qinit,q1,t);
robot.plot(transformation1,'workspace',[-15 15 -15 15 -15 15]);
pause(0.02);

% Second Transformation = 1st position to 2nd position 
transformation2 = jtraj(q1,q2,t);
robot.plot(transformation2,'workspace',[-15 15 -15 15 -15 15]);
pause(0.02);

% Third Transformation = 2nd position to 3rd position 
transformation3 = jtraj(q2,q3,t);
robot.plot(transformation3,'workspace',[-15 15 -15 15 -15 15]);
pause(0.02);

% Fourth Transformation = 3rd position to 4th position 
transformation4 = jtraj(q3,q4,t);
robot.plot(transformation4,'workspace',[-15 15 -15 15 -15 15]);
pause(0.02);

% Fifth Transformation = 4th position to 1st position 
transformation5 = jtraj(q4,q1,t);
robot.plot(transformation5,'workspace',[-15 15 -15 15 -15 15]);
pause(0.02);

% Sixth Transformation = 1st position to initial position 
transformation6 = jtraj(q1,qinit,t);
robot.plot(transformation6,'workspace',[-15 15 -15 15 -15 15]);
pause(0.02);

%% to plot the trajectory path

trajectory = [12 0 5; 
            matrix(1,:);
            matrix(2,:);
            matrix(3,:);
            matrix(4,:);
            12 0 5];
[nx,ny] = size(trajectory) ;
figure
hold on

for i = 1:nx-1
    v=[trajectory(i,:);trajectory(i+1,:)];
    plot3(v(:,1),v(:,2),v(:,3),'g');
    plot3(v(:,1),v(:,2),v(:,3),'bo')
end

text(trajectory(1,1),trajectory(1,2),trajectory(1,3), ' initial position');
text(trajectory(2,1),trajectory(2,2),trajectory(2,3), ' T1');
text(trajectory(3,1),trajectory(3,2),trajectory(3,3), ' T2');
text(trajectory(4,1),trajectory(4,2),trajectory(4,3), ' T3');
text(trajectory(5,1),trajectory(5,2),trajectory(5,3), ' T4');


 xlabel('X');
 ylabel('Y');
 zlabel('Z');

view(3);

%% if condition to show data

pause(2);

prompt1 = 'Permission to show data,(Yes=1 or No=0) \nuser input : ';
a = input(prompt1);

if a==1
    showdata(matrix,q1,q2,q3,q4);
end
if a==0
    disp(' table will not shown');
end


%% show data table contains of x, y, z, theta1_radians and theta2_radians

function showdata(m1,Q1,Q2,Q3,Q4)

close all;
col = {'x','y','z','theta1_radians','theta2_radians'};
row = {'T1','T2','T3','T4'};
datatable = [ m1(1,:) Q1(1,1:2); m1(2,:) Q2(1,1:2); m1(3,:) Q3(1,1:2); m1(4,:) Q4(1,1:2)];
figure;
DATAtable = uitable('columnname',col,'rowname',row,'position',[50 50 700 200],'columnwidth',{100},'data',datatable);

end

%% get continuous or discrete transformation matrix

function [g] = getmatrix(b)

if b==0 %DISCRETE
    g = [4 4 5;
            4 -4 5;
            -4 -4 5;
            -4 4 5;];
    return;
end

 if b==1
        g = [7 7 7;
            7 -7 1.5;
            -7 -7 -3;
            -7 7 6;];
        return; %CONTINUOUS
 end
    
 
 
end




