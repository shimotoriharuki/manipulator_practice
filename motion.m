robot = Manipulator_3DOF_2D([0; 0; 0], [1; 1]);

% %順運動学
% for angle = 0 : pi/10 : 2*pi
%     robot = robot.setJointThetas([angle; angle; 0]);
%     robot = robot.updateRobotState;
%     robot.plotRobot;
% 
%     drawnow; 
% end

%逆運動学
for position = 2 : -0.1 : 0
    robot = robot.setPosition([position; 0; 0]);
    robot = robot.calcInverseKinematics;
    robot = robot.updateRobotState;
    robot.plotRobot;

    drawnow; 
end