robot = Manipulator_3DOF_2D([0; 0; 0], [1; 1]);

for angle = 0 : pi/10 : 2*pi
    robot = robot.setJointThetas([angle; angle; 0]);
    robot = robot.calcKinematics;
    robot.plotRobot;

    drawnow; 
end