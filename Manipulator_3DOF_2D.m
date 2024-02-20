classdef Manipulator_3DOF_2D
    properties(SetAccess = public)
        joint_thetas_ = zeros(3, 1); %関節の角度
        position_ = zeros(3, 1);%手先の位置
        orientation_ = zeros(1, 1);%手先の姿勢
        arm_lengths_ = zeros(2, 1);

        P0_ = zeros(3, 1);
        P1_ = zeros(3, 1);
        P2_ = zeros(3, 1);
    end

    methods
        %コンストラクタ
        function obj = Manipulator_3DOF_2D(initial_thetas, arm_lengths)
            obj.joint_thetas_ = initial_thetas;
            obj.arm_lengths_ = arm_lengths;

            obj = obj.calcKinematics; %初期姿勢を計算
            obj.plotRobot; %初期姿勢をプロット
        end

        %手先位置の取得
        function p = getPosition(obj)
            p = obj.position_;
        end
        
        %手先位置の設定
        function obj = setPosition(obj, src_position)
            obj.position_ = src_position;
        end

        %手先姿勢の取得
        function o = getOrientation(obj)
            o = obj.orientation_;
        end

        %手先姿勢の設定
        function obj = setOrientation(obj, theta)
            obj.orientation_ = theta;
        end

        %各関節角度の取得
        function t = getJointThetas(obj)
           t = obj.joint_thetas_;
        end
     
        %各関節角度の設定
        function obj = setJointThetas(obj, thetas)
            obj.joint_thetas_ = thetas;
        end

        function obj = updateRobotState(obj)
            obj = obj.calcKinematics;
        end

        %ロボットの状態をプロット
        function plotRobot(obj)
            plot([obj.P0_(1), obj.P1_(1)], [obj.P0_(2), obj.P1_(2)]); %リンク1
            hold on 
            plot([obj.P1_(1), obj.P2_(1)], [obj.P1_(2), obj.P2_(2)]); %リンク2
            hold off

            axis equal
            xlim([-2, 2])
            ylim([-2, 2])

            drawnow
        end
    
        function obj = calcKinematics(obj)
            %関節角度から位置と姿勢を計算する
            theta0 = obj.joint_thetas_(1);
            length0 = obj.arm_lengths_(1);
            obj.P1_ = obj.P0_ + [length0, 0, 0; 
                  0, length0, 0; 
                  0, 0, length0] * [cos(theta0); sin(theta0); 0];

            theta1 = obj.joint_thetas_(2);
            length1 = obj.arm_lengths_(2);
            obj.P2_ = obj.P1_ + [length1, 0, 0; 
                       0, length1, 0; 
                       0, 0, length0] * [cos(theta0 + theta1); sin(theta0 + theta1); 0];
            
            obj.position_ = obj.P2_;
        end

        function obj = calcInverseKinematics(obj)
            %位置と姿勢から関節角度を計算する
            l0 = obj.arm_lengths_(1);
            l1 = obj.arm_lengths_(2);
            x = obj.position_(1);
            y = obj.position_(2);
            theta1 = pi - acos((l0^2 + l1^2 - x^2 - y^2)/(2 * l0 * l1));
            theta0 = atan2(y, x) - acos((x^2 + y^2 + l0^2 - l1^2)/(2 * sqrt(x^2 + y^2) * l0));
            obj.joint_thetas_ = [theta0; theta1; 0];
        end
    end

end