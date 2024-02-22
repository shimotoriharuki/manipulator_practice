classdef Manipulator_3DOF_2D
    properties(SetAccess = public)
        joint_thetas_ = zeros(3, 1); %関節の角度
        hand_position_ = zeros(3, 1);%手先の位置
        hand_orientation_ = zeros(1, 1);%手先の姿勢
        arm_lengths_ = zeros(2, 1);

        P0_ = zeros(3, 1);
        P1_ = zeros(3, 1);
        P2_ = zeros(3, 1);

        finger_position1_ = zeros(3, 1);
        finger_position2_ = zeros(3, 1);

        dt_ = 0; %制御周期
    end

    methods
        %コンストラクタ
        function obj = Manipulator_3DOF_2D(initial_thetas, arm_lengths, dt)
            obj.joint_thetas_ = initial_thetas;
            obj.arm_lengths_ = arm_lengths;
            obj.dt_ = dt;

            % obj = obj.calcKinematics; %初期姿勢を計算
            obj = obj.updateRobotState;
            obj.plotRobot; %初期姿勢をプロット
        end

        %手先位置の取得
        function p = getPosition(obj)
            p = obj.hand_position_;
        end
        
        %手先位置の設定
        function obj = setPosition(obj, src_position)
            obj.hand_position_ = src_position;
        end

        %手先姿勢の取得
        function o = getOrientation(obj)
            o = obj.hand_orientation_;
        end

        %手先姿勢の設定
        function obj = setOrientation(obj, theta)
            obj.hand_orientation_ = theta;
        end

        %各関節角度の取得
        function t = getJointThetas(obj)
           t = obj.joint_thetas_;
        end
     
        %各関節角度の設定
        function obj = setJointThetas(obj, thetas)
            obj.joint_thetas_ = thetas;
        end

        %ロボットの状態を更新
        function obj = updateRobotState(obj)
            %エンドエフェクタの位置を計算
            obj = obj.calcKinematics;

            %指先の位置を計算
            obj.finger_position1_ = obj.hand_position_ + [0.2 * cos(obj.hand_orientation_); 0.2 * sin(obj.hand_orientation_); 0];
            obj.finger_position2_ = obj.hand_position_ + [0.1 * cos(obj.hand_orientation_ + pi/2); 0.1 * sin(obj.hand_orientation_ + pi/2); 0];
        end

        %ロボットの状態をプロット
        function plotRobot(obj)
            plot([obj.P0_(1), obj.P1_(1)], [obj.P0_(2), obj.P1_(2)], "LineWidth", 3); %リンク1
            hold on 
            plot([obj.P1_(1), obj.P2_(1)], [obj.P1_(2), obj.P2_(2)], "LineWidth", 3); %リンク2
            plot([obj.P2_(1), obj.finger_position1_(1)], [obj.P2_(2), obj.finger_position1_(2)], "LineWidth", 3); %指先
            plot([obj.P2_(1), obj.finger_position2_(1)], [obj.P2_(2), obj.finger_position2_(2)], "LineWidth", 3); %指先
            hold off

            axis equal
            xlim([-2.5, 2.5])
            ylim([-2.5, 2.5])

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
            
            obj.hand_position_ = obj.P2_;
        end

        function obj = calcInverseKinematics(obj)
            %位置と姿勢から関節角度を計算する
            l0 = obj.arm_lengths_(1);
            l1 = obj.arm_lengths_(2);
            x = obj.hand_position_(1);
            y = obj.hand_position_(2);
            theta1 = pi - acos((l0^2 + l1^2 - x^2 - y^2)/(2 * l0 * l1));
            theta0 = atan2(y, x) - acos((x^2 + y^2 + l0^2 - l1^2)/(2 * sqrt(x^2 + y^2) * l0));
            obj.joint_thetas_ = [theta0; theta1; 0];
        end

        function obj = calcInverseKinematicsUsingJacobian(obj, d_hand_position)
            th0 = obj.joint_thetas_(1);
            th1 = obj.joint_thetas_(2);
            l0 = obj.arm_lengths_(1);
            l1 = obj.arm_lengths_(2);

            j11 = -cos(th0 + th1) / (l0 * cos(th0 + th1) * sin(th0) - l0 * sin(th0 + th1) * cos(th0));
            j12 = -sin(th0 + th1) / (l0 * cos(th0 + th1) * sin(th0) - l0 * sin(th0 + th1) * cos(th0));
            j21 = (l1 * cos(th0 + th1) + l0 * cos(th0)) / (l0 * l1 * cos(th0 + th1) * sin(th0) - l0 * l1 * sin(th0 + th1) * cos(th0));
            j22 = (l1 * sin(th0 + th1) + l0 * sin(th0))/(l0 * l1 * cos(th0 + th1) * sin(th0) - l0 * l1 * sin(th0 + th1) * cos(th0));
            
            J_inv = [j11, j12;
                     j21, j22];

            d_joint_thetas = J_inv * d_hand_position;

            obj.joint_thetas_ = obj.joint_thetas_ + [d_joint_thetas; 0] * obj.dt_;
            
        end
    end
end