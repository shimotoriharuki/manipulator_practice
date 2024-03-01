classdef Manipulator_3DOF_3D
    properties(SetAccess = public)
        joint_angles_ = zeros(3, 1); %関節の角度
        arm_links_ = zeros(5, 1);

        hand_position_ = zeros(3, 1);   %ハンドの位置

        %各関節の位置
        P0_ = zeros(3, 1); 
        P1_ = zeros(3, 1);
        P2_ = zeros(3, 1);
        P3_ = zeros(3, 1);
        P4_ = zeros(3, 1);

        dt_ = 0; %制御周期
    end

    methods
        %コンストラクタ
        function obj = Manipulator_3DOF_3D(initial_thetas, arm_lengths, dt, initial_pose_display)
            obj.joint_angles_ = initial_thetas;
            obj.arm_links_ = arm_lengths;
            obj.dt_ = dt;

            obj = obj.updateRobotState; %初期状態のロボットの状態をアップデート

            if(initial_pose_display == true)
                obj.plotRobot; %初期姿勢をプロット
            end
        end

        %ロボットの状態を更新
        function obj = updateRobotState(obj)
            %ハンドの位置と姿勢を計算
            obj = obj.calcKinematics;

        end

        %ロボットの状態をプロット
        function plotRobot(obj)
            plot3([obj.P0_(1), obj.P1_(1)], [obj.P0_(2), obj.P1_(2)], [obj.P0_(3), obj.P1_(3)], "LineWidth", 3); %リンク1
            hold on 
            plot3([obj.P1_(1), obj.P2_(1)], [obj.P1_(2), obj.P2_(2)], [obj.P1_(3), obj.P2_(3)], "LineWidth", 3); %リンク2
            plot3([obj.P2_(1), obj.P3_(1)], [obj.P2_(2), obj.P3_(2)], [obj.P2_(3), obj.P3_(3)], "LineWidth", 3); %リンク3
            plot3([obj.P3_(1), obj.P4_(1)], [obj.P3_(2), obj.P4_(2)], [obj.P3_(3), obj.P4_(3)], "LineWidth", 3); %リンク4  
            hold off

            axis equal
            xlim([-2.5, 2.5])
            ylim([-2.5, 2.5])
            xlabel("x")
            ylabel("y")
            zlabel("z")

            drawnow
        end
    
        function obj = calcKinematics(obj)
            %関節角度から位置と姿勢を計算する
            th0 = obj.joint_angles_(1);
            th1 = obj.joint_angles_(2);
            th2 = obj.joint_angles_(3);
            l0 = obj.arm_links_(1);
            l1 = obj.arm_links_(2);
            l2 = obj.arm_links_(3);
            l3 = obj.arm_links_(4);
            l4 = obj.arm_links_(5);
            
            obj.P1_ = obj.P0_ + [0;
                                 0;
                                 l0 + l1];

            obj.P2_ = [l2 * cos(th0) * cos(th1);
                       l2 * sin(th0) * cos(th1);
                       l0 + l1 + l2 * sin(th1)];

            obj.P3_ = [cos(th0) * (l2 * cos(th1) + l3 * cos(th1 + th2));
                       sin(th0) * (l2 * cos(th1) + l3 * cos(th1 + th2));
                       l0 + l1 + l2 * sin(th1) + l3 * sin(th1 + th2)];

            obj.P4_ = obj.P3_ + [0;
                                 0;
                                 -l4];
            
            %ハンドの位置
            obj.hand_position_ = obj.P4_;
            
        end

        function obj = calcInverseKinematics(obj)
            %位置と姿勢から関節角度を計算する
            %リンクの長さ
            l0 = obj.arm_links_(1);
            l1 = obj.arm_links_(2);
            l2 = obj.arm_links_(3);
            l3 = obj.arm_links_(4);
            l4 = obj.arm_links_(5);
            %P3の位置
            x3_shift = obj.hand_position_(1);
            y3_shift = obj.hand_position_(2);
            z3_shift = obj.hand_position_(3) + l4 - l0 - l1; %P1を原点に持ってくるように加工

            %各関節角度を計算
            L13 = sqrt(x3_shift^2 + y3_shift^2 + z3_shift^2); %P1からP3までの距離
            th2 = -pi + acos((-L13^2 + l2^2 + l3^2) / (2 * l2 * l3));
            % th2 = -acos((-L13^2 + l2^2 + l3^2) / (2 * l2 * l3));

            l13 = sqrt(x3_shift^2 + y3_shift^2); %L13を斜辺としたときの直角三角形の底辺
            th1 = atan2(z3_shift, l13) + acos((-l3^2 + L13^2 + l2^2) / (2 * L13 * l2));

            th0 = atan2(y3_shift, x3_shift);
            
            obj.joint_angles_ = [th0; th1; th2];

        end

        function obj = calcInverseKinematicsUsingJacobian(obj, d_P, d_th)
            th0 = obj.joint_angles_(1);
            th1 = obj.joint_angles_(2);
            l0 = obj.arm_links_(1);
            l1 = obj.arm_links_(2);

            j11 = -cos(th0 + th1) / (l0 * cos(th0 + th1) * sin(th0) - l0 * sin(th0 + th1) * cos(th0));
            j12 = -sin(th0 + th1) / (l0 * cos(th0 + th1) * sin(th0) - l0 * sin(th0 + th1) * cos(th0));
            j13 = 0;
            j21 = (l1 * cos(th0 + th1) + l0 * cos(th0)) / (l0 * l1 * cos(th0 + th1) * sin(th0) - l0 * l1 * sin(th0 + th1) * cos(th0));
            j22 = (l1 * sin(th0 + th1) + l0 * sin(th0))/(l0 * l1 * cos(th0 + th1) * sin(th0) - l0 * l1 * sin(th0 + th1) * cos(th0));
            j23 = 0;
            j31 = 0;
            j32 = 0;
            j33 = 1;

            J_inv = [j11, j12, j13;
                     j21, j22, j23;
                     j31, j32, j33];

            d_joint_thetas = J_inv * [d_P; d_th];

            obj.joint_angles_ = obj.joint_angles_ + d_joint_thetas * obj.dt_;
            
        end

        %ハンド位置の取得
        function p = getPosition(obj)
            p = obj.hand_position_;
        end
        
        %ハンド位置の設定
        function obj = setPosition(obj, src_position)
            obj.hand_position_ = src_position;
        end

        %各関節角度の取得
        function t = getJointThetas(obj)
           t = obj.joint_angles_;
        end
     
        %各関節角度の設定
        function obj = setJointThetas(obj, thetas)
            obj.joint_angles_ = thetas;
        end
    end
end