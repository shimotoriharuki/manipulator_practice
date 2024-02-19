classdef Manipulator_3DOF_2D
    properties 
        joint_thetas = zeros(1, 3); %関節の角度
        position = zeros(1, 3);%手先の位置
        orientation = zeros(1, 4);%手先の姿勢
        arm_lengths = zeros(1, 2);
    end

    methods
        %コンストラクタ
        function obj = Manipulator_3DOF_2D(initial_thetas, arm_lengths)
            obj.joint_thetas = initial_thetas;
            obj.arm_lengths = arm_lengths;

            obj.calcKinematics;
        end

        %手先位置の取得
        function p = getPosition(obj)
            p = obj.position;
        end
        
        %手先位置の設定
        function obj = setPosition(obj, src_position)
            obj.position = src_position;
        end

        %手先姿勢の取得
        function o = getOrientation(obj)
            o = obj.orientation;
        end

        %ロボットの状態をプロット
        function plotRobot(obj)
        end
    end

    methods (Access = private)
        function obj = calcKinematics(obj)
            %関節角度から位置と姿勢を計算する
            obj.position = [0, 0, 0];
        end
        function obj = calcInverseKinematics(obj)
            %位置と姿勢から関節角度をを計算する
            obj.joint_thetas = [0, 0, 0];
        end
    end

end