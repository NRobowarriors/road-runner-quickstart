package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerArm {
    private Servo armUpFlowersR, armUpFlowersL;

    public FlowerArm(HardwareMap hardwareMap) {
        armUpFlowersR = hardwareMap.get(Servo.class, "armUpFlowersR");
        armUpFlowersL = hardwareMap.get(Servo.class, "armDownFlowersL");
    }

    public class FlowerArmUp implements Action {
        double position;
        public FlowerArmUp(double positionOut){
            position = positionOut;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armUpFlowersL.setPosition(position);
            armUpFlowersR.setPosition(position);
            return false;
        }

    }

    public Action flowerArmUp(double position) {
        return new FlowerArmUp(position);
    }

    public class flowerArmDown implements Action {
        double position;
        public flowerArmDown(double positionIn){
            position = positionIn;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armUpFlowersL.setPosition(position);
            armUpFlowersR.setPosition(position);
            return false;
        }
        public Action flowerArmDown(double positionIn) {
            return new flowerArmDown(positionIn);
        }
    }
}
