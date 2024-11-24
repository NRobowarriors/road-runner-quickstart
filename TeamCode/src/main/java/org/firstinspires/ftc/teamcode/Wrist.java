package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private Servo clawWrist;
    public Wrist(HardwareMap hardwareMap) {
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
    }
    public class WristUp implements Action {
        double position;
        public WristUp(double positionUp){
            position = positionUp;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawWrist.setPosition(1);
            return false;
        }


    }
    public Action wristUp(double positionUp) {
        return new WristUp(positionUp);
    }
    public class WristDown implements Action {
        double position;
        public WristDown(double positionDown) {
            position = positionDown;
        }
        @Override
            public boolean run(@NonNull TelemetryPacket packet) {
            clawWrist.setPosition(1);
            return false;
            }
        }
        public Action wristDown(double positionDown) {
            return new WristDown(positionDown);
    }
}
