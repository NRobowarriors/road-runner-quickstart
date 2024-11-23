package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tilt {
    private Servo intakeTilt;

    public class TiltUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeTilt.setPosition(1);
            return false;
        }
    }

    public Action tiltUp() {
        return new TiltUp();
    }

    public Tilt(HardwareMap hardwareMap) {
        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
    }

    public class TiltDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeTilt.setPosition(0.5);
            return false;
        }
    }

    public Action tiltDown() {
        return new TiltDown();
    }
}
