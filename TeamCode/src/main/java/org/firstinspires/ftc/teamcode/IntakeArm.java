package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeArm {
    private Servo intakeArm;

    public IntakeArm(HardwareMap hardwareMap) {
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
    }

    public class IntakeArmIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeArm.setPosition(0.09);
            return false;
        }
    }

    public Action IntakeArmIn() {
        return new IntakeArmIn();
    }

    public class IntakeArmOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeArm.setPosition(0.34);
            return false;
        }
    }

    public Action intakeArmOut() {
        return new IntakeArmOut();
    }
}
