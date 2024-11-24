package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private CRServo intakeWheelRight, intakeWheelLeft;

    public Intake(HardwareMap hardwareMap) {
        intakeWheelLeft = hardwareMap.get(CRServo.class, "intakeWheelLeft");
        intakeWheelRight = hardwareMap.get(CRServo.class, "intakeWheelRight");
    }

    public class IntakeSuckIn implements Action {
        double position;
        public IntakeSuckIn() {

        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeWheelLeft.setPower(-0.5);
            intakeWheelRight.setPower(0.5);
            return false;
        }
    }

    public Action intakeSuckIn() {
        return new IntakeSuckIn();
    }
    public class intakeSuckInStop implements Action {
    double position;
    public intakeSuckInStop() {

    }
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        intakeWheelLeft.setPower(0);
        intakeWheelRight.setPower(0);
        return false;
    }
}

public Action intakeSuckInStop() {
    return new intakeSuckInStop();
}
}
