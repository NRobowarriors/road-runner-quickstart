package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalLift {
    private DcMotorEx motorVerticalRight, motorVerticalLeft;

    public VerticalLift(HardwareMap hardwareMap) {
        motorVerticalLeft = hardwareMap.get(DcMotorEx.class, "LmotorLift");
        motorVerticalRight = hardwareMap.get(DcMotorEx.class, "RmotorLift");
        motorVerticalLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorVerticalRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorVerticalLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorVerticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class VerticalLiftUp implements Action {
        private boolean initialized = false;
        private int ticks;
        private double power;
        public VerticalLiftUp(int ticksIn, double powerIn){
            ticks = ticksIn;
            power = powerIn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motorVerticalLeft.setPower(power);
                motorVerticalRight.setPower(power);
                initialized = true;
            }
            int pos = motorVerticalLeft.getCurrentPosition();
            packet.put("verticalPos", pos);
            if (pos < ticks) {
                return true;
            } else {
                motorVerticalLeft.setPower(0.001);
                motorVerticalRight.setPower(0.001);
                return false;
            }
        }
    }

    public Action verticalLiftUp(int ticks, double power) {
        return new VerticalLiftUp(ticks, power);
    }

    public class VerticalLiftDown implements Action {
        private boolean initialized = false;
        private int ticks;
        private double power;
        public VerticalLiftDown(int ticksIn, double powerIn){ticks = ticksIn; power = powerIn;}

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motorVerticalLeft.setPower(-1 * power);
                motorVerticalRight.setPower(-1 * power);
                initialized = true;
            }
            double pos = motorVerticalLeft.getCurrentPosition();
            packet.put("verticalPos", pos);
            if (pos > ticks) {
                return true;
            } else {
                motorVerticalLeft.setPower(0);
                motorVerticalRight.setPower(0);
                return false;
            }
        }
    }

    public Action verticalLiftDown(int ticksIn, double powerIn) {
        return new VerticalLiftDown(ticksIn, powerIn);
    }
    public class verticalLiftStay implements Action {
        private boolean initialized = false;
        private double power;
        public verticalLiftStay(double powerIn){ power = powerIn;}

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motorVerticalLeft.setPower(1 * power);
                motorVerticalRight.setPower(1 * power);
                initialized = true;
            }
            double pos = motorVerticalLeft.getCurrentPosition();
            packet.put("verticalPos", pos);
            return false;
        }
    }

    public Action verticalLiftStay(double powerIn) {
        return new verticalLiftStay(powerIn);
    }
}
