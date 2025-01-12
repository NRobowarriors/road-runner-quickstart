package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VerticalLift {
    private DcMotorEx motorVerticalRight, motorVerticalLeft;
    private Telemetry telemetry;

    public VerticalLift(HardwareMap hardwareMap, Telemetry intelemetry) {
        motorVerticalLeft = hardwareMap.get(DcMotorEx.class, "LmotorLift");
        motorVerticalRight = hardwareMap.get(DcMotorEx.class, "RmotorLift");
        motorVerticalLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorVerticalRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorVerticalLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorVerticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = intelemetry;
    }

    public class VerticalLiftUp implements Action {
        private boolean initialized = false;
        private int ticks;
        private double power;
        private int startPos;
        public VerticalLiftUp(int ticksIn, double powerIn){
            ticks = ticksIn;
            power = powerIn;
            startPos = motorVerticalLeft.getCurrentPosition();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motorVerticalLeft.setTargetPosition(startPos + ticks);
                motorVerticalRight.setTargetPosition(startPos + ticks);
                motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorVerticalLeft.setPower(power);
                motorVerticalRight.setPower(power);
                initialized = true;
            }
            int pos = motorVerticalLeft.getCurrentPosition();
            packet.put("verticalPos", pos);
            telemetry.addData("vertical Pos", pos);
            telemetry.update();
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
            packet.addLine("verticalPos " +pos);

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
    public class exactVertical implements Action {
        private boolean initialized = false;
        private int ticks;
        private double power;
        private int startPos;
        public exactVertical(int ticksIn, double powerIn){
            ticks = ticksIn;
            power = powerIn;
            startPos = motorVerticalLeft.getCurrentPosition();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motorVerticalLeft.setTargetPosition(startPos + ticks);
                motorVerticalRight.setTargetPosition(startPos + ticks);
                motorVerticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorVerticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorVerticalLeft.setPower(power);
                motorVerticalRight.setPower(power);
                initialized = true;
            }
            int pos = motorVerticalLeft.getCurrentPosition();
            packet.put("verticalPos", pos);
            telemetry.addData("vertical Pos", pos);
            telemetry.update();
            if (pos != ticks) {
                return true;
            } else {
                motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorVerticalLeft.setPower(0.001);
                motorVerticalRight.setPower(0.001);
                return false;
            }
        }
    }
    public Action exactVertical(int ticksIn, double powerIn) {
        return new exactVertical(ticksIn, powerIn);
    }
}

