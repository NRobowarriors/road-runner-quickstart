package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

//import org.firstinspires.ftc.teamcode.drive.opmode.OpenColorV_4;
//import org.firstinspires.ftc.teamcode.drive.opmode.openCV33;

//import com.qualcomm.robotcore.hardware.DcMotorSimple;




/**
 * All the code aside from importing, goes within a class file - essentially telling Android Studio-
 * to run this using java
 */
@TeleOp(name = "TeleOp4")


public class TeleOp4 extends OpMode {
    //OpenColorV_4 openCv;
    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorLiftLeft, motorLiftRight;
        private CRServo intakeWheelRight, intakeWheelLeft;
    private Servo intakeTilt, intakeArm, armUpFlowersR, armDownFlowersL;

    private boolean mecanumDriveMode = true, coastMotors = true;
    private float mecanumStrafe = 0, dominantXJoystick = 0;

    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        motorLiftLeft = hardwareMap.dcMotor.get("LmotorLift");
        motorLiftRight = hardwareMap.dcMotor.get("RmotorLift");
        intakeWheelRight = hardwareMap.crservo.get("intakeWheelRight");
        intakeWheelLeft = hardwareMap.crservo.get("intakeWheelLeft");
        intakeTilt = hardwareMap.servo.get("intakeTilt");
        intakeArm = hardwareMap.servo.get("intakeArm");
        armUpFlowersR = hardwareMap.servo.get("armUpFlowersR");
        armDownFlowersL = hardwareMap.servo.get("armDownFlowersL");

        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        //motorLiftRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Drive Base TeleOp\nInit Opmode");

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coastMotors = false;
    }

    /**
     * The next 2 lines ends the current state of Opmode, and starts Opmode.loop()
     * This enables control of the robot via the gamepad or starts autonomous functions
     */

    @Override
    public void loop() {

        telemetry.addData("LeftMTR  PWR: ", motorLeft.getPower());
        telemetry.addData("RightMTR PWR: ", motorRight.getPower());
        telemetry.addData("Tilt: ", intakeTilt.getPosition());

        if (abs(gamepad1.left_stick_x) > 0.15 || abs(gamepad1.right_stick_x) > 0.15) {
            dominantXJoystick = (abs(gamepad1.left_stick_x) - abs(gamepad1.right_stick_x));
            mecanumDriveMode = true;
        } else {
            mecanumDriveMode = false;
        }

        if (mecanumDriveMode) {
            if (dominantXJoystick > 0) {
                mecanumStrafe = gamepad1.left_stick_x;
            } else if (dominantXJoystick < 0) {
                mecanumStrafe = gamepad1.right_stick_x;
            }

            motorLeft.setPower((gamepad1.left_stick_y + -mecanumStrafe) / 2);
            motorLeft2.setPower((gamepad1.left_stick_y + mecanumStrafe) / 2);
            motorRight.setPower((gamepad1.right_stick_y + mecanumStrafe) / 2);
            motorRight2.setPower((gamepad1.right_stick_y + -mecanumStrafe) / 2);

        } else {
            drive(gamepad1.left_stick_y * 0.7, gamepad1.right_stick_y * 0.7);

        }
        if (gamepad2.left_trigger > 0.25) {
            intakeWheelLeft.setPower(0.5);
            intakeWheelRight.setPower(-0.5);
        } else if (gamepad2.right_trigger > 0.25) {
            intakeWheelLeft.setPower(-0.5);
            intakeWheelRight.setPower(0.5);
        } else {
            intakeWheelLeft.setPower(0);
            intakeWheelRight.setPower(0);

        }
        if (gamepad2.dpad_up) {
            intakeTilt.setPosition(1);
        }
        if (gamepad2.dpad_down) {
            intakeTilt.setPosition(0.75);
        }
        if (gamepad2.left_stick_y > 0.25) {
            intakeArm.setPosition(intakeArm.getPosition() - 0.001);
        } else if (gamepad2.left_stick_y < -0.25) {
            intakeArm.setPosition(intakeArm.getPosition() + 0.001);
        }
        if(gamepad2.right_stick_y > 0.25){
            motorLiftRight.setPower(0.1);
            motorLiftLeft.setPower(0.1);
        }
         else if(gamepad2.right_stick_y < -0.25){
             motorLiftRight.setPower(-0.1);
             motorLiftLeft.setPower(-0.1);
        }
         else{
            motorLiftRight.setPower(0);
            motorLiftLeft.setPower(0);
        }
        if (gamepad2.a) {
            if (armUpFlowersR.getPosition() != 0) {
                armUpFlowersR.setPosition(armUpFlowersR.getPosition() - 0.001);
                armDownFlowersL.setPosition(armDownFlowersL.getPosition() - 0.001);
            }
        } else if (gamepad2.y) {
            if (armUpFlowersR.getPosition() != 1) {
                armUpFlowersR.setPosition(armUpFlowersR.getPosition() + 0.001);
                armDownFlowersL.setPosition(armDownFlowersL.getPosition() + 0.001);
            }
        }
//        if(gamepad2.left_bumper){
//            if (claw.getPosition() > getNumber) {
//                claw.setPosition(claw.getPosition + 000000.00000000000000000000000000010);
//            }
//        else if(claw.getPosition() < -getNumber){
//                claw.setPosition(claw.getPosition - 000000.00000000000000000000000000010);
//            }
//        if(gamepad1.dpad_up){
//            Hang up
//        }
//        else if(gamepad1.dpad_down){
//            Hang down
//        }
//        if (gamepad2.left_trigger > 0.25) {
//            armUpFlowersR.setPosition(0.1);
//            armDownFlowersL.setPosition(0.1);
//            intakeArm.setPosition(0.01);
//            intakeTilt.setPosition(-0.1);
//        }
//        else if (gamepad2.left_bumper){
//            claw.setPosition(1);
//            intakeTilt.setPosition(0);
//            intakeArm.setPosition(0);
//            verticalArm.setPosition(0);
//            armDownFlowersL.setPosition(0);
//            armUpFlowersR.setPosition(0);
//            claw.setPosition(0);
//        }
//        if(gamepad1.right_trigger){
//            armDownFlowersL.setPosition(0.1);
//            armUpFlowersR.setPosition(0.1);
//            verticalArm.setPosition(0.1);
//            claw.setPosition(0.1);
//        }

//end of loop opmode programing
    }

        @Override
        public void stop () {
            telemetry.clearAll();
            telemetry.addLine("Stopped");
        }

        public void drive ( double left, double right){
            motorLeft.setPower(left);
            motorLeft2.setPower(left);
            motorRight.setPower(right);
            motorRight2.setPower(right);

        }


}