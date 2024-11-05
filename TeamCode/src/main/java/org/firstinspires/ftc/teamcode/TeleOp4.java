package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private ElapsedTime stopwatch = new ElapsedTime();
    //OpenColorV_4 openCv;
    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorVerticalLeft, motorVerticalRight;
        private CRServo intakeWheelRight, intakeWheelLeft;
    private Servo intakeTilt, intakeArm, armUpFlowersR, armDownFlowersL, claw, clawWrist;

    private boolean isRT = false, isLT = false, isRB = false, isLB = false;
    private boolean mecanumDriveMode = true, coastMotors = true;
    private float mecanumStrafe = 0, dominantXJoystick = 0;

    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        motorVerticalLeft = hardwareMap.dcMotor.get("LmotorLift");
        motorVerticalRight = hardwareMap.dcMotor.get("RmotorLift");
        intakeWheelRight = hardwareMap.crservo.get("intakeWheelRight");
        intakeWheelLeft = hardwareMap.crservo.get("intakeWheelLeft");
        intakeTilt = hardwareMap.servo.get("intakeTilt");
        intakeArm = hardwareMap.servo.get("intakeArm");
        armUpFlowersR = hardwareMap.servo.get("armUpFlowersR");
        armDownFlowersL = hardwareMap.servo.get("armDownFlowersL");
        claw = hardwareMap.servo.get("claw");
        clawWrist = hardwareMap.servo.get("clawWrist");

        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        motorVerticalRight.setDirection(DcMotor.Direction.REVERSE);
        //motorVerticalLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Drive Base TeleOp\nInit Opmode");

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorVerticalLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorVerticalRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        if(gamepad1.right_trigger > 0.25){
            isRT = true;
            isLT = false;
            isRB = false;
            isLB = false;
        }
        if(gamepad1.left_trigger > 0.25){
            isRT = false;
            isLT = true;
            isRB = false;
            isLB = false;
        }
        if(gamepad1.right_bumper){
            isRT = false;
            isLT = false;
            isRB = true;
            isLB = false;
        }
        if(gamepad1.left_bumper){
            isRT = false;
            isLT = false;
            isRB = false;
            isLB = true;
        }

        if(gamepad1.b){
            isRT = false;
            isLT = false;
            isRB = false;
            isLB = false;
            motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(isRT) {
            motorVerticalLeft.setTargetPosition(1000);
            motorVerticalRight.setTargetPosition(1000);
            int buffer = 10;
            if (motorVerticalLeft.getCurrentPosition() < motorVerticalLeft.getTargetPosition() - buffer || motorVerticalLeft.getCurrentPosition() > motorVerticalLeft.getTargetPosition() + buffer) {
                motorVerticalLeft.setPower(0.3);
                motorVerticalRight.setPower(0.3);
            } else {
                isRT = false;
                armDownFlowersL.setPosition(0.4);
                armUpFlowersR.setPosition(0.4);
                motorVerticalLeft.setPower(0);
                motorVerticalRight.setPower(0);
            }
        }
            else if (isLT) {
                armUpFlowersR.setPosition(0.1);
                armDownFlowersL.setPosition(0.1);
                intakeArm.setPosition(0.4);
                isLT = false;
            }
            else if (isLB){
                telemetry.addData("time", stopwatch.time());
                telemetry.addData("left", motorVerticalLeft.getCurrentPosition());
                telemetry.addData("right", motorVerticalRight.getCurrentPosition());
                telemetry.addData("left", motorVerticalLeft.getTargetPosition());
                telemetry.addData("right", motorVerticalRight.getTargetPosition());
                telemetry.update();
                if(stopwatch.time() == 0){
                    stopwatch.startTime();
                    motorVerticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorVerticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                armDownFlowersL.setPosition(0.1);
                armUpFlowersR.setPosition(0.1);
                motorVerticalLeft.setTargetPosition(0);
                motorVerticalRight.setTargetPosition(0);
                int buffer = 10;
                if((motorVerticalLeft.getCurrentPosition() < (motorVerticalLeft.getTargetPosition() - buffer)) ||
                        (motorVerticalLeft.getCurrentPosition() > (motorVerticalLeft.getTargetPosition() + buffer))) {
                    motorVerticalLeft.setPower(0.3);
                    motorVerticalRight.setPower(0.3);
                }else{
                    motorVerticalLeft.setPower(0);
                    motorVerticalRight.setPower(0);
                }
                clawWrist.setPosition(0.875);
                claw.setPosition(0.7);
                intakeTilt.setPosition(1);
                intakeArm.setPosition(0);
                if(stopwatch.time() > 1) {
                    armDownFlowersL.setPosition(0);
                    armUpFlowersR.setPosition(0);
                    stopwatch.reset();
                    isLB = false;
                    motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }


        if(!isRT && !isLT && !isRB && !isLB){
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
                intakeArm.setPosition(intakeArm.getPosition() - 0.003);
            } else if (gamepad2.left_stick_y < -0.25) {
                intakeArm.setPosition(intakeArm.getPosition() + 0.003);
            }
            if(gamepad2.right_stick_y > 0.25 ){
                motorVerticalLeft.setPower(-0.4);
                motorVerticalRight.setPower(-0.4);
            }
            else if(gamepad2.right_stick_y < -0.25){
                motorVerticalRight.setPower(0.4);
                motorVerticalLeft.setPower(0.4);
            }
            else{
                motorVerticalRight.setPower(0.05);
                motorVerticalLeft.setPower(0.05);
            }
            if (gamepad2.a) {
                if (armUpFlowersR.getPosition() != 0) {
                    armUpFlowersR.setPosition(armUpFlowersR.getPosition() - 0.005);
                    armDownFlowersL.setPosition(armDownFlowersL.getPosition() - 0.005);
                }
            } else if (gamepad2.y) {
                if (armUpFlowersR.getPosition() != 1) {
                    armUpFlowersR.setPosition(armUpFlowersR.getPosition() + 0.005);
                    armDownFlowersL.setPosition(armDownFlowersL.getPosition() + 0.005);
                }
            }
            if(gamepad2.left_bumper) {
                claw.setPosition(0.7);
            }
            else if(gamepad2.right_bumper){
                claw.setPosition(0.5);
            }

            if(gamepad2.dpad_right){
                clawWrist.setPosition(0.875);
            }
            else if(gamepad2.dpad_left){
                clawWrist.setPosition(1);
            }
        }

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
//        if(gamepad1.dpad_up){
//            Hang up
//        }
//        else if(gamepad1.dpad_down){
//            Hang down
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