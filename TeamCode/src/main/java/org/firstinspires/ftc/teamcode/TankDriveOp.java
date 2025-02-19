package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
@TeleOp(name = "Tank Drive")
public class TankDriveOp extends OpMode {
    //private ElapsedTime stopwatch = new ElapsedTime();
    //OpenColorV_4 openCv;
    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorVerticalLeft, motorVerticalRight, hang;
        private CRServo intakeWheelRight, intakeWheelLeft;
    private Servo intakeTilt, intakeArm, armUpFlowersR, armDownFlowersL, claw, clawWrist;

    private boolean ishc = false, iswall = false, ishb = false, islb = false, isdown = false;
    private boolean mecanumDriveMode = true, coastMotors = true;
    private float mecanumStrafe = 0, dominantXJoystick = 0;
    int buffer = 10;
    private double clawWristDown = 0.82;// Claw Wrist Up = 1
    private double flowerArmMin = 0.14; // max = 1
    private double flowerArmMid = 0.2;
    private double clawOpen = 0.5;
    private double clawClose = 1;//0.75;
    boolean isThere = true;
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
        intakeArm.setPosition(0.09);
        armUpFlowersR = hardwareMap.servo.get("armUpFlowersR");
        armDownFlowersL = hardwareMap.servo.get("armDownFlowersL");
        claw = hardwareMap.servo.get("claw");
        clawWrist = hardwareMap.servo.get("clawWrist");
        hang = hardwareMap.dcMotor.get("hang");

        motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setTargetPosition(0);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        clawWrist.setPosition(0.84);
    }

    /**
     * The next 2 lines ends the current state of Opmode, and starts Opmode.loop()
     * This enables control of the robot via the gamepad or starts autonomous functions
     */

    @Override
    public void loop() {
        telemetry.addData("ishc", ishc);
        telemetry.addData("BackRight", motorRight2.getCurrentPosition());
        telemetry.addData("FrontRight", motorRight.getCurrentPosition());
        telemetry.addData("BackLeft", motorLeft2.getCurrentPosition());
        telemetry.addData("wrist", clawWrist.getPosition());
        telemetry.addData("LeftMTR  PWR: ", motorLeft.getPower());
        telemetry.addData("RightMTR PWR: ", motorRight.getPower());
        telemetry.addData("Tilt: ", intakeTilt.getPosition());
        //telemetry.addData("time", stopwatch.time());
        telemetry.addData("left", motorVerticalLeft.getCurrentPosition());
        telemetry.addData("right", motorVerticalRight.getCurrentPosition());
        telemetry.addData("leftTarget", motorVerticalLeft.getTargetPosition());
        telemetry.addData("rightTarget", motorVerticalRight.getTargetPosition());
        telemetry.addData("flowerR", armUpFlowersR.getPosition());
        //telemetry.addData("motorVerticalLeft",motorVerticalLeft.getCurrentPosition());
        //telemetry.addData("motorVerticalRight",motorVerticalRight.getCurrentPosition());
        //telemetry.addData("hang",hang.getCurrentPosition());
        telemetry.update();
        //armUpFlowersR.setPosition(flowerArmMid);
        //armDownFlowersL.setPosition(flowerArmMid);
        if(gamepad2.y){
            ishc = true;
            iswall = false;
            ishb = false;
            islb = false;
            isdown = false;
        }
        if(gamepad2.a){
            ishc = false;
            iswall = true;
            ishb = false;
            islb = false;
            isdown = false;
        }
        if(gamepad2.dpad_up){
            ishc = false;
            iswall = false;
            ishb = true;
            islb = false;
            isdown = false;
        }
        if(gamepad2.dpad_left){
            ishc = false;
            iswall = false;
            ishb = false;
            islb = true;
            isdown = false;
        }
        if(gamepad2.dpad_down){
            ishc = false;
            iswall = false;
            ishb = false;
            islb = false;
            isdown = true;
        }
        if(gamepad2.b){
            ishc = false;
            iswall = false;
            ishb = false;
            islb = false;
            isdown = false;
            motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(ishc) {
            motorVerticalLeft.setTargetPosition(660);
            motorVerticalRight.setTargetPosition(660);
            motorVerticalLeft.setPower(0.001);
            motorVerticalRight.setPower(0.001);
            armDownFlowersL.setPosition(0.6572);
            armUpFlowersR.setPosition(0.6572);
            clawWrist.setPosition(0.9);
            motorVerticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorVerticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (motorVerticalLeft.getCurrentPosition() < motorVerticalLeft.getTargetPosition() - buffer ||
                    motorVerticalLeft.getCurrentPosition() > motorVerticalLeft.getTargetPosition() + buffer) {
                motorVerticalLeft.setPower(1);
                motorVerticalRight.setPower(1);
            }
            else {
                motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ishc = false;
            }
        }
        else if (ishb) {
            armDownFlowersL.setPosition(0.64);
            armUpFlowersR.setPosition(0.64);
            clawWrist.setPosition(0.9);
            motorVerticalLeft.setTargetPosition(2560);
            motorVerticalRight.setTargetPosition(2560);
            motorVerticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorVerticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (motorVerticalLeft.getCurrentPosition() < motorVerticalLeft.getTargetPosition()  - buffer||
                    motorVerticalLeft.getCurrentPosition() > motorVerticalLeft.getTargetPosition() + buffer) {
                motorVerticalLeft.setPower(1);
                motorVerticalRight.setPower(1);
            }
            else {
                motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ishb = false;
            }
        }
        else if (islb) {
            armDownFlowersL.setPosition(0.64);
            armUpFlowersR.setPosition(0.64);
            clawWrist.setPosition(0.9);
            motorVerticalLeft.setTargetPosition(1160);
            motorVerticalRight.setTargetPosition(1160);
            motorVerticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorVerticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (motorVerticalLeft.getCurrentPosition() < motorVerticalLeft.getTargetPosition() - buffer ||
                    motorVerticalLeft.getCurrentPosition() > motorVerticalLeft.getTargetPosition() + buffer) {
                motorVerticalLeft.setPower(1);
                motorVerticalRight.setPower(1);
            }
            else {
                motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                islb = false;
            }
        }
        else if (iswall) {
            armDownFlowersL.setPosition(1);
            armUpFlowersR.setPosition(1);
            motorVerticalLeft.setTargetPosition(500);
            motorVerticalRight.setTargetPosition(500);
            if(motorVerticalLeft.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                motorVerticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorVerticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (motorVerticalLeft.getCurrentPosition() < motorVerticalLeft.getTargetPosition() - buffer ||
                        motorVerticalLeft.getCurrentPosition() > motorVerticalLeft.getTargetPosition() + buffer) {
                    motorVerticalLeft.setPower(1);
                    motorVerticalRight.setPower(1);
                }
                else {
                    motorVerticalLeft.setPower(0.001);
                    motorVerticalRight.setPower(0.001);
                    clawWrist.setPosition(0.9);
                    motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    iswall = false;
                }
        }
        else if (isdown) {
            armDownFlowersL.setPosition(0);
            armUpFlowersR.setPosition(0);
            clawWrist.setPosition(0.84);
            //claw.setPosition(1);
            motorVerticalLeft.setTargetPosition(0);
            motorVerticalRight.setTargetPosition(0);
            motorVerticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorVerticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (motorVerticalLeft.getCurrentPosition() < motorVerticalLeft.getTargetPosition()||
                    motorVerticalLeft.getCurrentPosition() > motorVerticalLeft.getTargetPosition() + 5 ||
                    motorVerticalRight.getCurrentPosition() < motorVerticalRight.getTargetPosition()||
                    motorVerticalRight.getCurrentPosition() > motorVerticalRight.getTargetPosition() + 5) {
                motorVerticalLeft.setPower(1);
                motorVerticalRight.setPower(1);
            } else {
                motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isdown = false;
            }
        }

        if(!ishb && !iswall && !islb && !ishc && !isdown){
            if (gamepad2.left_trigger > 0.25) {
                intakeWheelLeft.setPower(0.8);
                intakeWheelRight.setPower(-0.8);
            } else if (gamepad2.right_trigger > 0.25) {
                intakeWheelLeft.setPower(-0.8);
                intakeWheelRight.setPower(0.8);
            } else {
                intakeWheelLeft.setPower(0);
                intakeWheelRight.setPower(0);

            }

            if(gamepad2.left_stick_y > 0.25 && motorVerticalLeft.getCurrentPosition() > 5){
                motorVerticalLeft.setPower(-1);
                motorVerticalRight.setPower(-1);
            }
            else if(gamepad2.left_stick_y < -0.25){
                motorVerticalRight.setPower(1);
                motorVerticalLeft.setPower(1);
            }
            else{
                motorVerticalRight.setPower(0.001);
                motorVerticalLeft.setPower(0.001);
            }

            if (gamepad2.right_stick_y > 0.25) {
                if (armUpFlowersR.getPosition() > 0) {
                    armUpFlowersR.setPosition(armUpFlowersR.getPosition() - 0.028);
                    armDownFlowersL.setPosition(armDownFlowersL.getPosition() - 0.028);
                }
            } else if (gamepad2.right_stick_y < -0.25) {
                if (armUpFlowersR.getPosition() < 1.0) {
                    armUpFlowersR.setPosition(armUpFlowersR.getPosition() + 0.028);
                    armDownFlowersL.setPosition(armDownFlowersL.getPosition() + 0.028);
                }
            }

            if(gamepad2.left_bumper) {
                claw.setPosition(clawOpen);
            }
            else if(gamepad2.right_bumper){
                claw.setPosition(clawClose);
            }

            if(gamepad2.dpad_up){
                if (clawWrist.getPosition() < 0.9) {
                    clawWrist.setPosition(clawWrist.getPosition() + 0.008);
                }
            }
            else if(gamepad2.dpad_down){
                if (clawWrist.getPosition() > 0.8) {
                    clawWrist.setPosition(clawWrist.getPosition() - 0.008);
                }
            }
            if(gamepad1.dpad_up){
                isThere = false;
                hang.setTargetPosition(-3640);
            } else if(gamepad1.dpad_down){
                isThere = false;
                hang.setTargetPosition(-2173);
            }else if(gamepad1.dpad_left){
                isThere = false;
                hang.setTargetPosition(0);
            }
            if(isThere){
                if(hang.getCurrentPosition() != hang.getTargetPosition()){
                    hang.setPower(1);
                }
            }
        }
        if (gamepad1.left_bumper) {
            intakeTilt.setPosition(1);
        }
        if (gamepad1.right_bumper) {
            intakeTilt.setPosition(0.75);
        }
//            if (gamepad1.right_trigger > 0.25) {intakeArm.setPosition(intakeArm.getPosition() + 0.008);}
//             else if (gamepad1.left_trigger > 0.25) {intakeArm.setPosition(intakeArm.getPosition() - 0.008);}

        if (gamepad1.right_trigger > 0.25 && intakeArm.getPosition() < 0.34) {
            intakeArm.setPosition(intakeArm.getPosition() + 0.02);
        } else if (gamepad1.left_trigger > 0.25 && intakeArm.getPosition() > 0.09) {
            intakeArm.setPosition(intakeArm.getPosition() - 0.02);
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

            motorLeft.setPower((gamepad1.left_stick_y + -mecanumStrafe));
            motorLeft2.setPower((gamepad1.left_stick_y + mecanumStrafe));
            motorRight.setPower((gamepad1.right_stick_y + mecanumStrafe));
            motorRight2.setPower((gamepad1.right_stick_y + -mecanumStrafe));

        } else {
            drive(gamepad1.left_stick_y * 1, gamepad1.right_stick_y * 1);

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