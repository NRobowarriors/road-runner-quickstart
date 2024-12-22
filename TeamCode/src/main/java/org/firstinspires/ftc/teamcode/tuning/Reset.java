/*package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Claw;
import org.firstinspires.ftc.teamcode.FlowerArm;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.IntakeArm;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Tilt;
import org.firstinspires.ftc.teamcode.VerticalLift;
import org.firstinspires.ftc.teamcode.Wrist;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;



@Autonomous(name = "Reset")
public final class Reset extends LinearOpMode {
    private CRServo intakeWheelRight, intakeWheelLeft;
    private Servo intakeTilt, intakeArm, armUpFlowersR, armDownFlowersL, claw, clawWrist;
    private DcMotor motorVerticalLeft, motorVerticalRight;
    private Pose2d beginPose = new Pose2d(0, 0, 0);
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
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
        motorVerticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVerticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            drive = new MecanumDrive(hardwareMap, beginPose);
            waitForStart();
        }  else {
            throw new RuntimeException();
        }
        if(!timerReset) {
            stopwatch.reset();
            timerReset = true;
        }
        motorVerticalLeft.setTargetPosition(100);
        motorVerticalRight.setTargetPosition(100);
        if(motorVerticalLeft.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            motorVerticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorVerticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(stopwatch.time() == 0){
            stopwatch.startTime();
        }

        if((motorVerticalLeft.getCurrentPosition() < (motorVerticalLeft.getTargetPosition() - 10)) ||
                (motorVerticalLeft.getCurrentPosition() > (motorVerticalLeft.getTargetPosition() + 10))) {
            motorVerticalLeft.setPower(0.3);
            motorVerticalRight.setPower(0.3);
        }else{
            motorVerticalLeft.setPower(0);
            motorVerticalRight.setPower(0);
        }
        clawWrist.setPosition(0.822);
        claw.setPosition(clawOpen);
        intakeTilt.setPosition(1);
        intakeArm.setPosition(0);
        armDownFlowersL.setPosition(flowerArmMin);
        armUpFlowersR.setPosition(flowerArmMin);
        if(stopwatch.time() > 1.0) {
            motorVerticalLeft.setTargetPosition(0);
            motorVerticalRight.setTargetPosition(0);
            motorVerticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorVerticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if((motorVerticalLeft.getCurrentPosition() < (motorVerticalLeft.getTargetPosition() - 10)) ||
                    (motorVerticalLeft.getCurrentPosition() > (motorVerticalLeft.getTargetPosition() + 10))) {
                motorVerticalLeft.setPower(0.3);
                motorVerticalRight.setPower(0.3);
            }else{
                motorVerticalLeft.setPower(0);
                motorVerticalRight.setPower(0);
                motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                timerReset = false;
            }

        }
    }
    }

}

 */
