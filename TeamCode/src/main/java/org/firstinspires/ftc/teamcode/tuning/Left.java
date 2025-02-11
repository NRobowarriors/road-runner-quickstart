package org.firstinspires.ftc.teamcode.tuning;

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



@Autonomous(name = "Left")
public final class Left extends LinearOpMode {
    private CRServo intakeWheelRight, intakeWheelLeft;
    private Servo intakeTilt, intakeArm, armUpFlowersR, armDownFlowersL, claw, clawWrist;
    private DcMotor motorVerticalLeft, motorVerticalRight;
    private int vertHigh = 2265;
    private int vertMid = 400;
    private int vertLow = 50;
    private double wristDrop = 0.89;
    private double flowerDrop = 0.64;
    private double wristGrab = 0.9;
    private double flowerGrab = 0.00;
    private double flowerMid = 0.2;
    private Pose2d beginPose = new Pose2d(0, 0, 0);
    MecanumDrive drive;
    private Action full;

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
        intakeArm.setPosition(0.09);
        motorVerticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVerticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVerticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorVerticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            drive = new MecanumDrive(hardwareMap, beginPose);
            BuildDriveTrajectories();
            intakeTilt.setPosition(1);
            claw.setPosition(1);
            clawWrist.setPosition(wristGrab);
            armDownFlowersL.setPosition(flowerMid);
            armUpFlowersR.setPosition(flowerMid);
            telemetry.addData("Vetical Ticks", motorVerticalLeft.getCurrentPosition());
            telemetry.update();
            waitForStart();
            Actions.runBlocking(full);
        }  else {
            throw new RuntimeException();
        }
    }

    private void BuildDriveTrajectories()
    {
        Claw claw = new Claw(hardwareMap);
        VerticalLift vertical = new VerticalLift(hardwareMap, telemetry);
        FlowerArm flowerArm = new FlowerArm(hardwareMap);
        Intake intakeSuck = new Intake(hardwareMap);
        Tilt intakeTilt = new Tilt(hardwareMap);
        IntakeArm intakeArm = new IntakeArm(hardwareMap);
        Wrist wrist = new Wrist(clawWrist);
        double bucketX = -23.5;
        double bucketY = 8.25;
        double firstSampleX = -16.25;
        double firstSampleY = 10.25;
        double secondSampleX = -26.5;
        double secondSampleY = 9.25;
        double thirdSampleX = -27.25;
        double thridSampleY = 13.25;
        double ninedy  = Math.toRadians(90);
        double fortyFive = Math.toRadians(45);
        Action firstSampleBucketDrop = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(10, 19), Math.toRadians(-23))
                .build();
        Action firstSampleDrive = drive.actionBuilder(new Pose2d(10,19,Math.toRadians(-23)))
                .strafeToLinearHeading(new Vector2d(16, 16.5), Math.toRadians(-23))
                .build();
        Action secondBucket = drive.actionBuilder(new Pose2d(10,19,Math.toRadians(-23)))
                .strafeToLinearHeading(new Vector2d(12, 22.5), Math.toRadians(-10))
                .build();
        Action secondSampleDrive = drive.actionBuilder(new Pose2d(12,22.5,Math.toRadians(-10)))
                .strafeToLinearHeading(new Vector2d(16, 22.5), Math.toRadians(-10))
                .build();
        Action thridBucket = drive.actionBuilder(new Pose2d(16,22.5,Math.toRadians(-10)))
                .strafeToLinearHeading(new Vector2d(12, 22.5), Math.toRadians(-10))
                .build();
        Action fourthSampleDrive = drive.actionBuilder(new Pose2d(12,22.5,Math.toRadians(-10)))
                .strafeToLinearHeading(new Vector2d(16, 8), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(22, 14), Math.toRadians(45))
                .build();
        Action fourthBucket = drive.actionBuilder(new Pose2d(22,14,Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(12, 22.5), Math.toRadians(-10))
                .build();
        full = new SequentialAction(
                new ParallelAction(
                        firstSampleBucketDrop,
                        vertical.verticalLiftUp(vertHigh, 1),
                        new SequentialAction(
                                new SleepAction(1),
                                flowerArm.flowerArmUp(flowerDrop),
                                new SleepAction(1),
                                claw.clawOpen(),
                                new SleepAction(0.3)
                        ),
                        wrist.wristUp(wristDrop),
                        intakeArm.intakeArmOut()
                ),
                new ParallelAction(
                        firstSampleDrive,
                        intakeTilt.tiltDown(),
                        intakeSuck.intakeSuckIn(),
                        flowerArm.flowerArmDown(0),
                        vertical.exactVertical(0,1),
                        wrist.wristDown(wristGrab)
                ),
                new ParallelAction(
                        intakeSuck.intakeSuckInStop(),
                        secondBucket,
                        intakeTilt.tiltUp(),
                        new SequentialAction(
                        intakeArm.IntakeArmIn(),
                                new SleepAction(0.5),
                                claw.clawClose(),
                                new SleepAction(0.3),
                                new ParallelAction(
                                        vertical.verticalLiftUp(vertHigh, 1),
                                        wrist.wristUp(wristDrop),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                flowerArm.flowerArmUp(flowerDrop),
                                                intakeArm.intakeArmOut(),
                                                new SleepAction(1),
                                                claw.clawOpen(),
                                                new SleepAction(0.3)
                                        )
                                )
                        )
                ),
                new ParallelAction(
                        secondSampleDrive,
                        intakeTilt.tiltDown(),
                        intakeSuck.intakeSuckIn(),
                        flowerArm.flowerArmDown(0),
                        vertical.exactVertical(0,1),
                        wrist.wristDown(wristGrab)
                ),
                new ParallelAction(
                        intakeSuck.intakeSuckInStop(),
                        thridBucket,
                        intakeTilt.tiltUp(),
                        new SequentialAction(
                                intakeArm.IntakeArmIn(),
                                new SleepAction(0.5),
                                claw.clawClose(),
                                new SleepAction(0.3),
                                new ParallelAction(
                                        vertical.verticalLiftUp(vertHigh, 1),
                                        wrist.wristUp(wristDrop),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                flowerArm.flowerArmUp(flowerDrop),
                                                intakeArm.intakeArmOut(),
                                                new SleepAction(1),
                                                claw.clawOpen(),
                                                new SleepAction(0.3)
                                        )
                                )
                        )
                ),
                new ParallelAction(
                        fourthSampleDrive,
                        intakeTilt.tiltDown(),
                        intakeSuck.intakeSuckIn(),
                        flowerArm.flowerArmDown(0),
                        vertical.exactVertical(0,1),
                        wrist.wristDown(wristGrab)
                ),
                new ParallelAction(
                        intakeSuck.intakeSuckInStop(),
                        fourthBucket,
                        intakeTilt.tiltUp(),
                        new SequentialAction(
                                intakeArm.IntakeArmIn(),
                                new SleepAction(0.5),
                                claw.clawClose(),
                                new SleepAction(0.3),
                                new ParallelAction(
                                        vertical.verticalLiftUp(vertHigh, 1),
                                        wrist.wristUp(wristDrop),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                flowerArm.flowerArmUp(flowerDrop),
                                                intakeArm.intakeArmOut(),
                                                new SleepAction(1),
                                                claw.clawOpen(),
                                                new SleepAction(0.3)
                                        )
                                )
                        )
                )
        );

    }
}
