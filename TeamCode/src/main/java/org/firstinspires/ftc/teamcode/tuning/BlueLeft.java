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



@Autonomous(name = "BlueLeft")
public final class BlueLeft extends LinearOpMode {
    private CRServo intakeWheelRight, intakeWheelLeft;
    private Servo intakeTilt, intakeArm, armUpFlowersR, armDownFlowersL, claw, clawWrist;
    private DcMotor motorVerticalLeft, motorVerticalRight;
    private int vertHigh = 1800;
    private int vertMid = 300;
    private int vertLow = 50;
    private double wristDrop = 0.89;
    private double flowerDrop = 0.6;
    private double wristGrab = 0.9;
    private double flowerGrab = 0.00;
    private double flowerMid = 0.5;
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
            clawWrist.setPosition(0.84);
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

        Action firstSampleBucketDrop = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-26, 15), Math.toRadians(42))
                .strafeToLinearHeading(new Vector2d(-32, 9), Math.toRadians(42))
                .build();
        Action secondSampleDriveToSample = drive.actionBuilder(new Pose2d(-32, 9, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-20, 12), Math.toRadians(93))
                .build();
        Action secondSamplePickUpDrive = drive.actionBuilder(new Pose2d(-20, 12, Math.toRadians(93)))
                .splineToConstantHeading(new Vector2d(-20, 19), Math.toRadians(93))
                .build();
        Action secondSampleTurnToBucket = drive.actionBuilder(new Pose2d(-22,19, Math.toRadians(93))).
                turn( Math.toRadians(-48))
                .build();
        Action secondSampleDriveToBucket = drive.actionBuilder(new Pose2d(-22,19, Math.toRadians(45)))
                .splineToConstantHeading(new Vector2d(-30, 18), Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-37, 14), Math.toRadians(45))
                .build();
        Action thirdSampleDriveToSample = drive.actionBuilder(new Pose2d(-33, 10, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-33, 11), Math.toRadians(84))
                .build();
        Action thirdSamplePickUpDrive = drive.actionBuilder(new Pose2d(-33, 15, Math.toRadians(81)))
                .splineToConstantHeading(new Vector2d(-33, 23), Math.toRadians(84))
                .build();
        Action thirdSampleTurnToBucket = drive.actionBuilder(new Pose2d(-37,23, Math.toRadians(81)))
                .turn( Math.toRadians(-45))
                .build();
        Action thirdSampleDriveToBucket = drive.actionBuilder(new Pose2d(-38,22, Math.toRadians(45)))
                .splineToConstantHeading(new Vector2d(-29, 15), Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-36, 7), Math.toRadians(45))
                .build();
        Action fourthSampleDriveToSample = drive.actionBuilder(new Pose2d(-30, 5, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-34, 10), Math.toRadians(110))
                .build();
        Action fourthSamplePickUpDrive = drive.actionBuilder(new Pose2d(-34, 10, Math.toRadians(110)))
                .splineToConstantHeading(new Vector2d(-36, 17), Math.toRadians(110))
                .build();
        Action fourthSampleTurnToBucket = drive.actionBuilder(new Pose2d(-36,17, Math.toRadians(110)))
                .turn( Math.toRadians(-90))
                .build();
        Action fourthSampleDriveToBucket = drive.actionBuilder(new Pose2d(-36,17, Math.toRadians(45)))
                .splineToConstantHeading(new Vector2d(-25, 17), Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-22, 4), Math.toRadians(45))
                .build();
        Action moveToHang = drive.actionBuilder(new Pose2d(-32,4, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-10, 51), Math.toRadians(180))
                .build();
        Action moveBackToHang = drive.actionBuilder(new Pose2d(-10,51, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(0, 51), Math.toRadians(180))
                .build();
        full = new SequentialAction(
                //First drive
                new ParallelAction(
                        firstSampleBucketDrop,
                        vertical.verticalLiftUp(vertHigh, 0.7),
                        flowerArm.flowerArmUp(flowerDrop),
                        wrist.wristUp(wristDrop)
                ),
                claw.clawOpen(),
                new SleepAction(0.3),
                //To 2nd sample
                new ParallelAction(
                        vertical.verticalLiftDown(vertMid, 0.5),
                        flowerArm.flowerArmDown(flowerMid),
                        wrist.wristDown(wristGrab),
                        secondSampleDriveToSample,
                        intakeArm.intakeArmOut()
                ),
                new SleepAction(0.35),
                intakeSuck.intakeSuckIn(),
                //Pick up 2nd Sample
                intakeTilt.tiltDown(),
                secondSamplePickUpDrive,
                new SleepAction(0.4),
                intakeSuck.intakeSuckInStop(),
                //Bring sample to Center
                new ParallelAction(
                        intakeTilt.tiltUp(),
                        intakeArm.IntakeArmIn(),
                        secondSampleTurnToBucket
                ),
                vertical.verticalLiftDown(vertLow,0.5),
                flowerArm.flowerArmDown(flowerGrab),
                new SleepAction(1),
                claw.clawClose(),
                new SleepAction(0.5),
                //Second sample in basket
                new ParallelAction(
                        secondSampleDriveToBucket,
                        flowerArm.flowerArmUp(flowerDrop),
                        vertical.verticalLiftUp(vertHigh,0.8),
                        wrist.wristUp(wristDrop)
                ),
                new SleepAction(0.5),
                claw.clawOpen(),
                new SleepAction(0.5),
                //Drive to 3rd sample
                new ParallelAction(
                        vertical.verticalLiftDown(vertMid, 0.5),
                        flowerArm.flowerArmDown(flowerGrab),
                        wrist.wristDown(wristGrab),
                        thirdSampleDriveToSample,
                        intakeArm.intakeArmOut()
                ),
                intakeSuck.intakeSuckIn(),
                new SleepAction(0.35),
                intakeTilt.tiltDown(),
                thirdSamplePickUpDrive,
                new SleepAction(0.2),
                intakeSuck.intakeSuckInStop(),
                //Sample to Center
                new ParallelAction(
                        intakeTilt.tiltUp(),
                        intakeArm.IntakeArmIn(),
                        thirdSampleTurnToBucket
                ),
                vertical.verticalLiftDown(vertLow,0.5),
                new SleepAction(1),
                claw.clawClose(),
                new SleepAction(0.5),
                //3rd Sample to basket
                new ParallelAction(
                        thirdSampleDriveToBucket,
                        flowerArm.flowerArmUp(flowerDrop),
                        vertical.verticalLiftUp(vertHigh,0.8),
                        wrist.wristUp(wristDrop)
                ),
                new SleepAction(0.5),
                claw.clawOpen(),
                new SleepAction(0.5),
                new ParallelAction(
                        vertical.verticalLiftDown(vertMid, 0.5),
                        flowerArm.flowerArmDown(flowerGrab),
                        wrist.wristDown(wristGrab),
                        fourthSampleDriveToSample,
                        intakeArm.intakeArmOut(),
                        intakeSuck.intakeSuckIn()
                ),
                new SleepAction(0.35),
                intakeTilt.tiltDown(),
                fourthSamplePickUpDrive,
                new SleepAction(0.4),
                intakeSuck.intakeSuckInStop(),
                //Sample to Center
                new ParallelAction(
                        intakeTilt.tiltUp(),
                        intakeArm.IntakeArmIn(),
                        fourthSampleTurnToBucket
                ),
                intakeArm.IntakeArmIn(),
                vertical.verticalLiftDown(vertLow,0.5),
                new SleepAction(1),
                claw.clawClose(),
                new SleepAction(0.5),
                //4rd Sample to basket
                new ParallelAction(
                        fourthSampleDriveToBucket,
                        flowerArm.flowerArmUp(flowerDrop),
                        vertical.verticalLiftUp(vertHigh,0.8),
                        wrist.wristUp(wristDrop)
                ),
                new SleepAction(0.5),
                claw.clawOpen(),
                new SleepAction(0.5),
                new ParallelAction(
                        vertical.verticalLiftDown(vertLow, 0.5),
                        flowerArm.flowerArmDown(0.4),
                        wrist.wristDown(wristGrab),
                        moveToHang
                ),
                moveBackToHang,
                flowerArm.flowerArmUp(0.7),
                new SleepAction(1)
        );
    }
}
