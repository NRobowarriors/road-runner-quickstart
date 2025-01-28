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
    private int vertHigh = 1760;
    private int vertMid = 300;
    private int vertLow = 50;
    private double wristDrop = 0.89;
    private double flowerDrop = 0.64;
    private double wristGrab = 0.84;
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
        double bucketX = -23.5;
        double bucketY = 8.25;
        double firstSampleX = -16.25;
        double firstSampleY = 10.25;
        double secondSampleX = -26.5;
        double secondSampleY = 10.25;
        double thirdSampleX = -27.25;
        double thridSampleY = 13.25;
        double ninedy  = Math.toRadians(90);
        double fortyFive = Math.toRadians(45);
        Action firstSampleBucketDrop = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(bucketX, bucketY), fortyFive)
                .build();
        Action secondSampleDriveToSample = drive.actionBuilder(new Pose2d(bucketX, bucketY, fortyFive))
                .strafeToLinearHeading(new Vector2d(firstSampleX, firstSampleY), ninedy)
                .build();
        Action secondSamplePickUpDrive = drive.actionBuilder(new Pose2d(firstSampleX, firstSampleY, ninedy))
                .splineToConstantHeading(new Vector2d(firstSampleX, firstSampleY + 4), ninedy)
                .build();
        Action secondSampleTurnToBucket = drive.actionBuilder(new Pose2d(firstSampleX,firstSampleY + 4, ninedy))
                .turn(-fortyFive)
                .build();
        Action secondSampleDriveToBucket = drive.actionBuilder(new Pose2d(firstSampleX,firstSampleY + 4, fortyFive))
                .strafeToLinearHeading(new Vector2d(bucketX, bucketY), fortyFive)
                .build();
        Action thirdSampleDriveToSample = drive.actionBuilder(new Pose2d(bucketX, bucketY, fortyFive))
                .strafeToLinearHeading(new Vector2d(secondSampleX, secondSampleY), ninedy)
                .build();
        Action thirdSamplePickUpDrive = drive.actionBuilder(new Pose2d(secondSampleX, secondSampleY, ninedy))
                .splineToConstantHeading(new Vector2d(secondSampleX, secondSampleY + 4), ninedy)
                .build();
        Action thirdSampleTurnToBucket = drive.actionBuilder(new Pose2d(secondSampleX,secondSampleY, ninedy))
                .turn((-fortyFive))
                .build();
        Action thirdSampleDriveToBucket = drive.actionBuilder(new Pose2d(secondSampleX,secondSampleY, fortyFive))
                .strafeToLinearHeading(new Vector2d(bucketX, bucketY), fortyFive)
                .build();
        Action fourthSampleDriveToSample = drive.actionBuilder(new Pose2d(bucketX, bucketY, fortyFive))
                .strafeToLinearHeading(new Vector2d(thirdSampleX, thridSampleY), Math.toRadians(112))
                .build();
        Action fourthSamplePickUpDrive = drive.actionBuilder(new Pose2d(thirdSampleX, thridSampleY, Math.toRadians(112)))
                .splineToConstantHeading(new Vector2d(thirdSampleX, thridSampleY + 4), Math.toRadians(112))
                .build();
        Action fourthSampleTurnToBucket = drive.actionBuilder(new Pose2d(thirdSampleX,thridSampleY + 4, Math.toRadians(110)))
                .turn( Math.toRadians(-65))
                .build();
        Action fourthSampleDriveToBucket = drive.actionBuilder(new Pose2d(thirdSampleX,thridSampleY + 4, fortyFive))
                .strafeToLinearHeading(new Vector2d(bucketX, bucketY), fortyFive)
                .build();
        Action moveToHang = drive.actionBuilder(new Pose2d(bucketX,bucketY, fortyFive))
                .strafeToLinearHeading(new Vector2d(0, 54), Math.toRadians(180))
                .build();
        Action moveBackToHang = drive.actionBuilder(new Pose2d(0,54, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(6, 54), Math.toRadians(180))
                .build();
        full = new SequentialAction(
                //First drive
                new ParallelAction(
                        firstSampleBucketDrop,
                        vertical.verticalLiftUp(vertHigh, 0.7),
                        new SequentialAction(
                                new SleepAction(0.75),
                                flowerArm.flowerArmUp(flowerDrop)
                        ),
                        wrist.wristUp(wristDrop)
                ),
                claw.clawOpen(),
                new SleepAction(0.5),
                //To 2nd sample
                new ParallelAction(
                        vertical.verticalLiftDown(vertMid, 0.5),
                        flowerArm.flowerArmDown(flowerMid),
                        wrist.wristDown(wristGrab),
                        intakeArm.intakeArmOut(),
                        new SequentialAction(
                                secondSampleDriveToSample,
                                intakeSuck.intakeSuckIn(),
                                new SleepAction(0.2),
                                intakeTilt.tiltDown(),
                                secondSamplePickUpDrive
                        )
                ),
                //Pick up 2nd Sample
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
                        new SequentialAction(
                                new SleepAction(0.75),
                                flowerArm.flowerArmUp(flowerDrop)
                        ),
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
                        intakeArm.intakeArmOut(),
                        new SequentialAction(
                                thirdSampleDriveToSample,
                                intakeSuck.intakeSuckIn(),
                                new SleepAction(0.2),
                                intakeTilt.tiltDown(),
                                thirdSamplePickUpDrive
                        )
                ),
                new SleepAction(0.2),
                intakeSuck.intakeSuckInStop(),

                //Sample to Center
                new ParallelAction(
                        intakeTilt.tiltUp(),
                        intakeArm.IntakeArmIn(),
                        thirdSampleTurnToBucket
                ),
                vertical.verticalLiftDown(vertLow,0.5),
                new SleepAction(0.5),
                claw.clawClose(),
                new SleepAction(0.5),
                //3rd Sample to basket
                new ParallelAction(
                        thirdSampleDriveToBucket,
                        new SequentialAction(
                                new SleepAction(0.75),
                                flowerArm.flowerArmUp(flowerDrop)
                        ),
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
                        intakeArm.intakeArmOut(),
                        new SequentialAction(
                                fourthSampleDriveToSample,
                                intakeSuck.intakeSuckIn(),
                                new SleepAction(0.2),
                                intakeTilt.tiltDown(),
                                fourthSamplePickUpDrive
                        )
                ),
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
                new SleepAction(0.5),
                claw.clawClose(),
                new SleepAction(0.5),
                //4rd Sample to basket
                new ParallelAction(
                        fourthSampleDriveToBucket,
                        new SequentialAction(
                                new SleepAction(0.75),
                                flowerArm.flowerArmUp(flowerDrop)
                        ),
                        vertical.verticalLiftUp(vertHigh,0.8),
                        wrist.wristUp(wristDrop)
                ),
                new SleepAction(0.5),
                claw.clawOpen(),
                new SleepAction(0.5),
                new ParallelAction(
                        vertical.verticalLiftDown(vertLow, 0.5),
                        flowerArm.flowerArmDown(0.5),
                        wrist.wristDown(wristGrab),
                        moveToHang
                ),
                moveBackToHang,
                flowerArm.flowerArmUp(0.6),
                new SleepAction(1)

        );
    }
}
