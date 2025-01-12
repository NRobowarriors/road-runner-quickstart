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



@Autonomous(name = "HailRight")
public final class HailRight extends LinearOpMode {
    private CRServo intakeWheelRight, intakeWheelLeft;
    private Servo intakeTilt, intakeArm, armUpFlowersR, armDownFlowersL, claw, clawWrist;
    private DcMotor motorVerticalLeft, motorVerticalRight;
    private int vertSpeciman = 495;
    private int vertGrab = 416;
    private int vertLow = 357;
    private double wristDrop = 0.84;
    private double flowerDrop = 1;
    private double wristGrab = 0.9;
    private double flowerGrab = 0.6572;
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
        intakeArm.IntakeArmIn();
        intakeTilt.tiltUp();
        double specimanX = -27.5;
        double specimanY = 0;
        double HumanPlayerX = -6.5;
        double HumanPlayerY = 26.5;
        double secondSampleDriveX = -46;
        double secondSampleDriveY = 40;
        double sampleHumanX = -10;
        double shiftAtTopY = 50;
//        double thridSampleY = 13.25;
//        double ninedy  = Math.toRadians(90);
        double zero = Math.toRadians(0);
        double turn = Math.toRadians(180);
        double ninety = Math.toRadians(90);
        Action firstSpeciman = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(specimanX, specimanY), zero)
                .build();
        Action driveToFirstSample = drive.actionBuilder(new Pose2d(specimanX, specimanY, zero))
                .splineTo(new Vector2d(-23, 16), ninety)
                .splineTo(new Vector2d(-40,26),turn)
                .splineToConstantHeading(new Vector2d(-50, 38), turn)
                .splineToConstantHeading(new Vector2d(-16,38),turn)
                //.lineToX(-12)
                .build();
//        Action driveToHumanPlayer = drive.actionBuilder(new Pose2d(secondSampleDriveX, secondSampleDriveY, turn))
//                .strafeToLinearHeading(new Vector2d(sampleHumanX, secondSampleDriveY ), turn)
//                .build();
        Action driveToSecondSample = drive.actionBuilder(new Pose2d(-16,39, turn))
                .splineToConstantHeading(new Vector2d(-40, 39), turn)
                .splineToConstantHeading(new Vector2d(-50, 47), turn)
                .splineToConstantHeading(new Vector2d(-16, 47), turn)
                //.lineToX(-12)
                .build();
        Action driveToThirdSample = drive.actionBuilder(new Pose2d(-16, 48, turn))
                .splineToConstantHeading(new Vector2d(-40, 48), turn)
                .splineToConstantHeading(new Vector2d(-50,54.5),turn)
                //.splineToConstantHeading(new Vector2d(-16,53.5),turn)
                .lineToX(-6.5)
                .build();
        Action secondSpeciman = drive.actionBuilder(new Pose2d(-6.5, 54.5, turn))
                //.strafeToLinearHeading(new Vector2d(specimanX, specimanY - 3), zero)
                //.splineTo(new Vector2d(specimanX, specimanY - 3), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(specimanX, specimanY - 3, zero), Math.toRadians(135))
                .build();
        Action driveToHumanPlayer = drive.actionBuilder(new Pose2d(specimanX,specimanY - 3, zero))
                .strafeToLinearHeading(new Vector2d(HumanPlayerX, HumanPlayerY), turn)
                .build();
        Action thirdSpeciman = drive.actionBuilder(new Pose2d(HumanPlayerX, HumanPlayerY, turn))
                .strafeToLinearHeading(new Vector2d(specimanX, specimanY - 6), zero)
                .build();
        Action driveToHumanPlayer2 = drive.actionBuilder(new Pose2d(specimanX,specimanY - 6, zero))
                .strafeToLinearHeading(new Vector2d(HumanPlayerX, HumanPlayerY), turn)
                .build();
        Action fourthSpeciman = drive.actionBuilder(new Pose2d(HumanPlayerX, HumanPlayerY, turn))
                .strafeToLinearHeading(new Vector2d(specimanX, specimanY - 9), zero)
                .build();
        Action driveToHumanPlayer3 = drive.actionBuilder(new Pose2d(specimanX,specimanY - 9, zero))
                .strafeToLinearHeading(new Vector2d(HumanPlayerX, HumanPlayerY), turn)
                .build();
        full = new SequentialAction(
                //First drive
                new ParallelAction(
                        vertical.exactVertical(vertSpeciman, 0.6),
                        intakeArm.IntakeArmIn(),
                        intakeTilt.tiltUp(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                //flowerArm.flowerArmUp(flowerGrab),
                                firstSpeciman
                        )
                ),
                claw.clawOpen(),
                new SleepAction(0.2),
                new ParallelAction(
                    driveToFirstSample,
                    new SequentialAction(
                            new SleepAction(0.5)
                            //flowerArm.flowerArmUp(flowerMid)
                    ),
                    vertical.verticalLiftDown(0, 0.7)
                ),
                driveToSecondSample,
                new ParallelAction(
                    driveToThirdSample,
                    //flowerArm.flowerArmUp(flowerDrop),
                    new SequentialAction(
                        vertical.exactVertical(vertGrab, 0.7),
                        vertical.verticalLiftStay(0.01)
                    )
                ),
                claw.clawClose(),
                new SleepAction(0.3),
                secondSpeciman,
                new SleepAction(0.2),
                driveToHumanPlayer,
                new SleepAction(0.3),
                thirdSpeciman,
                new SleepAction(0.2),
                driveToHumanPlayer2,
                new SleepAction(0.3),
                fourthSpeciman,
                new SleepAction(0.2),
                driveToHumanPlayer3,
//                new SleepAction(0.5),
//                new SleepAction(0.5),
//                secondSpeciman,
//                vertical.verticalLiftDown(vertGrab, 0.7),
//                vertical.verticalLiftStay(0.01),
//                new SleepAction(0.7),
//                claw.clawOpen()
                new SleepAction(5)
                );
    }
}
