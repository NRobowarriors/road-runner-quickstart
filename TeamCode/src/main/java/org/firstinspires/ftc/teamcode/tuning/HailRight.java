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
    private int vertGrab = 375;//416;
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
        double HumanPlayerX = -7;
        double HumanPlayerY = 28;
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
                .strafeToLinearHeading(new Vector2d(specimanX - 2.5, specimanY), zero)
                .build();
        Action driveToFirstSample = drive.actionBuilder(new Pose2d(specimanX - 2.5, specimanY, zero))
                .splineToSplineHeading(new Pose2d(-20,13, Math.toRadians(180)),Math.toRadians(90)) //try -90 or 90
                .splineToConstantHeading(new Vector2d(-50,35),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-10,35),Math.toRadians(0))
                //.lineToX(-11)
                .splineToConstantHeading(new Vector2d(-52,41),Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-10, 41), Math.toRadians(0))
                //.lineToX(-11)
                .splineToConstantHeading(new Vector2d(-48,54),Math.toRadians(0)) //try 45 or -45
                //.splineToConstantHeading(new Vector2d(-8, 54), Math.toRadians(0)) //try -70
                .build();
        Action driveToHumanPlayer = drive.actionBuilder(new Pose2d(-48, 54, turn))
                .lineToX(-6.5)
                .build();
        Action secondSpeciman = drive.actionBuilder(new Pose2d(-7.5, 53, turn))
                .splineToLinearHeading(new Pose2d(specimanX, specimanY - 3, zero), Math.toRadians(-135))
                .build();
        Action driveToHumanPlayer1 = drive.actionBuilder(new Pose2d(specimanX,specimanY - 5, zero))
                .splineToLinearHeading(new Pose2d(HumanPlayerX, HumanPlayerY, turn), Math.toRadians(0))
                .build();
        Action thirdSpeciman = drive.actionBuilder(new Pose2d(HumanPlayerX, HumanPlayerY, turn))
                .splineToLinearHeading(new Pose2d(specimanX, specimanY - 6, zero), Math.toRadians(-90))
                .build();
        Action driveToHumanPlayer2 = drive.actionBuilder(new Pose2d(specimanX,specimanY - 8, zero))
                .splineToLinearHeading(new Pose2d(HumanPlayerX, HumanPlayerY, turn), Math.toRadians(0))
                .build();
        Action fourthSpeciman = drive.actionBuilder(new Pose2d(HumanPlayerX, HumanPlayerY, turn))
                .splineToLinearHeading(new Pose2d(specimanX, specimanY - 9, zero), Math.toRadians(-90))
                .build();
        Action driveToHumanPlayer3 = drive.actionBuilder(new Pose2d(specimanX,specimanY - 8, zero))
                .splineToLinearHeading(new Pose2d(HumanPlayerX, HumanPlayerY, turn), Math.toRadians(0))
                .build();
        Action fifthSpeciman = drive.actionBuilder(new Pose2d(HumanPlayerX, HumanPlayerY, turn))
                .splineToLinearHeading(new Pose2d(specimanX, specimanY - 9, zero), Math.toRadians(-90))
                .build();

        full = new SequentialAction(
                //First drive
                flowerArm.flowerArmUp(flowerGrab),
                new ParallelAction(
                        vertical.exactVertical(vertSpeciman, 0.6),
                        intakeArm.IntakeArmIn(),
                        intakeTilt.tiltUp(),
                        new SequentialAction(
                                new SleepAction(0.4),
                                firstSpeciman
                        )
                ),
                claw.clawOpen(),
                new SleepAction(0.1),
                new ParallelAction(
                    driveToFirstSample,
                    new SequentialAction(
                            new SleepAction(0.5),
                            flowerArm.flowerArmUp(flowerMid)
                    ),
                    vertical.verticalLiftDown(0, 0.7)
                ),
                new ParallelAction(
                    driveToHumanPlayer,
                    flowerArm.flowerArmUp(flowerDrop),
                    new SequentialAction(
                        vertical.exactVertical(vertGrab, 0.7),
                        vertical.verticalLiftStay(0.01)
                    )
                ),
                claw.clawClose(),
                new SleepAction(0.3),
                vertical.verticalLiftUp(500,1),
                new ParallelAction(
                        secondSpeciman,
                        flowerArm.flowerArmUp(flowerGrab),
                        new SequentialAction(
                            vertical.exactVertical(vertSpeciman,0.7),
                            vertical.verticalLiftStay(0.001)
                        )
                ),
                claw.clawOpen(),
                new SleepAction(0.1),
                new ParallelAction(
                        driveToHumanPlayer1,
                        new SequentialAction(
                            new SleepAction(0.7),
                            flowerArm.flowerArmUp(flowerDrop)
                        ),
                        new SequentialAction(
                                vertical.exactVertical(vertGrab, 0.7),
                                vertical.verticalLiftStay(0.001)
                        )
                ),
                claw.clawClose(),
                new SleepAction(0.3),
                vertical.verticalLiftUp(500,1),
                new ParallelAction(
                        thirdSpeciman,
                        flowerArm.flowerArmUp(flowerGrab),
                        new SequentialAction(
                                vertical.exactVertical(vertSpeciman,0.7),
                                vertical.verticalLiftStay(0.001)
                        )
                ),
                claw.clawOpen(),
                new SleepAction(0.1),
                new ParallelAction(
                        driveToHumanPlayer2,
                        new SequentialAction(
                            new SleepAction(0.7),
                            flowerArm.flowerArmUp(flowerDrop)
                        ),
                        new SequentialAction(
                                vertical.exactVertical(vertGrab, 0.7),
                                vertical.verticalLiftStay(0.001)
                        )
                ),
                claw.clawClose(),
                new SleepAction(0.3),
                vertical.verticalLiftUp(500,1),
                new ParallelAction(
                        fourthSpeciman,
                        flowerArm.flowerArmUp(flowerGrab),
                        new SequentialAction(
                                vertical.exactVertical(vertSpeciman,0.7),
                                vertical.verticalLiftStay(0.001)
                        )
                ),
                claw.clawOpen(),
                new SleepAction(0.1),
                new ParallelAction(
                        driveToHumanPlayer3,
                        new SequentialAction(
                            new SleepAction(0.7),
                            flowerArm.flowerArmUp(flowerDrop)
                        ),
                        new SequentialAction(
                                vertical.exactVertical(vertGrab, 0.7),
                                vertical.verticalLiftStay(0.001)
                        )
                ),
                claw.clawClose(),
                new SleepAction(0.3),
                vertical.verticalLiftUp(500,1),
                new ParallelAction(
                        fifthSpeciman,
                        flowerArm.flowerArmUp(flowerGrab),
                        new SequentialAction(
                                vertical.exactVertical(vertSpeciman,0.7),
                                vertical.verticalLiftStay(0.001)
                        )
                ),
                claw.clawOpen(),
                new SleepAction(5)
                );
    }
}
