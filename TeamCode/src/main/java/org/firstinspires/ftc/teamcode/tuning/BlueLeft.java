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
    private DcMotor  motorVerticalLeft, motorVerticalRight;


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
        intakeArm.setPosition(0);

        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            intakeTilt.setPosition(1);
            claw.setPosition(0.7);
            clawWrist.setPosition(0.84);
            Claw claw = new Claw(hardwareMap);
            VerticalLift vertical = new VerticalLift(hardwareMap);
            FlowerArm flowerArm = new FlowerArm(hardwareMap);
            Intake intakeSuck = new Intake(hardwareMap);
            Tilt intakeTilt = new Tilt(hardwareMap);
            IntakeArm intakeArm = new IntakeArm(hardwareMap);
            Wrist wrist = new Wrist(hardwareMap);
            waitForStart();

            Actions.runBlocking(new SequentialAction(
                    //First drive
                    new ParallelAction(
                            drive.actionBuilder(beginPose).strafeToLinearHeading(new Vector2d(-32, 9), Math.toRadians(42)).build(),
                            vertical.verticalLiftUp(1750, 0.7),
                            flowerArm.flowerArmUp(0.63)
                    ),
                    claw.clawOpen(),
                    new SleepAction(0.3),
                    //To 2nd sample
                    new ParallelAction(
                            vertical.verticalLiftDown(300, 0.5),
                            flowerArm.flowerArmDown(0.06),
                            drive.actionBuilder(new Pose2d(-32, 9, Math.toRadians(45)))
                                    .strafeToLinearHeading(new Vector2d(-21, 12), Math.toRadians(90))
                                    .build(),
                            intakeArm.intakeArmOut(),
                            intakeSuck.intakeSuckIn()
                    ),
                    new SleepAction(0.35),
                    //Pick up 2nd Sample
                    intakeTilt.tiltDown(),
                    drive.actionBuilder(new Pose2d(-21, 12, Math.toRadians(90))).splineToConstantHeading(new Vector2d(-21, 19), Math.toRadians(90)).build(),
                    new SleepAction(0.2),
                    intakeSuck.intakeSuckInStop(),
                    //Bring sample to Center
                    new ParallelAction(
                            intakeTilt.tiltUp(),
                            intakeArm.IntakeArmIn(),
                            drive.actionBuilder(new Pose2d(-21,19, Math.toRadians(90))).turn( Math.toRadians(-45)).build()
                    ),
                    vertical.verticalLiftDown(50,0.5),
                    new SleepAction(0.15),
                    claw.clawClose(),
                    new SleepAction(0.2),
                    //Second sample in basket
                    new ParallelAction(
                            drive.actionBuilder(new Pose2d(-21,19, Math.toRadians(45))).splineToConstantHeading(new Vector2d(-30, 9), Math.toRadians(45)).build(),
                            flowerArm.flowerArmUp(0.63),
                            vertical.verticalLiftUp(1650,0.8)
                    ),
                    new SleepAction(0.5),
                    claw.clawOpen(),
                    new SleepAction(0.5),
                    //Drive to 3rd sample
                    new ParallelAction(
                            vertical.verticalLiftDown(300, 0.5),
                            flowerArm.flowerArmDown(0.06),
                            drive.actionBuilder(new Pose2d(-30, 9, Math.toRadians(45)))
                                    .strafeToLinearHeading(new Vector2d(-33, 12), Math.toRadians(90))
                                    .build(),
                            intakeArm.intakeArmOut(),
                            intakeSuck.intakeSuckIn()
                    ),
                    new SleepAction(0.35),
                    intakeTilt.tiltDown(),
                    drive.actionBuilder(new Pose2d(-30, 9, Math.toRadians(90))).splineToConstantHeading(new Vector2d(-30, 17), Math.toRadians(90)).build(),
                    new SleepAction(0.2),
                    intakeSuck.intakeSuckInStop(),
                    //Sample to Center
                    new ParallelAction(
                        intakeTilt.tiltUp(),
                        intakeArm.IntakeArmIn(),
                        drive.actionBuilder(new Pose2d(-30,17, Math.toRadians(90))).turn( Math.toRadians(-45)).build()
                    ),
                    vertical.verticalLiftDown(50,0.5),
                    new SleepAction(0.15),
                    claw.clawClose(),
                    new SleepAction(0.2),
                    //3rd Sample to basket
                    new ParallelAction(
                            drive.actionBuilder(new Pose2d(-30,17, Math.toRadians(45))).splineToConstantHeading(new Vector2d(-30, 2), Math.toRadians(45)).build(),
                            flowerArm.flowerArmUp(0.63),
                            vertical.verticalLiftUp(1650,0.8)
                    ),
                    new SleepAction(0.5),
                    claw.clawOpen(),
                    new SleepAction(0.5),
                    new ParallelAction(
                            vertical.verticalLiftDown(300, 0.5),
                            flowerArm.flowerArmDown(0.06),
                            drive.actionBuilder(new Pose2d(-30, 2, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(-34, 10), Math.toRadians(105))
                                .build(),
                            intakeArm.intakeArmOut(),
                            intakeSuck.intakeSuckIn()
                    ),
                    new SleepAction(0.35),
                    intakeTilt.tiltDown(),
                    drive.actionBuilder(new Pose2d(-34, 10, Math.toRadians(105))).splineToConstantHeading(new Vector2d(-34, 14), Math.toRadians(105)).build(),
                    new SleepAction(0.4),
                    intakeSuck.intakeSuckInStop(),
                    //Sample to Center
                    new ParallelAction(
                            intakeTilt.tiltUp(),
                            intakeArm.IntakeArmIn(),
                            drive.actionBuilder(new Pose2d(-30,17, Math.toRadians(105))).turn( Math.toRadians(-60)).build()
                    ),
                    vertical.verticalLiftDown(50,0.5),
                    new SleepAction(0.15),
                    claw.clawClose(),
                    new SleepAction(0.2)
                    //4rd Sample to basket
                    /*new ParallelAction(
                            drive.actionBuilder(new Pose2d(-34,14, Math.toRadians(45))).splineToConstantHeading(new Vector2d(-30, 2), Math.toRadians(45)).build(),
                            flowerArm.flowerArmUp(0.63),
                            vertical.verticalLiftUp(1650,0.8)
                    ),
                    new SleepAction(0.5),
                    claw.clawOpen(),
                    new SleepAction(0.5)

                     */
            )
        );
        }  else {
            throw new RuntimeException();
        }
    }
}
