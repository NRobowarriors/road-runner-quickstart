package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

            Claw claw = new Claw(hardwareMap);
            VerticalLift vertical = new VerticalLift(hardwareMap);
            FlowerArm flowerArm = new FlowerArm(hardwareMap);
            Intake intakeSuck = new Intake(hardwareMap);
            Tilt intakeTilt = new Tilt(hardwareMap);
            IntakeArm intakeArm = new IntakeArm(hardwareMap);
            Wrist wrist = new Wrist(hardwareMap);

            waitForStart();

            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            drive.actionBuilder(beginPose).splineToConstantHeading(new Vector2d(-24,0),0).build(),
                            vertical.verticalLiftUp(1750, 0.7),
                            flowerArm.flowerArmUp(0.75)
                    ),
                    claw.clawOpen(),
                    drive.actionBuilder(new Pose2d(-24.0,0.0,0.0)).splineToConstantHeading(new Vector2d(0,0),0).build()
            ));
        }  else {
            throw new RuntimeException();
        }

        //intakeWheelRight.setPower(-0.5);
        //intakeWheelLeft.setPower(0.5);
        //sleep(4000);

    }
}
