package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "drive")
public class Soldiers_Team_Code extends LinearOpMode {

    private DcMotor intake = null;
    private DcMotor intake2 = null;
    private Servo Airplane_Launcher = null;
    private DcMotor hanger = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hanger = hardwareMap.get(DcMotor.class, "hanger");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        Airplane_Launcher = hardwareMap.get(Servo.class, "Airplane_Launcher");
        Airplane_Launcher.setPosition(20);
        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            if (gamepad1.b || gamepad2.b) {
                Airplane_Launcher.setPosition(90);
            }
        }


            if (gamepad1.x|| gamepad2.x) {
                intake.setPower(1);
                intake2.setPower(1);
            } else {
                intake.setPower(0);
                intake2.setPower(0);
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                hanger.setPower(1);
            } else {
                hanger.setPower(0);
            }

            if (gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5) {
                hanger.setPower(-1);
            } else {
                hanger.setPower(0);
            }


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }


