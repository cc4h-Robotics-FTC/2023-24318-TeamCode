package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "drive")
public class Soldiers_Team_Code extends LinearOpMode {

    private DcMotor intake = null;
    private Servo Airplane_Launcher = null;
    private DcMotor arm = null;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.get(DcMotor.class,"arm");
       intake =  hardwareMap.get(DcMotor.class, "Intake");
       Airplane_Launcher = hardwareMap.get(Servo.class, "Airplane_Launcher");
        Airplane_Launcher.setPosition(90);
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
    Airplane_Launcher.setPosition(0);
}
            if (gamepad1.x || gamepad2.x) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.y) {
                arm.setPower(1);
            } else{
                    arm.setPower(0);
            }

            if (gamepad2.a) {
                arm.setPower(-1);
            } else{
                arm.setPower(0);
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
