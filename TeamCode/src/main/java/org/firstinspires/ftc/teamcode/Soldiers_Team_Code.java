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


                        private DcMotor arm = null;
                        private Servo pixel = null;
                        private Servo right = null;
                        private Servo left = null;
                        @Override
                        public void runOpMode() throws InterruptedException {

                            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

                            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.get(DcMotor.class, "arm");
        pixel = hardwareMap.get(Servo.class, "pixel_holder");
        right = hardwareMap.get(Servo.class, "right_claw");
        left = hardwareMap.get(Servo.class,"left_claw");
        pixel.setPosition(0);
        left.setPosition(0.4);
        right.setPosition(0);

                            waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );




          if (gamepad2.left_bumper) {
              pixel.setPosition(0);
              right.setPosition(0);
              left.setPosition(0);
              sleep(250);
              arm.setPower(0.5);
          } else {
              arm.setPower(0);}

          if (gamepad2.right_bumper) {
              arm.setPower(-0.5);
          } else {
              arm.setPower(0);}


          }
          if (gamepad2.x) {
              right.setPosition(-0.4);
              left.setPosition(0.4);
              pixel.setPosition(0);
          }
          else{
              right.setPosition(0);
              left.setPosition(0);
              pixel.setPosition(0.5);
          }







            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }





