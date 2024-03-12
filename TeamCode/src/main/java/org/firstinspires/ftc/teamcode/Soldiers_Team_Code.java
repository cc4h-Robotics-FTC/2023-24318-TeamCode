package org.firstinspires.ftc.teamcode; //name

import com.acmerobotics.roadrunner.geometry.Pose2d;  //weird stuff to describe what servos motors and things like that are
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


                    @TeleOp(group = "drive")
                    public class Soldiers_Team_Code extends LinearOpMode {


                        private DcMotor arm = null; //Describes motors and servos
                        private Servo pixel = null;
                        private DcMotor intake = null;
                        private Servo airplane = null;
                        private Servo gloves = null;


                        @Override
                        public void runOpMode() throws InterruptedException {

                            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //drive with Mecanum drive

                            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            arm = hardwareMap.get(DcMotor.class, "arm");  //explains variables
                            pixel = hardwareMap.get(Servo.class, "pixel_holder");
                            intake = hardwareMap.get(DcMotor.class, "intake");
                            airplane = hardwareMap.get(Servo.class, "airplane");
                            gloves = hardwareMap.get(Servo.class, "gloves");
                            pixel.setPosition(0.5);
                            airplane.setPosition(0.5);


                            waitForStart();

                            while (!isStopRequested()) {
                                drive.setWeightedDrivePower(
                                        new Pose2d(    //Drive controls
                                                -gamepad1.left_stick_y,
                                                -gamepad1.left_stick_x,
                                                -gamepad1.right_stick_x
                                        )
                                );


                                if (gamepad2.right_trigger > 0.5) { //arm up, servo on box close, intake spin
                                    arm.setPower(0.5);
                                    pixel.setPosition(0);
                                    intake.setPower(1);
                                } else {  //arm stop, intake stop
                                    arm.setPower(0);
                                    intake.setPower(0);
                                }

                                while (gamepad1.right_trigger > 0.5)

                                    gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);
                                gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);
                                gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);
                                gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);
                                gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);
                                gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);
                                gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);
                                gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);
                                gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);
                                gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);
                                gloves.setPosition(0.5);
                                sleep(500);
                                gloves.setPosition(1.5);
                                sleep(500);


                                if (gamepad2.left_trigger > 0.5) { // arm down, intake opposite direction
                                    arm.setPower(-0.5);
                                    intake.setPower(-0.5);
                                } else {   //arm stop, intake stop
                                    arm.setPower(0);
                                    intake.setPower(0);
                                }

                                if (gamepad2.y) {
                                    pixel.setPosition(0.5);
                                }
                                if (gamepad2.x) {
                                    pixel.setPosition(0);
                                }
                                if (gamepad2.b) {
                                    airplane.setPosition(1.5);
                                }

                                if (gamepad2.a) {  //intake on
                                    intake.setPower(1);
                                } else {  //intake off
                                    intake.setPower(0);
                                }


                                Pose2d poseEstimate = drive.getPoseEstimate();  //send us data on these things: x y and direction.
                                telemetry.addData("x", poseEstimate.getX());
                                telemetry.addData("y", poseEstimate.getY());
                                telemetry.addData("heading", poseEstimate.getHeading());
                                telemetry.update();
                            }
                        }
                    }





