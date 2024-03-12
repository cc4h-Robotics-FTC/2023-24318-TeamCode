//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import java.util.List;
//
//@Config
//@Autonomous(group = "Soldiers")
//public class AutoBlueCLOSERtwo extends LinearOpMode {
//
//
//
//
//    public static double ANGLE = -90;
//    public static double ANGLE2 = 90;
//    private DcMotor arm = null;
//    private Servo pixel = null;
//
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
//                .forward(24)
//                .build();
//
//        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory.end())
//                .back(30)
//                .build();
//
//        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
//                .strafeLeft(20)
//                .build();
//
//
//        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
//                .forward(5)
//                .build();
//
//        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
//                .strafeRight(18)
//                .build();
//
//
//
//        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
//                .back(24)
//                .build();
//
//        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
//                .strafeRight(30)
//                .build();
//
//        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
//                .strafeRight(24)
//                .build();
//
//
//
//        arm = hardwareMap.get(DcMotor.class, "arm");  //explains variables
//        pixel = hardwareMap.get(Servo.class,"pixel_holder");
//        purple = hardwareMap.get(Servo.class, "autopixel");
//        pixel.setPosition(0);
//        purple.setPosition(0);
//
//
//
//
//        waitForStart();
//
//        pixel.setPosition(0);
//        drive.followTrajectory(trajectory);
//
//
//        if (spikemark = 3) {
//            drive.turn(Math.toRadians(ANGLE));
//            purple.setPosition(0.5);
//            sleep(500);
//            drive.turn((Math.toRadians(ANGLE2)));
//            drive.followTrajectory(trajectory6);
//            drive.turn((Math.toRadians(ANGLE)));
//            drive.followTrajectory(trajectory2);
//            drive.followTrajectory(trajectory3);
//            arm.setPower(1);
//            sleep(2500);
//            arm.setPower(0);
//            pixel.setPosition(0.5);
//            sleep(500);
//            arm.setPower(-1);
//            sleep(2500);
//            arm.setPower(0);
//            drive.followTrajectory(trajectory4);
//            drive.followTrajectory(trajectory5);
//
//        }
//
//        else if (spikemark = 1) {
//            drive.turn(Math.toRadians(ANGLE2));
//            purple.setPosition(0.5);
//            sleep(500);
//            drive.turn((Math.toRadians(ANGLE)));
//            drive.followTrajectory(trajectory6);
//            drive.turn((Math.toRadians(ANGLE)));
//            drive.followTrajectory(trajectory2);
//            drive.followTrajectory(trajectory7);
//            arm.setPower(1);
//            sleep(2500);
//            arm.setPower(0);
//            pixel.setPosition(0.5);
//            sleep(500);
//            arm.setPower(-1);
//            sleep(2500);
//            arm.setPower(0);
//            drive.followTrajectory(trajectory4);
//            drive.followTrajectory(trajectory5);
//
//        }
//
//        else {
//            purple.setPosition(0.5);
//            sleep(500);
//            drive.followTrajectory(trajectory6);
//            drive.turn((Math.toRadians(ANGLE)));
//            drive.followTrajectory(trajectory2);
//            drive.followTrajectory(trajectory8);
//            arm.setPower(1);
//            sleep(2500);
//            arm.setPower(0);
//            pixel.setPosition(0.5);
//            sleep(500);
//            arm.setPower(-1);
//            sleep(2500);
//            arm.setPower(0);
//            drive.followTrajectory(trajectory4);
//            drive.followTrajectory(trajectory5);
//
//        }
//
//
//
//
//
//
//
//
//
//
//        telemetry.update();
//        sleep(2);
//
//
//
//        //if (isStopRequested()) return;
//        //drive.followTrajectory(trajectory);
//        while (!isStopRequested() && opModeIsActive()) {
//
//            telemetry.update();
//
//            // Push telemetry to the Driver Station.
//
//        }
//    }
//
//
//
//    }
//
//
