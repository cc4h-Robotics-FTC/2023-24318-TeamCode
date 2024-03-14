package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
@Autonomous(name="Soldiers Blue Back Sequence", group="Soldiers")
public class PropDetectionBlueBackSequence extends LinearOpMode {

    public static double STARTING_POSE_X = 12;
    public static double STARTING_POSE_Y = 61;
    public static double STARTING_POSE_ANGLE = 270;

    // prop values
    public static double RIGHT_FORWARD = 35;
    public static double RIGHT_TURN = -72;
    public static double RIGHT_STRAFE = 12;
    public static double CENTER_FORWARD = 26;
    public static double CENTER_TURN = 68;
    public static double LEFT_FORWARD = 15;
    public static double LEFT_TURN = 75;
    public static double LEFT_STRAFE = 6;
    public static double LEFT_STRAFE_PARK = 27;
    public static double LEFT_PARK = 12;
    public static double CENTER_PARK = 14;
    public static double RIGHT_PARK = 18;


    // park values
    public static double PARK = 24;
    public static double LEFT_PIXEL_RETREAT = 3; // in
    public static double RIGHT_PIXEL_RETREAT = 3; // in


    // movement values
    public static long PIXEL_SERVO_WAIT_MILLISECS = 750;
    public static long ARM_SERVO_WAIT_MILLISECS = 1000;

    DistanceSensor Ldistance;
    DistanceSensor Rdistance;
    private DcMotor arm = null;
    private Servo pixel = null;
    private DcMotor intake = null;

    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
    //private AprilTagProcessor aprilTag;
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "blockprop-obstructed.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blockprop",
    };

    // Camera resolution
    private static final int CAMHORIZRES = 640;
    private static final int CAMVERTRES = 480;

    // Duck Spike Mark Detection Thresholds
    private static final int  LEFTSPIKETHRESHOLD = 200;
    private static final int  RIGHTSPIKETHRESHOLD = 440;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store the minimum confidence of duck prop recognition.
     */
    private float tfodMinConfidence = 0.92f;

    /**
     * The variable to store the location of the prop (1=>left, 2=>center, 3=>right)
     */
    private int spikeMark = 0;

    /**
     * The variable to store our instance of the vision portal.
     */
    private  VisionPortal visionPortal;


    @Override
    public void runOpMode() {
        // TODO:
        // make the autonomous for all three other positions. (blue far , near and red far)


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pixel = hardwareMap.get(Servo.class, "pixel_holder");
        pixel.setPosition(0);


        initDoubleVision();
        visionPortal.setProcessorEnabled(aprilTag, false);

        Pose2d startingPose = new Pose2d(
                STARTING_POSE_X,
                STARTING_POSE_Y,
                Math.toRadians(STARTING_POSE_ANGLE));
        drive.setPoseEstimate(startingPose);

        TrajectorySequence leftProp = drive.trajectorySequenceBuilder(startingPose)
                .strafeLeft(LEFT_STRAFE)
                //.turn(Math.toRadians(RIGHT_TURN))
                .forward(LEFT_FORWARD)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(leftProp.end())
                .back(LEFT_PIXEL_RETREAT)
                .strafeLeft(LEFT_STRAFE_PARK/2)
                .turn(Math.toRadians(-LEFT_TURN))
                .strafeLeft(LEFT_FORWARD/9)
                .build();

        TrajectorySequence centerProp = drive.trajectorySequenceBuilder(startingPose)
                .forward(CENTER_FORWARD)
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(centerProp.end())
                .back(LEFT_PIXEL_RETREAT)
                .strafeLeft(LEFT_STRAFE_PARK/1.1)
                .turn(Math.toRadians(-CENTER_TURN))
                .back(LEFT_STRAFE_PARK/13.5)
                .build();


        TrajectorySequence rightProp = drive.trajectorySequenceBuilder(startingPose)
                .forward(RIGHT_FORWARD-9)
                .turn(Math.toRadians(RIGHT_TURN))
                .forward(RIGHT_PIXEL_RETREAT)
//                .back(LEFT_PIXEL_RETREAT)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(rightProp.end())
                .back(RIGHT_PIXEL_RETREAT)
//                .turn(Math.toRadians(-LEFT_TURN))
                .back(RIGHT_FORWARD)
                // .strafeRight(LEFT_STRAFE)
//                .strafeRight(RIGHT_STRAFE_PARK/2.5)
                .build();

        waitForStart();

        if (opModeIsActive()) {
            int iterations = 1;
            boolean propFound = false;
            while ( iterations < 600 && spikeMark == 0 ) {
                //while ( opModeIsActive() ) {
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                for ( Recognition recognition : currentRecognitions ) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2;

                    if ( x <= LEFTSPIKETHRESHOLD ) {
                        telemetry.addData("Prop Location", "Left");
                        telemetry.update();
                        propFound = true;
                        spikeMark = 1;
                        //visionPortal.setProcessorEnabled(tfod, false);
                        break;
                    } else if ( x > LEFTSPIKETHRESHOLD && x <= RIGHTSPIKETHRESHOLD ) {
                        telemetry.addData("Prop Location", "Center");
                        telemetry.update();
                        propFound = true;
                        spikeMark = 2;
                        //visionPortal.setProcessorEnabled(tfod, false);
                        break;
                    } else if ( x > RIGHTSPIKETHRESHOLD ) {
                        telemetry.addData("Prop Location", "Right");
                        telemetry.update();
                        propFound = true;
                        spikeMark = 3;
                        //visionPortal.setProcessorEnabled(tfod, false);
                        break;
                    }
                }
                sleep(25);
                iterations++;
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

        if (isStopRequested()) return;

        // Choose the correct trajectory sequence
        if ( spikeMark == 1 ) { // Left
            drive.followTrajectorySequence(leftProp);
        } else if ( spikeMark == 2 ) { // Center
            drive.followTrajectorySequence(centerProp);
        } else if ( spikeMark == 3 ) { // Right
            drive.followTrajectorySequence(rightProp);
        } else {
            drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(startingPose)
                            .strafeRight(PARK)
                            .build()
            );
        }

        // Deposit the purple pixel
        intake.setPower(-0.25);

        sleep(PIXEL_SERVO_WAIT_MILLISECS );
        intake.setPower(0);



// TODO: Move robot to backdrop, place pixel

        // park robot in backstage
        if ( spikeMark == 1 ) { // Left
            drive.followTrajectorySequence(leftPark);
        } else if ( spikeMark == 2 ) { // Center
            drive.followTrajectorySequence(centerPark);
        } else if ( spikeMark == 3 ) { // Right
            drive.followTrajectorySequence(rightPark);

        }

        arm.setPower(1);
        sleep(800);
        arm.setPower(0);
        sleep(1000);
        pixel.setPosition(0.5);
        sleep(1000);
        arm.setPower(-1);
        sleep(800);
        arm.setPower(0);



       //
    } // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initDoubleVision() {
        // Create the AprilTagProcessor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        tfod.setMinResultConfidence(tfodMinConfidence);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(CAMHORIZRES, CAMVERTRES));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessors(tfod, aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initDoubleVision()

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    private String getPropLocation( double x ) {
        if ( x <= LEFTSPIKETHRESHOLD ) {
            return "Left";
        } else if ( x > LEFTSPIKETHRESHOLD && x <= RIGHTSPIKETHRESHOLD ) {
            return "Center";
        } else if ( x > RIGHTSPIKETHRESHOLD ) {
            return "Right";
        } else {
            return "Unknown";
        }
    }

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y, getPropLocation(x));
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
}
