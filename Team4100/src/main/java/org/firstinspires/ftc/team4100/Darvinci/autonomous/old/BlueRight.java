package org.firstinspires.ftc.team4100.Darvinci.autonomous.old;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Disabled
@Config
@Autonomous(name="Old Blue [R, RP]", group="Old")
public class BlueRight extends LinearOpMode {
    private FtcDashboard Dashboard;
    private List<LynxModule> Hubs;
    private DarvinciChassis Drive;
    private DcMotorEx Slide, Intake;
    private Servo Outtake;
    private CRServo Push;

    private static final boolean USE_WEBCAM = true;

    private WebcamName Webcam;
    private VisionPortal visionPortal;
    private VisionPortal Vision;

    private AprilTagProcessor aprilTagProcessor;
    private TfodProcessor tfod;
    private static final String[] LABELS = {
            "bluetin"
            //BlueTin.tflite
    };
    public static String DETECTION = "right";

    private final Pose2d START_POS = new Pose2d(10.5, 62, Math.toRadians(90));

    @Override
    public void runOpMode() {
//        initTfod();

        this.Dashboard = FtcDashboard.getInstance();
        this.Hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : this.Hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        this.Drive = new DarvinciChassis(hardwareMap);
        this.Drive.setPoseEstimate(START_POS);

        this.Slide = hardwareMap.get(DcMotorEx.class, "Slide");
        this.Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int slideSTART = Slide.getCurrentPosition() + 25;//2198
        int slideSCORE = slideSTART + 690;
        int slideHIGHONDRUGS = slideSTART + 1460;
        this.Slide.setTargetPosition(slideSTART);
        this.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.Slide.setPower(1);

        this.Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        this.Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.Outtake = hardwareMap.get(Servo.class, "Outtake");
        this.Outtake.setPosition(0.95);
        //start position

        this.Push = hardwareMap.get(CRServo.class, "Push");

        this.Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        telemetry = new MultipleTelemetry(telemetry, Dashboard.getTelemetry());
        double xd = 0;
        double yd = 0;

      //  initDetection();
        initTfod();

        waitForStart();


        if (opModeIsActive()) {
            getDetection();

            if (DETECTION.equals("left")) {
                TrajectorySequence middleTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineTo(new Vector2d(10.5, 47))
                        .waitSeconds(0.6)
                        .lineToSplineHeading(new Pose2d(10, 38, Math.toRadians(102)))
                        .waitSeconds(0.6)
                        .back(2.2)
                        .waitSeconds(0.6)
                        .forward(5.4)

                        .lineToSplineHeading(new Pose2d(7.1, 60, Math.toRadians(179)))
                        .waitSeconds(0.6)
                        .lineToSplineHeading(new Pose2d(60, 60, Math.toRadians(178)))
                        .waitSeconds(7.5)
                        .lineToSplineHeading(new Pose2d(83, 37, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(middleTraj);
            } else if (DETECTION.equals("middle")) {
                TrajectorySequence middleTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineTo(new Vector2d(10.5, 34))
                        .waitSeconds(0.6)
                        .forward(3)
                        .waitSeconds(0.6)

                        .lineToSplineHeading(new Pose2d(7.1, 60, Math.toRadians(179)))
                        //for going to the door
                                        // .lineTo(new Vector2d(4.5, 34))
                        .waitSeconds(0.6)
                        .lineToSplineHeading(new Pose2d(60, 60, Math.toRadians(178)))
                        .waitSeconds(7.5)
                        //.forward 10
                        .lineToSplineHeading(new Pose2d(83, 37, Math.toRadians(180)))
                        //       .lineToSplineHeading(new Pose2d(83, 24, Math.toRadians(180)))

                        .build();
                Drive.followTrajectorySequence(middleTraj);
            } else { //    right
                TrajectorySequence middleTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineTo(new Vector2d(10.5, 52))//go to like some side position
                        .lineToSplineHeading(new Pose2d(9, 33, Math.toRadians(17)), //actually go to place
                                DarvinciChassis.getVelocityConstraint(44, Math.toRadians(17), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(44))
                        .forward(0.6)


                        .lineToSplineHeading(new Pose2d(7.1, 60, Math.toRadians(179)))
                        .waitSeconds(0.1)
                        .lineToSplineHeading(new Pose2d(60, 60, Math.toRadians(178)))
                        .waitSeconds(7.5)
                        .lineToSplineHeading(new Pose2d(83, 37, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(middleTraj);
            }
            //right x = 92 y = 28
            //middle x = 92 y = 31
            // left x = 92 y = 47

            Pose2d currentPose = Drive.getPoseEstimate();

            if (DETECTION.equals("left")) {
                xd = 98;
                yd = 35.4;
            } else if (DETECTION.equals("middle")) {
                xd = 98;
                yd = 34.7;
            } else { //MIDDLE
                xd = 98;
                yd = 31.2; //30.7
            }


                // -0.03
                // 13.3
                // 39.585
                // 27.92

            TrajectorySequence aprilTagTraj2 = Drive.trajectorySequenceBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(xd, yd, Math.toRadians(180)))
                        .addDisplacementMarker(() -> {
                            Slide.setTargetPosition(slideSTART + 695);
                            Slide.setPower(1);
                        })
                        .build();
                Drive.followTrajectorySequence(aprilTagTraj2);

                sleep(200);

                Outtake.setPosition(0.32-0.06);
                sleep(350);
                Push.setPower(1);
                sleep(450);
                Push.setPower(0);
                sleep(350);
                Push.setPower(-1);
                sleep(500);
                Slide.setTargetPosition(Slide.getCurrentPosition() + 70);
                sleep(500);
                Push.setPower(0);


            //score on backdrops
            TrajectorySequence parkTraj2 = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                    .forward(5)
                    .lineToSplineHeading(new Pose2d(94.5, 20, Math.toRadians(180)))
                    .lineToSplineHeading(new Pose2d(99.5, 13, Math.toRadians(180)))
                    .build();
            Drive.followTrajectorySequence(parkTraj2);


            sleep(150);
            Outtake.setPosition(0.95);
            sleep(100);
            Slide.setTargetPosition(slideSTART + 26);
           // Outtake.setPosition(0.32);


            // -0.03
            // 13.3
            // 39.585
            // 27.92


            //raises slide

//if
            //score on backdrops and park



        }


    }


    private void initDetection() {
        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(Webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .addProcessor(aprilTagProcessor);

        this.visionPortal = builder.build();
//        Dashboard.startCameraStream((CameraStreamSource) aprilTagProcessor, 0);
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName("BlueTin.tflite")
              //  .setModelFileName("TheNEWdetection.tflite")

                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)

                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.20f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    public void getDetection() {
        int attempts = 0;

        do {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                if (x >= 182.0 && x <= 570.0 && recognition.getConfidence() * 100 > 66.0) {
                    telemetry.addData("whitepixel", "Middle");
                    DETECTION = "middle";
                } else if (x >= 17.0 && x <= 80.0 && recognition.getConfidence() * 100 > 66.0) {
                    telemetry.addData("whitepixel", "Left");
                    DETECTION = "left"; //
                } else {
                    telemetry.addData("whitepixel", "Right");
                    DETECTION = "right"; // RIGHT
                }

            }   // end for() loop
        } while (DETECTION == null && (attempts++ < 8));
    }
    private AprilTagPoseFtc getAprilTagPose() {
        int attempts = 0;
        final int DETECTION_ID = DETECTION.equals("left") ? 1 : DETECTION.equals("middle") ? 2 : 3;

        while (true) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == DETECTION_ID) {
                    visionPortal.close();
                    return detection.ftcPose;
                }
            }

            if (++attempts > 7) {
                return null;
            }

            telemetry.update();
            sleep(20);
        }
    }
}

