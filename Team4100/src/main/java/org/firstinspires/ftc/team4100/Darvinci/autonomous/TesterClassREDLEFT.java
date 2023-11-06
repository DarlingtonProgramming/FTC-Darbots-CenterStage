package org.firstinspires.ftc.team4100.Darvinci.autonomous;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config

@Autonomous(name=" RED LEFT", group = "Competition", preselectTeleOp = "TeleOp Red")
public class TesterClassREDLEFT extends LinearOpMode {
    private FtcDashboard Dashboard;
    private List<LynxModule> Hubs;
    private DarvinciChassis Drive;
    private DcMotorEx Slide, Intake;
    private Servo Outtake;
    private CRServo Push;

    private static final boolean USE_WEBCAM = true;

    private WebcamName Webcam;
    private VisionPortal visionPortal;
    private TfodProcessor tfod;
    private static final String[] LABELS = {
            "REDtin"
    };
    private AprilTagProcessor aprilTagProcessor;
    public static String DETECTION = "right";

    private static Pose2d STRT = new Pose2d(-36.35, -61.27, Math.toRadians(90.00));

    @Override
    public void runOpMode() {
        //  initTfod();

        this.Dashboard = FtcDashboard.getInstance();
        this.Hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : this.Hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        this.Drive = new DarvinciChassis(hardwareMap);
        this.Drive.setPoseEstimate(STRT);

        this.Slide = hardwareMap.get(DcMotorEx.class, "Slide");
        this.Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int slideSTART = Slide.getCurrentPosition() + 26;
        int slideSCORE = slideSTART + 25;
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

        initTfod();


        while (!opModeIsActive()) {
            telemetry.addData(" = ", getDetection());
            telemetry.update();
            sleep(75);
        }


        waitForStart();

        if (opModeIsActive()) {

            if (getDetection().equals("middle")) {
                TrajectorySequence untitled0 = Drive.trajectorySequenceBuilder(new Pose2d(-34.38, -67.37, Math.toRadians(270.00)))
                        .lineTo(new Vector2d(-35.31, -38.91))// place middle spike
                        .waitSeconds(0.23)

                        .splineTo(new Vector2d(-40.51, -64.35), Math.toRadians(180.00)) //manuver to the blue trangle
                        .waitSeconds(0.3)

                        .lineTo(new Vector2d(10.81, -64.35)) //under the truss
                        .waitSeconds(0.1)
                        .build();
                Drive.setPoseEstimate(untitled0.start());
                Drive.followTrajectorySequence(untitled0);
            } else if (getDetection().equals("left")) {
                TrajectorySequence untitled0 = Drive.trajectorySequenceBuilder(new Pose2d(-34.38, -67.37, Math.toRadians(270.00)))
                        .lineToSplineHeading(new Pose2d(-36.01, -40.91, Math.toRadians(330))) // place left spike
                        .forward(4)
                        .strafeRight(17)
                        .waitSeconds(0.2)

                        .splineTo(new Vector2d(-40.51, -64.35), Math.toRadians(180.00)) //manuver to the blue trangle
                        .waitSeconds(0.2)

                        .lineTo(new Vector2d(10.81, -64.35)) //under the truss
                        .waitSeconds(0.1)
                        .build();
                Drive.setPoseEstimate(untitled0.start());
                Drive.followTrajectorySequence(untitled0);
            } else {
                TrajectorySequence untitled0 = Drive.trajectorySequenceBuilder(new Pose2d(-34.38, -67.37, Math.toRadians(270.00)))
                        .lineToSplineHeading(new Pose2d(-35.31, -40.91, Math.toRadians(200))) // place right spike
                        .back(5)
                        .forward(4)
                        .waitSeconds(0.2)

                        .splineTo(new Vector2d(-40.51, -64.35), Math.toRadians(180.00)) //manuver to the blue trangle
                        .waitSeconds(0.2)
                        
                        .lineTo(new Vector2d(10.81, -64.35)) //under the truss
                        .waitSeconds(0.1)

                                .build();
                Drive.setPoseEstimate(untitled0.start());
                Drive.followTrajectorySequence(untitled0);
            }

            Pose2d currentPose = Drive.getPoseEstimate();
            while (Drive.isBusy()) sleep(500);

            if (DETECTION.equals("left")) {
                xd = 55.04;
                yd = -36.0;
            } else if (DETECTION.equals("middle")) {
                xd = 55.04;
                yd = -41.89;
            } else { //right
                xd = 55.04;
                yd = -48.0; //30.7
            }


            sleep(4000);
            Slide.setTargetPosition(slideSTART + 725);
            Slide.setPower(1);
            sleep(100);
            Outtake.setPosition(0.36);//flip back

            TrajectorySequence aprilTagTraj2 = Drive.trajectorySequenceBuilder(currentPose)
                    .lineTo(new Vector2d(20.81, -47.35)) //under the truss
                    .lineTo(new Vector2d(xd, yd))//strafe to left right middle backboard
                    .forward(0.4) // forward 1

                    .build();
            Drive.followTrajectorySequence(aprilTagTraj2);

            Outtake.setPosition(0.246);//flip back
            sleep(10);
            Push.setPower(-1); //get the pixel out
            // sleep(400);
            //  Slide.setTargetPosition(Slide.getCurrentPosition() + 35);
            // Outtake.setPosition(0.243);
            sleep(3020);
            Slide.setTargetPosition(Slide.getCurrentPosition() + 75);
            sleep(1000);
            Push.setPower(0);


            TrajectorySequence aprilTagTraj3 = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                    .lineTo(new Vector2d(52.88, -19.75)) // strafe to the left side of the backboard

               //     .lineTo(new Vector2d(52.88, -56.95)) // strafe to the right side of the backboard

                    .build();
            Drive.followTrajectorySequence(aprilTagTraj3);



            Outtake.setPosition(0.95);
            sleep(300);
            Slide.setTargetPosition( (slideSTART + 26) - 1);
            sleep(140);



            TrajectorySequence aprilTagTraj4 = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                    .lineTo(new Vector2d(63.06, -19.56)) // move in to park on left side of backboard

               //     .lineTo(new Vector2d(63.06, -58.56)) // move in to park on right side of backboard

                    .build();
            Drive.followTrajectorySequence(aprilTagTraj4);




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
                .setModelAssetName("REDtin.tflite")
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
        tfod.setMinResultConfidence(0.16f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    public String getDetection() {
        int attempts = 0;


            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            if (currentRecognitions.size() == 0) {
                telemetry.addData("Nevermind but still", "Right");
                DETECTION = "right"; // RIGHT
            }
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
              //  telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.addData("FUCK you if ", "    you thought THE CAMERA WOULD ONLY SEE RIGHT ");

                if (x >= 20.0 && x <= 300.0 && recognition.getConfidence() * 100 > 66.0) {
                    telemetry.addData(" = ", "left");
                    DETECTION = "left";
                } else if (x >= 300.0 && x <= 700.0 && recognition.getConfidence() * 100 > 66.0) {
                    telemetry.addData(" = ", "middle");
                    DETECTION = "middle";
                }

            }   // end for() loop} while (DETECTION == null && (attempts++ < 8))
        return DETECTION;
    }

}