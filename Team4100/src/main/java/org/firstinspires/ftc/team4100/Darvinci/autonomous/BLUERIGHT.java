package org.firstinspires.ftc.team4100.Darvinci.autonomous;
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
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Disabled
@Autonomous(name=" BLUE - RIGHT", group = "Competition", preselectTeleOp = "TeleOp Blue")
public class BLUERIGHT extends LinearOpMode {
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
            "BlueTin"
    };
    private AprilTagProcessor aprilTagProcessor;
    public static String DETECTION = "right";
    private final Pose2d START_POS = new Pose2d(-34.29, 59.43, Math.toRadians(90.00));



    @Override
    public void runOpMode() {


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

        int slideSTART = Slide.getCurrentPosition() + 25;
        int slideSCORE = slideSTART + 690;
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


       // initDetection();

        initTfod();



        while (!opModeIsActive()) {
            telemetry.addData(" = ", getDetection());
            telemetry.update();
            sleep(75);
        }


        waitForStart();



        if (opModeIsActive()) {
            //telemetry.addData(getDetection(), "crazy");
            //telemetry.update();


            if (getDetection().equals("middle")) {
                    TrajectorySequence untitled0 = Drive.trajectorySequenceBuilder(START_POS)
                            .lineTo(new Vector2d(-36.87, 30.87)) // go to middle spike
                            .waitSeconds(0.3)

                            .splineTo(new Vector2d(-45.21, 55.46), Math.toRadians(180))//manuver to the red triangle
                            .waitSeconds(0.2)

                            .lineTo(new Vector2d(7.28, 55.46)) // go to truss
                            .waitSeconds(0.1)

                            .build();
                    Drive.setPoseEstimate(untitled0.start());
                    Drive.followTrajectorySequence(untitled0);
                } else if (getDetection().equals("left")){
                    TrajectorySequence untitled0 = Drive.trajectorySequenceBuilder(START_POS)
                            // go to left spike
                            .lineToSplineHeading(new Pose2d(-37.77, 36.87, Math.toRadians(146)))
                            .back(8)
                            .forward(5)
                            .waitSeconds(0.2)

                            .splineTo(new Vector2d(-45.21, 55.46), Math.toRadians(180))//manuver to the red triangle
                            .waitSeconds(0.3)

                            .lineTo(new Vector2d(7.28, 55.46)) // go to truss
                            .waitSeconds(0.1)

                            .build();
                    Drive.setPoseEstimate(untitled0.start());
                    Drive.followTrajectorySequence(untitled0);
                } else {
                    TrajectorySequence untitled0 = Drive.trajectorySequenceBuilder(START_POS)
                            .lineToSplineHeading(new Pose2d(-36.87, 36.87, Math.toRadians(35))) // go to right spike
                            .forward(1)
                            .strafeLeft(4)
                            .waitSeconds(0.1)

                            .splineTo(new Vector2d(-45.21, 55.46), Math.toRadians(180))//manuver to the red triangle
                            .waitSeconds(0.3)

                            .lineTo(new Vector2d(7.28, 55.46)) // go to truss
                            .waitSeconds(0.1)


                            .build();
                    Drive.setPoseEstimate(untitled0.start());
                    Drive.followTrajectorySequence(untitled0);
                }

            Pose2d currentPose = Drive.getPoseEstimate();
            while (Drive.isBusy()) sleep(500);

            if (DETECTION.equals("left")) {
                xd = 53.96;
                yd = 40.13;
            } else if (DETECTION.equals("middle")) {
                xd = 53.96;
                yd = 32.13;
            } else { //right
                xd = 53.96;
                yd = 28.2; //30.7
            }
            sleep(4000);
            Slide.setTargetPosition(slideSTART + 725);
            Slide.setPower(1);
            sleep(100);
            Outtake.setPosition(0.36);//flip back

            TrajectorySequence aprilTagTraj2 = Drive.trajectorySequenceBuilder(currentPose)
                    .lineTo(new Vector2d(19.28, 55.46)) // go to truss
                    .lineTo(new Vector2d(xd, yd))//strafe to left right middle backboard
                    .back(1) // forward 1

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

            /*
            if (DETECTION.equals("middle")) {
                TrajectorySequence aprilTagTraj5 = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                        .forward(4)

                        .addDisplacementMarker(3,() -> {
                            sleep(150);
                            Outtake.setPosition(0.95);
                            sleep(400);
                            Slide.setTargetPosition(slideSTART + 26);
                            sleep(400);
                        })

                        .lineTo(new Vector2d(-38.15, 35.2)) //under the truss

                        .lineToSplineHeading(new Pose2d(-58.67, 35.2, Math.toRadians(180.0)))  // to face stack


                        .addDisplacementMarker(1,() -> {
                            Intake.setPower(1);
                        })
                        .lineToSplineHeading(new Pose2d(-65.67, 36.81, Math.toRadians(180)), //actually go to stack
                                DarvinciChassis.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(20))

                        .back(5) //back up


                        .build();
                Drive.setPoseEstimate(aprilTagTraj5.start());
                Drive.followTrajectorySequence(aprilTagTraj5);

                Intake.setPower(0);


                TrajectorySequence aprilTagTraj6 = Drive.trajectorySequenceBuilder(currentPose)
                        .lineTo(new Vector2d(-45.15, 35.2))//go back

                        .addDisplacementMarker(10,() -> {
                            Slide.setTargetPosition(slideSTART + 725);
                            Slide.setPower(1);
                            sleep(100);
                            Outtake.setPosition(0.36);//flip back
                        })


                        .lineTo(new Vector2d(53.96, 32.63))//strafe to middle backboard
                        .back(1) // forward 1

                        .build();
                Drive.followTrajectorySequence(aprilTagTraj6);

                Outtake.setPosition(0.24);//flip back
                sleep(200);
                Push.setPower(1);
                sleep(600);
                Push.setPower(0);
                sleep(100);
                Push.setPower(-1);
                sleep(600);
                Slide.setTargetPosition(Slide.getCurrentPosition() + 70);
                sleep(630);
                Push.setPower(0);
                sleep(30);

            }


            */



            TrajectorySequence aprilTagTraj3 = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                    .forward(2)
                    .lineTo(new Vector2d(47.92, 11.0))//strafe to park

                    .build();
            Drive.followTrajectorySequence(aprilTagTraj3);

            sleep(50);
            Outtake.setPosition(0.95);
            sleep(420);
            Slide.setTargetPosition( (slideSTART + 26) - 1);
            sleep(140);



            TrajectorySequence aprilTagTraj4 = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                    .lineTo(new Vector2d(60.96, 11.0))//finally move in to park on right of backboards.

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
              //  telemetry.addData("- Position", "%.0f / %.0f", x, y);
              //  telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.addData("FUCK you if ", "    you thought THE CAMERA WOULD ONLY SEE RIGHT ");

                if (x >= 80.0 && x <= 450.0 && recognition.getConfidence() * 100 > 66.0) {
                    telemetry.addData(" = ", "Middle");
                    DETECTION = "middle";
                } else if (x >= 17.0 && x <= 80.0 && recognition.getConfidence() * 100 > 66.0) {
                    telemetry.addData(" = ", "Left");
                    DETECTION = "left"; //
                }
            }   // end for() loop
        return DETECTION;
    }
}