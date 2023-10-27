package org.firstinspires.ftc.team4100.Darvinci.autonomous.old;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team4100.Darvinci.DarvinciCore;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;
import org.firstinspires.ftc.team4100.Darvinci.util.PoseStorage;
import org.firstinspires.ftc.team4100.Darvinci.vision.PropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name="Old Blue [L, LP]", group="Old")
public class BlueLeftLeftParking extends LinearOpMode {
    // Declare variables and devices
    private FtcDashboard Dashboard;
    private List<LynxModule> Hubs;
    private DarvinciChassis Drive;
    private DcMotorEx Slide, Intake;
    private int SLIDE_INITIAL;
    private Servo Outtake;
    private CRServo Push;

    private WebcamName Webcam;
    private VisionPortal Vision;
    private PropDetectionProcessor propDetectionProcessor;
    private AprilTagProcessor aprilTagProcessor;
    public static PropDetectionProcessor.Detection DETECTION = PropDetectionProcessor.Detection.RIGHT;

    private final Pose2d START_POS = new Pose2d(10.5, 62, Math.toRadians(90));

    @Override
    public void runOpMode() {
        // Initialize variables
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
        SLIDE_INITIAL = this.Slide.getCurrentPosition();
        // Offset is due to bucket dragging on ground
        this.Slide.setTargetPosition(SLIDE_INITIAL + 25);
        this.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.Slide.setPower(1);

        this.Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        this.Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.Outtake = hardwareMap.get(Servo.class, "Outtake");
        // Initialize the bucket position
        this.Outtake.setPosition(DarvinciSettings.BUCKET_POS_IN);

        this.Push = hardwareMap.get(CRServo.class, "Push");

        this.Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Multiple telemetry for FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, Dashboard.getTelemetry());

        // Initialize portal and detection processors
        initDetection();

        // Constantly get detected side while not started
        while (!opModeIsActive()) {
            DETECTION = propDetectionProcessor.getDetectedSide();
            telemetry.addData("Detected", DETECTION);
            telemetry.update();
            sleep(10);
        }

        waitForStart();

        if (isStopRequested()) return;

        // Change to april tag processor to prop detection
        Vision.setProcessorEnabled(propDetectionProcessor, false);
        Vision.setProcessorEnabled(aprilTagProcessor, true);

        if (opModeIsActive()) {
            // Change trajectory based on detected side
            if (DETECTION == PropDetectionProcessor.Detection.LEFT) {
                TrajectorySequence leftTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineToSplineHeading(new Pose2d(19, 39, Math.toRadians(92)))
                        .forward(14)
                        .lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(leftTraj);
            } else if (DETECTION == PropDetectionProcessor.Detection.MIDDLE) {
                TrajectorySequence middleTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineTo(new Vector2d(11.5, 34))
                        .forward(10)
                        .lineToSplineHeading(new Pose2d(40, 33, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(middleTraj);
            } else {
                // assume right
                TrajectorySequence rightTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineTo(new Vector2d(13.5, 58))
                        .lineToSplineHeading(new Pose2d(10.5, 33, Math.toRadians(15)),
                                DarvinciChassis.getVelocityConstraint(20, Math.toRadians(30), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(20))
                        .back(3)
                        .lineToSplineHeading(new Pose2d(40, 26, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(rightTraj);
            }

            // Get the pose of the designated april tag
            AprilTagPoseFtc aprilTagPose = getAprilTagPose();

            // If an april tag is detected then drive to it
            if (aprilTagPose != null) {
                // Get current pose
                Pose2d currentPose = Drive.getPoseEstimate();

                // Calculate a traj based on the april tag pose
                TrajectorySequence aprilTagTraj = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(currentPose.getX() + (aprilTagPose.y - 2), currentPose.getY() + (aprilTagPose.x + 4), Math.toRadians(180)))
                        .addDisplacementMarker(() -> {
                            // Slide up to dropping height
                            Slide.setTargetPosition(SLIDE_INITIAL + 690);
                            Slide.setPower(1);
                        })
                        .build();
                Drive.followTrajectorySequence(aprilTagTraj);

                // Flip the bucket to prepare to drop (not all the way)
                Outtake.setPosition(DarvinciSettings.BUCKET_POS_OUT - 0.06);
                sleep(650);

                // Make sure the pixel is in the bucket all the way
                Push.setPower(1);
                sleep(450);

                // Stop the push
                Push.setPower(0);
                sleep(350);

                // Drop the pixel
                Push.setPower(-1);
                sleep(3300);

                // Slide up in case it gets stuck on backboard
                Slide.setTargetPosition(Slide.getCurrentPosition() + 70);
                sleep(850);

                // Stop the push
                Push.setPower(0);
            }

//            Drive.setMotorPowers(-0.1, -0.1, -0.1, -0.1);
//            sleep(450);

            // Get away from backboard
            TrajectorySequence back = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                    .forward(5)
                    .build();
            Drive.followTrajectorySequence(back);

            // Set bucket to pos where it doesn't hit
            this.Outtake.setPosition(DarvinciSettings.BUCKET_POS_IN - 0.13);
            sleep(250);

            // Slide down where bucket needs to turn
            Slide.setTargetPosition(SLIDE_INITIAL + 75);

            // Wait till slide is at that position then set bucket pos to initial
            while (Slide.getCurrentPosition() > (SLIDE_INITIAL + 95)) {
                sleep(10);
            }
            sleep(100);
            this.Outtake.setPosition(DarvinciSettings.BUCKET_POS_IN);

            // Go back to initial slide position
            sleep(500);
            this.Slide.setTargetPosition(SLIDE_INITIAL - 25); // <- Think about this

            sleep(200);

            // Park Darvinci
            TrajectorySequence parkTraj = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(41, 59, Math.toRadians(180)))
                    .lineToSplineHeading(new Pose2d(57, 59, Math.toRadians(180)))
                    .build();
            Drive.followTrajectorySequence(parkTraj);

            PoseStorage.currentPose = Drive.getPoseEstimate();
        }
    }

    private AprilTagPoseFtc getAprilTagPose() {
        // List of april tag poses
        ArrayList<AprilTagPoseFtc> list = new ArrayList<>();

        // Number of attempts looped through
        int attempts = 0;

        // Get april tag id based on detected side
        final int DETECTION_ID = DETECTION == PropDetectionProcessor.Detection.LEFT ? 1 : DETECTION == PropDetectionProcessor.Detection.MIDDLE ? 2 : 3;

        // Get the april tag poses
        while (true) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                // If it's the one we're looking for, add to the list
                if (detection.id == DETECTION_ID) {
                    list.add(detection.ftcPose);
                }
            }

            // Max number of attempts
            if (++attempts > 9) {
                break;
            }

            telemetry.update();
            sleep(20);
        }

        // Close the vision portal to save memory and cpu
        Vision.close();

        // If no april tags detected, just return null and park Darvinci
        if (list.size() == 0) {
            return null;
        }

        // Get average of april tag poses to get one that is GOOD
        double x_avg = 0;
        double y_avg = 0;

        for (AprilTagPoseFtc pose : list) {
            x_avg += pose.x;
            y_avg += pose.y;
        }

        x_avg = x_avg / list.size();
        y_avg = y_avg / list.size();

        // Return the averaged pose
        return new AprilTagPoseFtc(x_avg, y_avg, 0, 0, 0, 0, 0, 0, 0);
    }

    private void initDetection() {
        // Initialize the processors
        this.propDetectionProcessor = new PropDetectionProcessor(DarvinciCore.AllianceType.BLUE);
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

        // Build the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(Webcam)
                // Keep at 640x480
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .addProcessors(propDetectionProcessor, aprilTagProcessor);

        // Vision portal should be prop detection first
        this.Vision = builder.build();
        this.Vision.setProcessorEnabled(aprilTagProcessor, false);
        this.Vision.setProcessorEnabled(propDetectionProcessor, true);
    }
}

