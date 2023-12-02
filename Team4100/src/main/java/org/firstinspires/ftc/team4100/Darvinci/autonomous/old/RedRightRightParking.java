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
@Disabled
@Config
@Autonomous(name="Old Red [R, RP]", group="Old")
public class RedRightRightParking extends LinearOpMode {
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

    private final Pose2d START_POS = new Pose2d(7.5, -62, Math.toRadians(270));

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
        SLIDE_INITIAL = this.Slide.getCurrentPosition();
        this.Slide.setTargetPosition(SLIDE_INITIAL + 25);
        this.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.Slide.setPower(1);

        this.Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        this.Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.Outtake = hardwareMap.get(Servo.class, "Outtake");
        this.Outtake.setPosition(DarvinciSettings.BUCKET_POS_IN);

        this.Push = hardwareMap.get(CRServo.class, "Push");

        this.Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        telemetry = new MultipleTelemetry(telemetry, Dashboard.getTelemetry());
        initDetection();

        while (!opModeIsActive()) {
            DETECTION = propDetectionProcessor.getDetectedSide();
            telemetry.addData("Detected", DETECTION);
            telemetry.update();
            sleep(10);
        }

        waitForStart();

        Vision.setProcessorEnabled(propDetectionProcessor, false);
        Vision.setProcessorEnabled(aprilTagProcessor, true);

        if (opModeIsActive()) {
            if (DETECTION == PropDetectionProcessor.Detection.LEFT) {
                TrajectorySequence leftTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineTo(new Vector2d(13.5, -58))
                        .lineToSplineHeading(new Pose2d(10.5, -33, Math.toRadians(345)),
                                DarvinciChassis.getVelocityConstraint(20, Math.toRadians(30), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(20))
                        .back(4)
                        .lineToSplineHeading(new Pose2d(40, -30, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(leftTraj);
            } else if (DETECTION == PropDetectionProcessor.Detection.MIDDLE) {
                TrajectorySequence middleTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineTo(new Vector2d(11.5, -34))
                        .forward(10)
                        .lineToSplineHeading(new Pose2d(40, -37, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(middleTraj);
            } else {
                TrajectorySequence rightTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineToSplineHeading(new Pose2d(19.5, -40, Math.toRadians(268)))
                        .forward(14)
                        .lineToSplineHeading(new Pose2d(40, -42, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(rightTraj);
            }

            AprilTagPoseFtc aprilTagPose = getAprilTagPose();

            if (aprilTagPose != null) {
                Pose2d currentPose = Drive.getPoseEstimate();

                TrajectorySequence aprilTagTraj = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(currentPose.getX() + (aprilTagPose.y - 2), currentPose.getY() + (aprilTagPose.x + 1.5), Math.toRadians(180)))
                        .addDisplacementMarker(() -> {
                            Slide.setTargetPosition(SLIDE_INITIAL + 690);
                            Slide.setPower(1);
                        })
                        .build();
                Drive.followTrajectorySequence(aprilTagTraj);

                Outtake.setPosition(DarvinciSettings.BUCKET_POS_OUT - 0.06);
                sleep(650);
                Push.setPower(1);
                sleep(450);
                Push.setPower(0);
                sleep(350);
                Push.setPower(-1);
                sleep(3300);
                Slide.setTargetPosition(Slide.getCurrentPosition() + 70);
                sleep(850);
                Push.setPower(0);
            }

//            Drive.setMotorPowers(-0.1, -0.1, -0.1, -0.1);
//            sleep(450);

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

            TrajectorySequence parkTraj = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(41, -59, Math.toRadians(180)))
                    .lineToSplineHeading(new Pose2d(57, -59, Math.toRadians(180)))
                    .build();
            Drive.followTrajectorySequence(parkTraj);

            PoseStorage.currentPose = Drive.getPoseEstimate();
        }
    }

    private AprilTagPoseFtc getAprilTagPose() {
        ArrayList<AprilTagPoseFtc> list = new ArrayList<>();
        int attempts = 0;
        final int DETECTION_ID = DETECTION == PropDetectionProcessor.Detection.LEFT ? 4 : DETECTION == PropDetectionProcessor.Detection.MIDDLE ? 5 : 6;

        while (true) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == DETECTION_ID) {
                    list.add(detection.ftcPose);
                }
            }

            if (++attempts > 9) {
                break;
            }

            telemetry.update();
            sleep(20);
        }

        Vision.close();

        if (list.size() == 0) {
            return null;
        }

        double x_avg = 0;
        double y_avg = 0;

        for (AprilTagPoseFtc pose : list) {
            x_avg += pose.x;
            y_avg += pose.y;
        }

        x_avg = x_avg / list.size();
        y_avg = y_avg / list.size();

        return new AprilTagPoseFtc(x_avg, y_avg, 0, 0, 0, 0, 0, 0, 0);
    }

    private void initDetection() {
        this.propDetectionProcessor = new PropDetectionProcessor(DarvinciCore.AllianceType.RED);
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
                .addProcessors(propDetectionProcessor, aprilTagProcessor);

        this.Vision = builder.build();
        this.Vision.setProcessorEnabled(aprilTagProcessor, false);
        this.Vision.setProcessorEnabled(propDetectionProcessor, true);
    }
}

