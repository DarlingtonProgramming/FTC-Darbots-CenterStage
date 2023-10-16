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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name="Blue - Left")
public class BlueLeft extends LinearOpMode {
    private FtcDashboard Dashboard;
    private List<LynxModule> Hubs;
    private DarvinciChassis Drive;
    private DcMotorEx Slide, Intake;
    private Servo Outtake;
    private CRServo Push;

    private WebcamName Webcam;
    private VisionPortal Vision;
    private AprilTagProcessor aprilTagProcessor;
    public static String DETECTION = "right";

    private final Pose2d START_POS = new Pose2d(10.5, 62, Math.toRadians(90));

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
        this.Slide.setTargetPosition(62);
        this.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.Slide.setPower(1);

        this.Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        this.Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.Outtake = hardwareMap.get(Servo.class, "Outtake");
        this.Outtake.setPosition(0.75);

        this.Push = hardwareMap.get(CRServo.class, "Push");

        this.Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        telemetry = new MultipleTelemetry(telemetry, Dashboard.getTelemetry());
        initDetection();
        waitForStart();

        if (opModeIsActive()) {
            if (DETECTION.equals("left")) {
                TrajectorySequence middleTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineToSplineHeading(new Pose2d(21, 38, Math.toRadians(92)))
                        .waitSeconds(1)
                        .forward(14)
                        .lineToSplineHeading(new Pose2d(40, 42, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(middleTraj);
            } else if (DETECTION.equals("middle")) {
                TrajectorySequence middleTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineTo(new Vector2d(11.5, 34))
                        .waitSeconds(1)
                        .forward(10)
                        .lineToSplineHeading(new Pose2d(40, 35, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(middleTraj);
            } else {
                TrajectorySequence middleTraj = Drive.trajectorySequenceBuilder(START_POS)
                        .lineTo(new Vector2d(13.5, 58))
                        .lineToSplineHeading(new Pose2d(10.5, 33, Math.toRadians(17)),
                                DarvinciChassis.getVelocityConstraint(20, Math.toRadians(30), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(20))
                        .back(1.3)
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(40, 28, Math.toRadians(180)))
                        .build();
                Drive.followTrajectorySequence(middleTraj);
            }

            AprilTagPoseFtc aprilTagPose = getAprilTagPose();
            Pose2d currentPose = Drive.getPoseEstimate();

            if (aprilTagPose != null) {
                telemetry.addData("april x", aprilTagPose.x);
                telemetry.addData("april y", aprilTagPose.y);
                telemetry.addData("current x", currentPose.getX());
                telemetry.addData("current y", currentPose.getY());
                telemetry.update();

                // -0.03
                // 13.3
                // 39.585
                // 27.92

                TrajectorySequence aprilTagTraj = Drive.trajectorySequenceBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(currentPose.getX() + (aprilTagPose.y - 2), currentPose.getY() + (aprilTagPose.x - (DETECTION.equals("right") ? 2 : 3))))
                        .addDisplacementMarker(() -> {
                            Slide.setTargetPosition(596);
                            Slide.setPower(1);
                        })
                        .build();
                Drive.followTrajectorySequence(aprilTagTraj);

                Outtake.setPosition(0.05);
                Push.setPower(0.8);
                sleep(2000);
            }

            TrajectorySequence parkTraj = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                    .addDisplacementMarker(0, () -> {
                        Outtake.setPosition(0.75);
                    })
                    .addDisplacementMarker(10, () -> {
                        Slide.setTargetPosition(62);
                        Slide.setPower(1);
                    })
                    .setConstraints(DarvinciChassis.getVelocityConstraint(30, Math.toRadians(30), DriveConstants.TRACK_WIDTH),
                            DarvinciChassis.getAccelerationConstraint(30))
                    .lineToSplineHeading(new Pose2d(41, 59, Math.toRadians(180)))
                    .lineToSplineHeading(new Pose2d(59.5, 59.5, Math.toRadians(180)))
                    .build();
            Drive.followTrajectorySequence(parkTraj);
        }
    }

    private AprilTagPoseFtc getAprilTagPose() {
        int attempts = 0;
        final int DETECTION_ID = DETECTION.equals("left") ? 1 : DETECTION.equals("middle") ? 2 : 3;

        while (true) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == DETECTION_ID) {
                    Vision.close();
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

        this.Vision = builder.build();
//        Dashboard.startCameraStream((CameraStreamSource) aprilTagProcessor, 0);
    }
}

