package org.firstinspires.ftc.team5100.MolotovCock.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team5100.MolotovCock.roadrunner.drive.MolotovChassis;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.team5100.MolotovCock.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "TestAuto BLUE LEFT (younglings)", group = "Test")
public class BL extends LinearOpMode {
    // Components
    private MolotovChassis drive;
    private DcMotor slide, lifter;
    private Servo lifterreleaser;
    private Servo pinch;
    private Servo angelo;
    private Servo deltatigers;

    // Vision
    private TfodProcessor tfod;
    private static final String[] LABELS = {
            "BlueTin"
    };
    private VisionPortal visionPortal;
    private String detectionResult = "Right";

    // Other
    private int slideInitial;

    @Override
    public void runOpMode() {
        drive = new MolotovChassis(hardwareMap);
        drive.setPoseEstimate(new Pose2d(10.67, 62.88, Math.toRadians(270)));

        slide = hardwareMap.get(DcMotor.class, "slide");
        pinch = hardwareMap.get(Servo.class, "pinch");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        lifterreleaser = hardwareMap.get(Servo.class, "lifterreleaser");
        deltatigers = hardwareMap.get(Servo.class, "deltatigers");
        angelo = hardwareMap.get(Servo.class, "angelo");

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideInitial = slide.getCurrentPosition();
        slide.setTargetPosition(slideInitial);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);

        deltatigers.setDirection(Servo.Direction.REVERSE);
        angelo.setDirection(Servo.Direction.REVERSE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        final int slideMId = 1100;
        final int liftersart = lifter.getCurrentPosition();

        lifter.setTargetPosition(liftersart + 20);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        final double OPEN = 0.22;
        final double CLOSE = 0.30;
        pinch.setPosition(CLOSE); //open

        final double TMINUS = 0.35;
        final double LAUNCH = 0.0;
        deltatigers.setPosition(TMINUS); //not launched

        final double FLAT = 0.78;
        angelo.setPosition(FLAT); //flat

        initTensorFlow();

        while (!opModeIsActive()) {
            detectionResult = getDetection();
            telemetry.addData("Detected", detectionResult);
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        if (opModeIsActive()) {
            if (detectionResult.equals("Left")) {
                TrajectorySequence leftTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(10.67, 36))
                        .turn(Math.toRadians(180))
                        .addDisplacementMarker(() -> {
                            pinch.setPosition(OPEN);
                        })
                        .turn(Math.toRadians(270))
                        .back(7)
                        .lineToSplineHeading(new Pose2d(48,36, Math.toRadians(355)))
                        .build();
                drive.followTrajectorySequence(leftTrajectory);
            } else if (detectionResult.equals("Center")) {
                TrajectorySequence middleTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .forward(27.5)
                        .addDisplacementMarker(25, () -> {
                            pinch.setPosition(OPEN);
                        })
                        .back(7)
                        .lineToSplineHeading(new Pose2d(48, 34, Math.toRadians(0)))
                        .build();
                drive.followTrajectorySequence(middleTrajectory);
            } else {
                TrajectorySequence rightTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(10.67, 36))
                        .turn(Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            pinch.setPosition(OPEN);
                        })
                        .turn(Math.toRadians(270))
                        .waitSeconds(1)
                        .lineToSplineHeading(new Pose2d(48,36, Math.toRadians(355)))
                        .waitSeconds(1)
                        .build();
                drive.followTrajectorySequence(rightTrajectory);
            }

            angelo.setPosition(0.2);
            sleep(1000);
            pinch.setPosition(CLOSE);
            sleep(1000);

            slide.setTargetPosition(slideMId - 250);
            sleep(1000);
            TrajectorySequence thisisdumb = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(4)
                    .build();
            drive.followTrajectorySequence(thisisdumb);
            angelo.setPosition(0.6);
            sleep(1000);
            pinch.setPosition(OPEN);
            sleep(1000);
            TrajectorySequence moveee = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(3)
                    .build();

            drive.followTrajectorySequence(moveee);
            TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> slide.setTargetPosition(slideInitial))
                    .strafeLeft(25)
                    .build();
            drive.followTrajectorySequence(park);
            angelo.setPosition(0.6);
            sleep(1000);
        }
    }
    public void initTensorFlow() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName("ROOKblue.tflite")
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.18f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }

    public String getDetection() {
        String detectedSide = "Right";
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        if (currentRecognitions.size() == 0) {
            detectedSide = "Right";
        }

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (x >= 98.0 && x <= 430.0 && recognition.getConfidence() * 100 > 66.0) {
                detectedSide = "Center";
            } else if (x >= 14.0 && x <= 98.0 && recognition.getConfidence() * 100 > 66.0) {
                detectedSide = "Left";
            }
        }

        return detectedSide;
    }
}