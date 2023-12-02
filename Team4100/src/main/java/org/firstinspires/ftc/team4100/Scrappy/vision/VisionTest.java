package org.firstinspires.ftc.team4100.Scrappy.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Autonomous
@Config
public class VisionTest extends LinearOpMode {
    private PropDetectionProcessor propDetectionProcessor;
    public static boolean isBlue = true;
    public static boolean isLeft = false;

    @Override
    public void runOpMode() throws InterruptedException {
        propDetectionProcessor = new PropDetectionProcessor(isBlue ? ScrappyCore.AllianceType.BLUE : ScrappyCore.AllianceType.RED, isLeft ? ScrappyCore.AllianceSide.LEFT : ScrappyCore.AllianceSide.RIGHT);
        new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(propDetectionProcessor)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Detected", propDetectionProcessor.getDetectedSide());
            sleep(1);
        }

    }
}