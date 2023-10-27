package org.firstinspires.ftc.team4100.Darvinci.tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team4100.Darvinci.DarvinciCore;
import org.firstinspires.ftc.team4100.Darvinci.vision.PropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Vision Test")
@Disabled
public class VisionTest extends LinearOpMode {
    private FtcDashboard dashboard;
    private PropDetectionProcessor propDetectionProcessor;
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        propDetectionProcessor = new PropDetectionProcessor(DarvinciCore.AllianceType.RED);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(propDetectionProcessor)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Detected", propDetectionProcessor.getDetectedSide());
            sleep(20);
        }

    }
}