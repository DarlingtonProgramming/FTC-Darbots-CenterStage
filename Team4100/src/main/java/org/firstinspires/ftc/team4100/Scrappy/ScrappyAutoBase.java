package org.firstinspires.ftc.team4100.Scrappy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.team4100.Darvinci.util.PoseStorage;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore.AllianceType;

public abstract class ScrappyAutoBase extends CommandOpMode {
    public enum Parking {
        LEFT, MIDDLE, RIGHT
    }

    private final AllianceType m_allianceType;
    private final Pose2d m_startingPose;
    protected ScrappyCore robot;

    public ScrappyAutoBase(AllianceType allianceType, Pose2d startPose) {
        m_allianceType = allianceType;
        m_startingPose = startPose;
    }

    @Override
    public void initialize() {
        robot = new ScrappyCore(hardwareMap, m_allianceType);
        robot.m_drive.setPoseEstimate(m_startingPose);
        robot.m_lift.toInitial();
        robot.m_dropper.back();

        while (opModeInInit()) {
            telemetry.addData("Detected", "Nothing");
            telemetry.update();
            sleep(1);
        }

        initAuto();
    }

    @Override
    public void run() {
        super.run();
        PoseStorage.currentPose = robot.m_drive.getPoseEstimate();
    }

    public abstract void initAuto();
}
