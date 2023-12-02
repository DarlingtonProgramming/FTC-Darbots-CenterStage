package org.firstinspires.ftc.team4100.Scrappy;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team4100.Scrappy.commands.BulkCacheHandler;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.drive.ScrappyChassis;
import org.firstinspires.ftc.team4100.Scrappy.subsystems.ConveyorSubsystem;
import org.firstinspires.ftc.team4100.Scrappy.subsystems.DropperSubsystem;
import org.firstinspires.ftc.team4100.Scrappy.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team4100.Scrappy.subsystems.LiftSubsystem;
import org.firstinspires.ftc.team4100.Scrappy.subsystems.PlaneSubsystem;

public class ScrappyCore extends Robot {
    public enum AllianceType {
        BLUE, RED
    }

    public enum AllianceSide {
        LEFT, RIGHT
    }

    public final AllianceType ALLIANCE_TYPE;
    public final AllianceSide ALLIANCE_SIDE;

    // Subsystems
    public ScrappyChassis m_drive;
    public IntakeSubsystem m_intake;
    public LiftSubsystem m_lift;
    public ConveyorSubsystem m_conveyor;
    public DropperSubsystem m_dropper;
    public PlaneSubsystem m_plane;

    public ScrappyCore(HardwareMap hardwareMap, AllianceType allianceType, AllianceSide allianceSide) {
        ALLIANCE_TYPE = allianceType;
        ALLIANCE_SIDE = allianceSide;

        // Schedule to clear cache continuously (manual mode)
        schedule(new BulkCacheHandler(hardwareMap));

        // Initialize subsystems
        m_drive = new ScrappyChassis(hardwareMap);
        m_intake = new IntakeSubsystem(hardwareMap);
        m_lift = new LiftSubsystem(hardwareMap);
        m_conveyor = new ConveyorSubsystem(hardwareMap);
        m_dropper = new DropperSubsystem(hardwareMap);
        m_plane = new PlaneSubsystem(hardwareMap);
    }
}