package org.firstinspires.ftc.team4100.Scrappy;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team4100.Scrappy.commands.BulkCacheHandler;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.drive.ScrappyChassis;
import org.firstinspires.ftc.team4100.Scrappy.subsystems.ConveyorSubsystem;
import org.firstinspires.ftc.team4100.Scrappy.subsystems.DropperSubsystem;
import org.firstinspires.ftc.team4100.Scrappy.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team4100.Scrappy.subsystems.LiftSubsystem;

public class ScrappyCore extends Robot {
    public enum AllianceType {
        BLUE, RED
    }

    public final AllianceType ALLIANCE_TYPE;

    // Subsystems
    public ScrappyChassis m_drive;
    public IntakeSubsystem m_intake;
    public LiftSubsystem m_lift;
    public ConveyorSubsystem m_conveyor;
    public DropperSubsystem m_dropper;

    public ScrappyCore(HardwareMap hardwareMap, AllianceType allianceType) {
        ALLIANCE_TYPE = allianceType;

        // Schedule to clear cache continuously (manual mode)
        schedule(new BulkCacheHandler(hardwareMap));

        // Initialize subsystems
        m_drive = new ScrappyChassis(hardwareMap);
        m_intake = new IntakeSubsystem(hardwareMap);
        m_lift = new LiftSubsystem(hardwareMap);
        m_conveyor = new ConveyorSubsystem(hardwareMap);
        m_dropper = new DropperSubsystem(hardwareMap);
    }
}