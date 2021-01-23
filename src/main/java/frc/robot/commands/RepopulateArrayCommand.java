package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

/**
 * RepopulateArrayCommand
 * 
 * This command repopulates the drum widget with the correct values
 * Stops in acquire position
 */
public class RepopulateArrayCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private StorageSubsystem m_storageSubsystem;

    public RepopulateArrayCommand(StorageSubsystem storageSubsystem, ShooterSubsystem shooterSubsystem) {
        m_storageSubsystem = storageSubsystem;

        addCommands(
            new InstantCommand(() -> {
                m_storageSubsystem.setIsRepopulating(true);
                m_storageSubsystem.resetBallArray();
                shooterSubsystem.setIsShooting(false);
            }),
            RobotContainer.getIndexDrumCommand(storageSubsystem.getDrumStoragePositionInput(), false),
            new WaitCommand(1),
            RobotContainer.getIndexDrumCommand(storageSubsystem.getDrumStoragePositionInput(), false),
            new WaitCommand(1),
            RobotContainer.getIndexDrumCommand(storageSubsystem.getDrumStoragePositionInput(), false),
            new WaitCommand(1),
            RobotContainer.getIndexDrumCommand(storageSubsystem.getDrumStoragePositionInput(), false),
            new WaitCommand(1),
            RobotContainer.getIndexDrumCommand(storageSubsystem.getDrumStoragePositionInput(), false),
            new WaitCommand(1),
            RobotContainer.getIndexDrumCommand(storageSubsystem.getDrumStoragePositionInput(), false)
            .andThen(() -> m_storageSubsystem.setIsRepopulating(false))
        );


        // addCommands(
        //     new PrintCommand("Repopulate Array Command")
        //     .andThen(() -> m_storageSubsystem.setIsRepopulating(true))
        //     .andThen(() -> m_storageSubsystem.resetBallArray())
        //     .andThen(() -> populateIndex())
        //     .andThen(RobotContainer.getRotateDrumOneSectionCommand())
        //     .andThen(() -> populateIndex())
        //     .andThen(RobotContainer.getRotateDrumOneSectionCommand())
        //     .andThen(() -> populateIndex())
        //     .andThen(RobotContainer.getRotateDrumOneSectionCommand())
        //     .andThen(() -> populateIndex())
        //     .andThen(RobotContainer.getRotateDrumOneSectionCommand())
        //     .andThen(() -> populateIndex())
        //     .andThen(RobotContainer.getRotateDrumOneSectionCommand())
        //     .andThen(() -> m_storageSubsystem.setIsRepopulating(false))
        // );
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements();
    }
    /**
     * Checks the current slot if there is a ball in there, and sets the drum array accordingly.
     */
    public void populateIndex() {
        System.out.println("populateIndex");
        if(m_storageSubsystem.isAcquireBallOccupied()) {
            m_storageSubsystem.getBallArray().acquire();
        }
    }
}