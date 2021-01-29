package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

/**
 * ShootEntireDrumCommand
 * 
 * This command shoots the entire drum. It calls tryToShootOnce 5 times.
 */
public class ShootEntireDrumCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public ShootEntireDrumCommand(ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem) {
        addCommands(
            new PrintCommand("Shoot entire drum")
            .andThen(() -> RobotContainer.getIndexDrumCommand(storageSubsystem.getDrumShooterPositionInput(), false)),
            new WaitCommand(0.5),
            RobotContainer.getTryToShootCommand(),
            new WaitCommand(Constants.TIME_BETWEEN_SHOTS),
            RobotContainer.getTryToShootCommand(),
            new WaitCommand(Constants.TIME_BETWEEN_SHOTS),
            RobotContainer.getTryToShootCommand(),
            new WaitCommand(Constants.TIME_BETWEEN_SHOTS),
            RobotContainer.getTryToShootCommand(),
            new WaitCommand(Constants.TIME_BETWEEN_SHOTS),
            RobotContainer.getTryToShootCommand()
            .andThen(() -> {
                shooterSubsystem.setIsShooting(false);
            })
        );
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements();
    }
}