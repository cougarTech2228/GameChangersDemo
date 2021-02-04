package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * GalacticSearchAutoCommand
 * 
 * Will use vision to determine which path is the correct one, and run the correct path based on that.
 */
public class GalacticSearchAutoCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public GalacticSearchAutoCommand(VisionSubsystem visionSubsystem) {
        addCommands(
           
        );
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(shooterSubsystem);
    }
}
