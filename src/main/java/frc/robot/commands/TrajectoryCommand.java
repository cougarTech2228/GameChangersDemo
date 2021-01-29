package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivebaseSubsystem;

/**
 * TrajectoryCommand
 * 
 *  */
public class TrajectoryCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public TrajectoryCommand(Trajectory trajectory, DrivebaseSubsystem drivebaseSubsystem) {
        addCommands(
            new RamseteCommand(
                trajectory, 
                drivebaseSubsystem::getCurrentPose,
                drivebaseSubsystem.getRamseteController(), 
                drivebaseSubsystem.getFeedforward(),
                drivebaseSubsystem.getDriveKinematics(),
                drivebaseSubsystem::getSpeeds,
                drivebaseSubsystem.getLeftPIDController(),
                drivebaseSubsystem.getRightPIDController(),
                drivebaseSubsystem::setOutputVolts,
                drivebaseSubsystem).andThen(() -> drivebaseSubsystem.stop())
        );
        
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements();
    }
}
