package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LidarSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 *
 */
public class ShooterMotorAdjustmentCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private ShooterSubsystem m_shooterSubsystem;
    private LidarSubsystem m_lidarSubsystem;
    private int m_loopCounter;

    public ShooterMotorAdjustmentCommand(ShooterSubsystem shooterSubsystem, LidarSubsystem lidarSubsystem) {

        m_shooterSubsystem = shooterSubsystem;
        m_lidarSubsystem = lidarSubsystem;
        m_loopCounter = 0;

        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Auto adjustment of the shooter motor velocity ended");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_loopCounter >= 25) { // 25 loop iterations (0.5 seconds)
            double curSpeed = m_shooterSubsystem.getShooterMotor().getSpeed();
            int curDistance = (int) (m_lidarSubsystem.getLidarAverage());
            double newSpeed = m_shooterSubsystem.getShooterMotor().closestDistance(curDistance);

            // If the current speed is not zero and the change in speed is greater than or
            // equal to 10000
            if (curSpeed > 0 && Math.abs(newSpeed - curSpeed) > 10000) {
                m_shooterSubsystem.getShooterMotor().start(curDistance);
            }
            m_loopCounter = 0;
        } else {
            m_loopCounter++;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Should either end via this boolean value or being interrupted through the ShootEntireDrumCommand
        return !m_shooterSubsystem.isShooting();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Auto adjustment of the shooter motor velocity ended");
    }
}