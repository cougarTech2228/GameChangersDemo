package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 *
 */
public class ShooterMotorAdjustmentCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private ShooterSubsystem m_shooterSubsystem;

    private int m_loopCounter;

    public ShooterMotorAdjustmentCommand(ShooterSubsystem shooterSubsystem) {

        m_shooterSubsystem = shooterSubsystem;
        m_loopCounter = 0;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Auto adjustment of the shooter motor velocity started");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_shooterSubsystem.isShooterMotorUpToSpeed()) {
            if (m_loopCounter >= 50) { // 50 loop iterations (1 seconds)
                System.out.println("Running adjustment loop");
                double curSpeed = m_shooterSubsystem.getShooterMotor().getSelectedSensorVelocity();
                double newSpeed = m_shooterSubsystem.getShooterMotor().getFormulaVelocity();

                // System.out.println("Is cur speed greater than 0 ? " + (curSpeed > 0));
                // System.out.println("Is the speed difference greater than 1000? " + (Math.abs(newSpeed - curSpeed) > 1000));
                // System.out.println("Is the new speed greater than 0? " + (newSpeed > 0));

                // If the current speed is not zero and the change in speed is greater than or
                // equal to 1000
                if (curSpeed > 0 && Math.abs(newSpeed - curSpeed) > 1000 && newSpeed > 0) {
                    System.out.println("New Speed: " + newSpeed + " curSpeed: " + curSpeed);
                    System.out.println("Changing shooter velocity to: " + newSpeed);
                    m_shooterSubsystem.getShooterMotor().start(m_shooterSubsystem);
                }
                m_loopCounter = 0;
            } else {
                m_loopCounter++;
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Should either end via this boolean value or being interrupted through the
        // ShootEntireDrumCommand
        return !m_shooterSubsystem.isShooting();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Auto adjustment of the shooter motor velocity ended");
    }
}