package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AcquisitionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

/**
 * ShootEntireDrumCommand
 * 
 * This command shoots the entire drum. It calls tryToShootOnce 5 times.
 */
public class ShootEntireDrumCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private StorageSubsystem m_storageSubsystem;

    public ShootEntireDrumCommand(ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, AcquisitionSubsystem acquisitionSubsystem) {

        m_storageSubsystem = storageSubsystem;

        addCommands(
            // new PrintCommand("Shoot entire drum")
            // .andThen(() -> shooterSubsystem.setIsShooting(true))
            // .andThen(() -> storageSubsystem.startDrumMotor(Constants.DRUM_MOTOR_VELOCITY_FAST)),
            // waitBopIndex(),
            // waitBopIndex(),
            // waitBopIndex(),
            // waitBopIndex(),
            // waitBopIndex()
            // .andThen(() -> shooterSubsystem.setIsShooting(false))
            // .andThen(() -> {
            //     acquisitionSubsystem.retractAcquirer();
            //     storageSubsystem.stopDrumMotor();
            //     storageSubsystem.stopBarMotor();
            //     shooterSubsystem.stopShooterMotor();
            // })

            new SelectCommand(
                Map.ofEntries(
                    Map.entry(true, new SequentialCommandGroup(
                                    new PrintCommand("Shoot entire drum")
                                    .andThen(() -> {
                                        shooterSubsystem.setIsShooting(true);
                                        storageSubsystem.startDrumMotor(Constants.DRUM_MOTOR_VELOCITY_FAST);
                                        //storageSubsystem.startBarMotor();
                                    }),
                                    waitBopIndex(),
                                    waitBopIndex(),
                                    waitBopIndex(),
                                    waitBopIndex(),
                                    waitBopIndex()
                                    .andThen(() -> shooterSubsystem.setIsShooting(false))
                                    .andThen(() -> {
                                        acquisitionSubsystem.retractAcquirer();
                                        storageSubsystem.stopDrumMotor();
                                        //storageSubsystem.stopBarMotor();
                                        shooterSubsystem.stopShooterMotor();
                                    }))),
                    Map.entry(false, new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            System.out.println("Shooter motor is not up to speed");
                                            System.out.println("Current shooter speed: " + shooterSubsystem.getShooterMotor().getSelectedSensorVelocity());
                                            System.out.println("Targeted motor speed: " + shooterSubsystem.getShooterMotor().getFormulaVelocity());           
                                        }),
                                        RobotContainer.getRumbleCommand(0.5)
                                    ))
                    
                ), shooterSubsystem::isShooterMotorUpToSpeed
            )
        );
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements(shooterSubsystem);
    }

    public SequentialCommandGroup waitBopIndex() {
        return new SequentialCommandGroup(
            new WaitCommand(Constants.TIME_BETWEEN_SHOTS),
            RobotContainer.getBopperCommand()
            .andThen(() -> m_storageSubsystem.startDrumMotor(Constants.DRUM_MOTOR_VELOCITY_FAST))
        );
    }
}