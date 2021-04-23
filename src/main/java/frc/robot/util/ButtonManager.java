package frc.robot.util;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.commands.TargetCorrectionCommand;
import frc.robot.subsystems.AcquisitionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class ButtonManager {

    // Initializes the xbox controller at port 0
    private final static OI m_oi = new OI();

    private ShooterSubsystem m_shooterSubsystem;
    private StorageSubsystem m_storageSubsystem;
    private AcquisitionSubsystem m_acquisitionSubsystem;
    
    public ButtonManager(ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, AcquisitionSubsystem acquisitionSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        m_storageSubsystem = storageSubsystem;
        m_acquisitionSubsystem = acquisitionSubsystem;
    }

    public void configureButtonBindings() {
        Button rightTrigger = new Button(OI::getXboxRightTriggerPressed);
        Button leftTrigger = new Button(OI::getXboxLeftTriggerPressed);
        Button rightBumper = new Button(OI::getXboxRightBumper);
        Button leftBumper = new Button(OI::getXboxLeftBumper);

        Button aButton = new Button(OI::getXboxAButton);
        Button bButton = new Button(OI::getXboxBButton);
        Button xButton = new Button(OI::getXboxXButton);
        Button yButton = new Button(OI::getXboxYButton);

        Button dpadUp = new Button(OI::getXboxDpadUp);
        Button dpadDown = new Button(OI::getXboxDpadDown);
        Button dpadLeft = new Button(OI::getXboxDpadLeft);
        Button dpadRight = new Button(OI::getXboxDpadRight);

        // Reverses the acquirer only if the acquirer is deployed
        rightTrigger.whenPressed(
            new InstantCommand(() -> {
                if(m_acquisitionSubsystem.isAcquirerDeployed()) {
                    m_acquisitionSubsystem.startAcquirerMotorReverse();
                }
            })
        );

        // If the acquirer is deployed put the acquirer back to normal direction
        rightTrigger.whenReleased(
            new InstantCommand(() -> {
                if(m_acquisitionSubsystem.isAcquirerDeployed()) {
                    m_acquisitionSubsystem.startAcquirerMotor();
                }
            })
        );


        // Acquirer button
        rightBumper.whenPressed(
            new SelectCommand(
                Map.ofEntries(
                    Map.entry(true, new InstantCommand(() -> {
                                        m_acquisitionSubsystem.retractAcquirer();
                                        m_storageSubsystem.stopDrumMotor();
                                    }).withName("Stop Acquirer Motor SeqCommand")),
                    Map.entry(false, new InstantCommand(() -> {
                                        m_acquisitionSubsystem.deployAcquirer(true);
                                        m_storageSubsystem.startDrumMotor(Constants.DRUM_MOTOR_VELOCITY_SLOW);
                                    }).withName("Start Acquirer Motor SeqCommand"))
                ),
                m_acquisitionSubsystem::isAcquirerDeployed
            ).withName("Acquirer Select Command")
        );

        bButton.toggleWhenPressed(
            new SequentialCommandGroup(
                new InstantCommand(() -> m_acquisitionSubsystem.deployAcquirer(true)),
                new WaitCommand(0.5),
                new TargetCorrectionCommand(
                    RobotContainer.getDrivebaseSubsystem(), 
                    RobotContainer.getAcquisitionSubsystem(),
                    RobotContainer.getShooterSubsystem()
                ).beforeStarting(() -> m_storageSubsystem.stopDrumMotor())
            ), true
        );

        aButton.whenPressed(
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    OI.setXboxRumbleStop();
                    m_shooterSubsystem.setIsShooting(false);
                    m_storageSubsystem.stopDrumMotor();
                    m_shooterSubsystem.stopShooterMotor();
                    RobotContainer.getDrivebaseSubsystem().allowDriving(true);
                    RobotContainer.getDrivebaseSubsystem().stop();
                    CommandScheduler.getInstance().cancelAll();
                })
            )
        );

        // ---------------- Diagnostic Buttons ----------------
        // Allocate available buttons when testing

        // Reset Lidar
        // bButton.whenPressed(new InstantCommand(() -> m_lidarManager.getLidar().reset()));

        // Index Drum
        //bButton.whenPressed(new InstantCommand(() -> m_storageSubsystem.startDrumMotor(Constants.DRUM_MOTOR_VELOCITY_SLOW)).beforeStarting(() -> m_storageSubsystem.doIndexing(true)));
        
        // Start Drum
        //xButton.whenPressed(new InstantCommand(() -> m_storageSubsystem.startDrumMotor(Constants.DRUM_MOTOR_VELOCITY_VERY_SLOW)));

        // Stop Drum
        //yButton.whenPressed(new InstantCommand(() -> m_storageSubsystem.stopDrumMotor()));

        // Start Bar Motor
        // bButton.whenPressed(new InstantCommand(() -> m_storageSubsystem.startBarMotor()));

        // Stop Bar Motor 
        // bButton.whenPressed(new InstantCommand(() -> m_storageSubsystem.stopBarMotor()));
        
        // Start Shooter Motor
        // bButton.whenPressed(new InstantCommand(() -> m_shooterSubsystem.startShooterMotor()));

        // Bopper
        // bButton.whenPressed(RobotContainer.getBopperCommand());

        // Print "B Button"
        // bButton.whenPressed(new PrintCommand("B Button"));
    }

    public static OI getOI() {
        return m_oi;
    }
}
