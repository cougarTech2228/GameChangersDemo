/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Toolkit.CT_CommandToggler;
import frc.robot.util.ButtonManager;
import frc.robot.util.LidarManager;
import frc.robot.util.ShooterMotor;
import frc.robot.Toolkit.CT_CommandToggler.CommandState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Robot Subsystems
    private final static DrivebaseSubsystem m_drivebaseSubsystem = new DrivebaseSubsystem();

    // TrajectoryManager must be instantiated after DrivebaseSubsystem since it
    // relies on it
    

    private final static LidarManager m_lidarManager = new LidarManager();
    private final static AcquisitionSubsystem m_acquisitionSubsystem = new AcquisitionSubsystem();
    private final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final static StorageSubsystem m_storageSubsystem = new StorageSubsystem(m_shooterSubsystem);
    private final static ButtonManager m_buttonManager = new ButtonManager(m_shooterSubsystem, m_storageSubsystem, m_acquisitionSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        DriverStation.getInstance().silenceJoystickConnectionWarning(true);

        Thread lidarManagerThread = new Thread(m_lidarManager);
        lidarManagerThread.start();

        // Configure the button and shuffleboard bindings
        m_buttonManager.configureButtonBindings();
        configureShuffleboardBindings();
    }

    // Use this method to define shuffleboard buttons or widgets
    private void configureShuffleboardBindings() {

        // Velocity Options

        
        // SmartDashboard.putData("Cancel all commands", new
        // InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
        // SmartDashboard.putData("Reset Drum Array", new
        // PrintCommand("resetting drum array").andThen(() ->
        // m_storageSubsystem.resetBallArray()));
        // SmartDashboard.putData("Repopulate drum array", getRepopulateArrayCommand());
         
        //     // Diagnostic buttons SmartDashboard.putData("Rotate Drum Forwards", new
        // InstantCommand(m_storageSubsystem::startDrumMotor, m_storageSubsystem));
        // SmartDashboard.putData("Rotate Drum Backwards", new
        // InstantCommand(m_storageSubsystem::startDrumMotorBackwards,
        // m_storageSubsystem)); SmartDashboard.putData("Stop Drum Motor", new
        // InstantCommand(m_storageSubsystem::stopDrumMotor, m_storageSubsystem));
        // SmartDashboard.putData("Change mode to shooting", new InstantCommand(() ->
        // m_shooterSubsystem.setIsShooting(true), m_shooterSubsystem));
        // SmartDashboard.putData("Change mode to acquiring", new InstantCommand(() ->
        // m_shooterSubsystem.setIsShooting(false), m_shooterSubsystem));
        // SmartDashboard.putData("Rotate drum one index",
        // getIndexDrumCommand(m_storageSubsystem.getDrumStoragePositionInput(), true));
        // SmartDashboard.putData("Bopper", getBopperCommand());
        // SmartDashboard.putData("Wiggle", getShakeDialCommand());
        // SmartDashboard.putData("Run Acquirer Motor", new InstantCommand(() ->
        // m_acquisitionSubsystem.startAcquirerMotor()));
        // SmartDashboard.putData("Stop Acquirer Motor", new
        // InstantCommand(m_acquisitionSubsystem::stopAcquirerMotor));
        // SmartDashboard.putData("Deploy Acquirer", new
        // InstantCommand(m_acquisitionSubsystem::deployAcquirer));
        // SmartDashboard.putData("Retract Acquirer", new
        // InstantCommand(m_acquisitionSubsystem::retractAcquirer));
        // SmartDashboard.putData("Reverse Acquirer", new
        // InstantCommand(m_acquisitionSubsystem::startAcquirerMotorReverse));
        
    }

    // Command Getters

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    // Shooting Commandss

    public static ShootEntireDrumCommand getShootEntireDrumCommand(int numOfShots) {
        return new ShootEntireDrumCommand(m_shooterSubsystem, m_storageSubsystem, m_acquisitionSubsystem, numOfShots);
    }

    public static BopperCommand getBopperCommand() {
        return new BopperCommand(m_shooterSubsystem);
    }

    public static RumbleCommand getRumbleCommand(double time) {
        return new RumbleCommand(time);
    }

    // Subsystem Getters

    public static DrivebaseSubsystem getDrivebaseSubsystem() {
        return m_drivebaseSubsystem;
    }

    public static AcquisitionSubsystem getAcquisitionSubsystem() {
        return m_acquisitionSubsystem;
    }

    public static StorageSubsystem getStorageSubsystem() {
        return m_storageSubsystem;
    }

    public static ShooterSubsystem getShooterSubsystem() {
        return m_shooterSubsystem;
    }

    public static LidarManager getLidarManager() {
        return m_lidarManager;
    }
}