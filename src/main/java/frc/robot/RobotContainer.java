/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Toolkit.CT_CommandToggler;
// import frc.robot.util.TrajectoryManager;
import frc.robot.Toolkit.CT_CommandToggler.CommandState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Initializes the xbox controller at port 0
  private final static OI m_oi = new OI();

  // Robot Subsystems

  // private final static TrajectoryManager m_trajectoryManager = new TrajectoryManager();
  private final static DrivebaseSubsystem m_drivebaseSubsystem = new DrivebaseSubsystem();
  private final static AcquisitionSubsystem m_acquisitionSubsystem = new AcquisitionSubsystem();
  private final static StorageSubsystem m_storageSubsystem = new StorageSubsystem();
  private final static GarminLidarSubsystem m_garminLidarSubsystem = new GarminLidarSubsystem();
  private final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(m_storageSubsystem,
      m_garminLidarSubsystem, m_acquisitionSubsystem);

  private final static SendableChooser<Double> m_manualVelocityChooser = new SendableChooser<>();
  // private final static TrajectoryCommand m_centerTrajectoryCommand = new
  // TrajectoryCommand(m_trajectoryManager.getCenterTrajectory(),
  // m_drivebaseSubsystem);
  // private final static TrajectoryCommand m_leftTrajectoryCommand = new
  // TrajectoryCommand(m_trajectoryManager.getLeftTrajectory(),
  // m_drivebaseSubsystem);
  // private final static TrajectoryCommand m_rightTrajectoryCommand = new
  // TrajectoryCommand(m_trajectoryManager.getRightTrajectory(),
  // m_drivebaseSubsystem);
  // private final static TrajectoryCommand m_basicTrajectoryCommand = new
  // TrajectoryCommand(m_trajectoryManager.getBasicTrajectory(),
  // m_drivebaseSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button and shuffleboard bindings
    configureButtonBindings();
    configureShuffleboardBindings();
  }

  // Use this method to define shuffleboard buttons or widgets
  private void configureShuffleboardBindings() {

    // Velocity Options

    SmartDashboard.putData("Velocity Chooser", m_manualVelocityChooser);
    m_manualVelocityChooser.setDefaultOption("Use Lidar", -1.0);
    m_manualVelocityChooser.addOption("Initiation Line", 75000.0);
    m_manualVelocityChooser.addOption("Trench", 77000.0);
    m_manualVelocityChooser.addOption("Control Panel", 85000.0);
    m_manualVelocityChooser.addOption("Diagnostic Velocity", 30000.0);

    // Command Buttons

    SmartDashboard.putData("Robot machine broke, reset",
        RobotContainer.getResetEverythingCommand().beforeStarting(() -> CommandScheduler.getInstance().cancelAll()));
    SmartDashboard.putData("Reset Drum Array",
        new PrintCommand("resetting drum array").andThen(() -> m_storageSubsystem.resetBallArray()));
    SmartDashboard.putData("Repopulate drum array", getRepopulateArrayCommand());

    // Diagnostic buttons

    SmartDashboard.putData("Rotate Drum Forwards",
        new InstantCommand(m_storageSubsystem::startDrumMotor, m_storageSubsystem));
    SmartDashboard.putData("Rotate Drum Backwards",
        new InstantCommand(m_storageSubsystem::startDrumMotorBackwards, m_storageSubsystem));
    SmartDashboard.putData("Stop Drum Motor",
        new InstantCommand(m_storageSubsystem::stopDrumMotor, m_storageSubsystem));
    SmartDashboard.putData("Change mode to shooting",
        new InstantCommand(() -> m_shooterSubsystem.setIsShooting(true), m_shooterSubsystem));
    SmartDashboard.putData("Change mode to acquiring",
        new InstantCommand(() -> m_shooterSubsystem.setIsShooting(false), m_shooterSubsystem));
    SmartDashboard.putData("Rotate drum one index", getRotateDrumOneSectionCommand());
    SmartDashboard.putData("Bopper", getBopperCommand());
    SmartDashboard.putData("Wiggle", getShakeDialCommand());
    SmartDashboard.putData("Run Acquirer Motor", new InstantCommand(() -> m_acquisitionSubsystem.startAcquirerMotor()));
    SmartDashboard.putData("Stop Acquirer Motor", new InstantCommand(m_acquisitionSubsystem::stopAcquirerMotor));
    SmartDashboard.putData("Deploy Acquirer", new InstantCommand(m_acquisitionSubsystem::deployAcquirer));
    SmartDashboard.putData("Retract Acquirer", new InstantCommand(m_acquisitionSubsystem::retractAcquirer));
    SmartDashboard.putData("Reverse Acquirer", new InstantCommand(m_acquisitionSubsystem::startAcquirerMotorReverse));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

      Button rightTrigger = new Button(OI::getXboxRightTriggerPressed);
      Button leftTrigger = new Button(OI::getXboxLeftTriggerPressed);
      Button rightBumper = new Button(OI::getXboxRightBumper);
      Button leftBumper = new Button(OI::getXboxLeftBumper);

      Button aButton = new Button(OI::getXboxAButton);
      Button bButton = new Button(OI::getXboxBButton);
      Button xButton = new Button(OI::getXboxXButton);
      Button yButton = new Button(OI::getXboxYButton);

      // --------------------------------------Acquirer Buttons-----------------------------------------------------

      rightTrigger.whenPressed(() -> m_acquisitionSubsystem.startAcquirerMotor());
      rightTrigger.whenReleased(() -> m_acquisitionSubsystem.stopAcquirerMotor());

      leftTrigger.whenPressed(() -> m_acquisitionSubsystem.startAcquirerMotorReverse());
      leftTrigger.whenReleased(() -> m_acquisitionSubsystem.stopAcquirerMotor());

      new CT_CommandToggler( // Acquirer Motor Toggle - Right Bumper
          new InstantCommand(m_acquisitionSubsystem::deployAcquirer),
          new InstantCommand(m_acquisitionSubsystem::retractAcquirer)
      )
      .setDefaultState(CommandState.Interruptible)
      .setToggleButton(rightBumper)
      .setCycle(true);

      //--------------------------------------Shooter Buttons--------------------------------------------------------

      new CT_CommandToggler( // Shooter Motor Toggle - Left Bumper
          new InstantCommand(m_shooterSubsystem::startShooterMotor, m_shooterSubsystem),
          new InstantCommand(m_shooterSubsystem::stopShooterMotor, m_shooterSubsystem)
      )
      .setDefaultState(CommandState.Interruptible)
      .setToggleButton(leftBumper)
      .setCycle(true);

      new CT_CommandToggler() // Shoot Entire Drum Toggle - A Button
          .setDefaultState(CommandState.Interruptible)
          .addJumpCommand(
            getShootEntireDrumCommand().beforeStarting(() -> m_shooterSubsystem.setIsShooting(true)),
            CommandState.Interruptible
          )
          .addCommand(null)
          .setToggleButton(aButton)
          .setCycle(true);

      //--------------------------------------Dial Buttons-----------------------------------------------------------

      bButton.whenPressed(getShakeDialCommand());

      yButton.whenPressed(getRepopulateArrayCommand());
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

  // Utilization Commands

  public static RumbleCommand getRumbleCommand(double time) {
    return new RumbleCommand(time);
  }

  public static Double getManualVelocity() {
    return m_manualVelocityChooser.getSelected();
  }

  // Storage Commands

  public static RotateDrumOneSectionCommand getRotateDrumOneSectionCommand() {
    return new RotateDrumOneSectionCommand(m_storageSubsystem, m_shooterSubsystem);
  }

  // Shooting Commands

  public static TryToShootCommand getTryToShootCommand() {
    return new TryToShootCommand(m_shooterSubsystem, m_storageSubsystem);
  }

  public static ShootOnceCommand getShootOnceCommand() {
    return new ShootOnceCommand(m_shooterSubsystem);
  }

  public static ShootEntireDrumCommand getShootEntireDrumCommand() {
    return new ShootEntireDrumCommand(m_shooterSubsystem);
  }

  public static ShootWhenHeldCommand getShootWhenHeld() {
    return new ShootWhenHeldCommand(m_shooterSubsystem);
  }

  public static BopperCommand getBopperCommand() {
    return new BopperCommand(m_shooterSubsystem);
  }

  // Diagnostic Commands

  public static ResetEverythingCommand getResetEverythingCommand() {
    return new ResetEverythingCommand(m_storageSubsystem, m_shooterSubsystem, m_garminLidarSubsystem,
        m_drivebaseSubsystem, m_acquisitionSubsystem);
  }

  public static RepopulateArrayCommand getRepopulateArrayCommand() {
    return new RepopulateArrayCommand(m_storageSubsystem);
  }

  public static ShakeDialCommand getShakeDialCommand() {
    return new ShakeDialCommand(m_storageSubsystem);
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

  public static GarminLidarSubsystem getGarminLidarSubsystem() {
    return m_garminLidarSubsystem;
  }

}
