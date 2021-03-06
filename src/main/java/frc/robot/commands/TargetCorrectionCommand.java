package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivebaseSubsystem;



public class TargetCorrectionCommand extends CommandBase {
    // The subsystem the command runs on
    private final DrivebaseSubsystem m_drivebaseSubsystem;
    private NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    private NetworkTable table = ntinst.getTable("PowerTower");

    private NetworkTableEntry difference = table.getEntry("difference");

    private boolean m_startedMotor;

    public TargetCorrectionCommand(DrivebaseSubsystem subsystem) {
        m_drivebaseSubsystem = subsystem;
        addRequirements(m_drivebaseSubsystem);

        m_startedMotor = false;
    }
  
    @Override
    public void initialize() {
        System.out.println("Target Correction Started");
        m_drivebaseSubsystem.setBrakeMode();
        RobotContainer.getAcquisitionSubsystem().deployAcquirer(false);
        RobotContainer.getAcquisitionSubsystem().stopAcquirerMotor();
        RobotContainer.getStorageSubsystem().stopDrumMotor();
    }
  
    @Override
    public boolean isFinished() {
        difference = table.getEntry("difference");
        
        double diff = difference.getDouble(-1000);
        System.out.println("Outside Difference: " + diff);
        if ((diff < 480.0) && (diff > -480.0)){
            m_drivebaseSubsystem.allowDriving(false);

            double magnitude = Math.abs(diff);

            if(magnitude > 100) {
                if(diff > 0) {
                    System.out.println("Turn left fast");
                    m_drivebaseSubsystem.turnLeft(0.1);
                } else {
                    System.out.println("Turn right fast");
                    m_drivebaseSubsystem.turnRight(0.1);
                }
            } else {

                if(!RobotContainer.getShooterSubsystem().isShooting()) {
                    System.out.println("Starting the shooter motor");
                    RobotContainer.getShooterSubsystem().startShooterMotor();
                }

                if (diff > 5.0){ 
                    System.out.println("Turn left slow");
                    m_drivebaseSubsystem.turnLeft(0.075);
                }
                else if(diff < -5.0){
                    System.out.println("Turn right slow");
                    m_drivebaseSubsystem.turnRight(0.075);
                }
                else {
                    if(RobotContainer.getShooterSubsystem().isShooterMotorUpToSpeed()) {
                        RobotContainer.getShootEntireDrumCommand().schedule();
                        return true;
                    } else {
                        System.out.println("Waiting for shooter motor to come up to speed");
                    }
                    
                }
            }
        } else {
            System.out.println("Outside Difference: " + diff);
        }

      return false;
    }

    @Override
    public void end(boolean interuppted){
        m_drivebaseSubsystem.stop();
        m_drivebaseSubsystem.setCoastMode();
        m_drivebaseSubsystem.allowDriving(true);
        new ShooterMotorAdjustmentCommand(RobotContainer.getShooterSubsystem()).schedule(true);
    }
  }
