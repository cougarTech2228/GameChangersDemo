package frc.robot.util;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.RobotContainer;
import frc.robot.commands.TrajectoryCommand;

public class TrajectoryManager implements Runnable {

    private TrajectoryConfig m_config;

    private Trajectory m_basicTrajectory;
    private Trajectory m_barrelRacingTrajectory;
    private Trajectory m_slalomTrajectory;
    private Trajectory m_bounceTrajectory;

    public TrajectoryManager() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                RobotContainer.getDrivebaseSubsystem().getFeedforward(),
                RobotContainer.getDrivebaseSubsystem().getDriveKinematics(),
                RobotContainer.getDrivebaseSubsystem().getMaxVoltage());

        // Create m_config for all trajectories
        m_config = new TrajectoryConfig(RobotContainer.getDrivebaseSubsystem().getMaxSpeed(),
                RobotContainer.getDrivebaseSubsystem().getMaxAcceleration())
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(RobotContainer.getDrivebaseSubsystem().getDriveKinematics())
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setEndVelocity(0.0);
}
    private Trajectory makeTrajectory(String shuffleBoardName, List<Translation2d> pathList, Pose2d endPose) {
        return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), // Assuming start pose is 0,0
                pathList, endPose, m_config);
    }

    @Override
    public void run() {
        
        // m_basicTrajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d(2.0, 0, new Rotation2d())),
        // m_config);

        m_basicTrajectory = makeTrajectory("Basic", 
        List.of(new Translation2d(1.0, 0.0),
                new Translation2d(2.0, 0.0),
                new Translation2d(3.0, 0.0)),       
                new Pose2d(4.0, 0.0, new Rotation2d(0.0)));
        
        RobotContainer.setBasicTrajectoryCommand(new TrajectoryCommand(m_basicTrajectory, RobotContainer.getDrivebaseSubsystem()));

        // Trajectories are read from Pathweaver .json file; place file in src/main/deploy before building
        try {
            m_barrelRacingTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/BarrelRacing.wpilib.json"));
            RobotContainer.setBarrelRacingTrajectoryCommand(new TrajectoryCommand(m_barrelRacingTrajectory, RobotContainer.getDrivebaseSubsystem()));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
/*
        try {
            m_slalomTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Slalom.wpilib.json"));
            RobotContainer.setSlalomTrajectoryCommand(new TrajectoryCommand(m_slalomTrajectory, RobotContainer.getDrivebaseSubsystem()));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        try {
            m_bounceTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Bounce.wpilib.json"));
            RobotContainer.setBounceTrajectoryCommand(new TrajectoryCommand(m_bounceTrajectory, RobotContainer.getDrivebaseSubsystem()));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        */
    }
}