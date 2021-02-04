package frc.robot.util;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.TrajectoryCommand;

public class TrajectoryManager implements Runnable {

    private TrajectoryConfig m_config;

    private Trajectory m_basicTrajectory;
    private Trajectory m_barrelRacingTrajectory;
    private Trajectory m_slalomTrajectory;
    private Trajectory m_bounceTrajectory;
    private Trajectory m_GSBlueA;
    private Trajectory m_GSBlueB;
    private Trajectory m_GSRedA;
    private Trajectory m_GSRedB;
    private Command[] m_galacticTrajectories;

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
        //m_config);

        // m_basicTrajectory = makeTrajectory("Basic", 
        // List.of(new Translation2d(1.0, 0.0),
        //         new Translation2d(2.0, 0.0),
        //         new Translation2d(3.0, 0.0)),       
        //         new Pose2d(4.0, 0.0, new Rotation2d(0.0)));
        
        // RobotContainer.setBasicTrajectoryCommand(new TrajectoryCommand(m_basicTrajectory, RobotContainer.getDrivebaseSubsystem()));

        // Trajectories are read from Pathweaver .json file; place file in src/main/deploy before building
        try {
            m_barrelRacingTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/BarrelRacing5.wpilib.json"));
            CommandBase cmd = new TrajectoryCommand(m_barrelRacingTrajectory, RobotContainer.getDrivebaseSubsystem()).beforeStarting(() -> {
                RobotContainer.getDrivebaseSubsystem().resetOdometry(m_barrelRacingTrajectory.getInitialPose());
            });
            RobotContainer.setBarrelRacingTrajectoryCommand(cmd);
            // RobotContainer.getDrivebaseSubsystem().resetOdometry(m_barrelRacingTrajectory.getInitialPose());
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        try {
            m_slalomTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Slalom.wpilib.json"));
            RobotContainer.setSlalomTrajectoryCommand(new TrajectoryCommand(m_slalomTrajectory, RobotContainer.getDrivebaseSubsystem()));
            CommandBase cmd = new TrajectoryCommand(m_slalomTrajectory, RobotContainer.getDrivebaseSubsystem()).beforeStarting(() -> {
                RobotContainer.getDrivebaseSubsystem().resetOdometry(m_slalomTrajectory.getInitialPose());
            });
            RobotContainer.setSlalomTrajectoryCommand(cmd);
            //RobotContainer.getDrivebaseSubsystem().resetOdometry(m_slalomTrajectory.getInitialPose());
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        try {
            Trajectory a3 = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Bounce-A3.wpilib.json"));
            Trajectory a6 = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Bounce-A6.wpilib.json"));
            Trajectory a9 = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Bounce-A9.wpilib.json"));
            Trajectory finish = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Bounce-finish.wpilib.json"));
            SequentialCommandGroup command = new SequentialCommandGroup(
                new TrajectoryCommand(a3, RobotContainer.getDrivebaseSubsystem()).beforeStarting(() -> {
                        RobotContainer.getDrivebaseSubsystem().resetOdometry(a3.getInitialPose());
                    }),
                new TrajectoryCommand(a6, RobotContainer.getDrivebaseSubsystem()),
                new TrajectoryCommand(a9, RobotContainer.getDrivebaseSubsystem()),
                new TrajectoryCommand(finish, RobotContainer.getDrivebaseSubsystem())
            );

            RobotContainer.setBounceTrajectoryCommand(command);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        try {
            m_GSBlueA = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Slalom.wpilib.json"));
            m_GSBlueB = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/GS-B-Blue.wpilib.json"));
            m_GSRedA = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Slalom.wpilib.json"));
            m_GSRedB = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/GS-B-Red.wpilib.json"));

            m_galacticTrajectories[0] = createTrajectory(m_GSBlueA);
            m_galacticTrajectories[1] = createTrajectory(m_GSBlueB);
            m_galacticTrajectories[2] = createTrajectory(m_GSRedA);
            m_galacticTrajectories[3] = createTrajectory(m_GSRedB);

            RobotContainer.setGalacticSearchTrajectoryCommands(m_galacticTrajectories);

        } catch (IOException e) {
            e.printStackTrace();
        }

        System.out.println("Auto paths created");
        RobotContainer.configureAutoChooser();
        
    }

    /**
     * Creates a basic trajectory and resets the odometry of the path to the initial pose
     * @param trajectory the trajectory to be created
     * @return the complete command group
     */
    private SequentialCommandGroup createTrajectory (Trajectory trajectory) {
        return new TrajectoryCommand(trajectory, RobotContainer.getDrivebaseSubsystem()).beforeStarting(() -> {
            RobotContainer.getDrivebaseSubsystem().resetOdometry(trajectory.getInitialPose());
        });
    }
}