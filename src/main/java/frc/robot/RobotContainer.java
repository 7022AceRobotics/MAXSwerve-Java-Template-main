// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlgaeShoot;
import frc.robot.commands.AlgaeSuck;
import frc.robot.commands.driveToCoralStation;
import frc.robot.commands.driveToLeft;
import frc.robot.commands.driveToRight;
import frc.robot.commands.limelight;
import frc.robot.commands.moveElevatorTo;
import frc.robot.commands.pivotTo;
import frc.robot.commands.resetPID;
import frc.robot.commands.score2;
import frc.robot.commands.shoot;
import frc.robot.commands.suck;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightHelpersC;
import frc.robot.subsystems.PhotonVisionHelper;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RudolphTheReindeer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@Logged(strategy = Strategy.OPT_IN)
public class RobotContainer {
  // The robot's subsystems
  @Logged(name = "Drive")
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The robot's limelight subsystems
  private final LimelightHelpers m_limelight_subsystem = new LimelightHelpers();
  private final LimelightHelpersC m_limelight_subsystem2 = new LimelightHelpersC();

  private final PhotonVisionHelper m_photon_vision_subsystem = new PhotonVisionHelper(m_robotDrive);

  private final AlgaeCollectorSubsystem m_algae_collector_subsystem = new AlgaeCollectorSubsystem();
  @Logged(name = "Pivot")
  private final PivotSubsystem m_pivot_subsystem = new PivotSubsystem();
  private final ShooterSubsystem m_shooter_subsystem = new ShooterSubsystem();
  
  @Logged(name = "Elevator")
  private final ElevatorSubsystem m_elevator_subsystem = new ElevatorSubsystem();

  private final RudolphTheReindeer m_led_subsystem = new RudolphTheReindeer();

  private final Optional<Alliance> ally = DriverStation.getAlliance();

  private final SendableChooser<Command> autoChooser;

  // The robot's logging system
  //private DataLogManager DataLogManager = new DataLogManager();


  // Swerve Pose Estimator
  //@SuppressWarnings("rawtypes")
  //private final PoseEstimator m_swerve_pose_estimator = new PoseEstimator(DriveConstants.kDriveKinematics, new Rotation2d(), var1, new Pose2d());
  //private final SwerveDrivePoseEstimator m_differential_drive_pose_estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_robotDrive.getHeading(), m_robotDrive.getSwerveModulePositions(), new Pose2d());

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  PS4Controller m_operator_controller = new PS4Controller(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, ally),
            m_robotDrive));

    m_limelight_subsystem2.setDefaultCommand(new limelight(m_limelight_subsystem2, m_robotDrive));
    m_shooter_subsystem.setDefaultCommand(new shoot(m_shooter_subsystem, () -> m_driverController.getRawAxis(2)));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    DataLogManager.start();

    NamedCommands.registerCommand("L1", new suck(m_shooter_subsystem).withTimeout(10));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, Button.kCircle.value).toggleOnTrue(
      new driveToLeft(m_robotDrive, m_limelight_subsystem)
    );

     new JoystickButton(m_driverController, Button.kTriangle.value).whileTrue(
      new RunCommand(() -> m_robotDrive.zeroHeading())
     );

    new JoystickButton(m_driverController, Button.kSquare.value).toggleOnTrue(
      new driveToCoralStation(m_robotDrive, m_photon_vision_subsystem)
    );
    new JoystickButton(m_driverController, Button.kCross.value).toggleOnTrue(
      new driveToRight(m_robotDrive, m_limelight_subsystem)
    );

    // new JoystickButton(m_driverController, Button.kR1.value).whileTrue(
    //   new AlgeaSuck(m_algae_collector_subsystem)
    // );

    // new JoystickButton(m_driverController, Button.kL1.value).whileTrue(
    //   new AlgeaShoot(m_algae_collector_subsystem)
    // ); 

    new JoystickButton(m_operator_controller, 10).whileTrue(
      new score2(m_elevator_subsystem, m_pivot_subsystem,m_robotDrive,m_shooter_subsystem, 1)
    ); // smartdashboard Start
    new JoystickButton(m_operator_controller, 9).whileTrue(
      new score2(m_elevator_subsystem, m_pivot_subsystem,m_robotDrive,m_shooter_subsystem, 2)
    ); // Level 2 Back
    new JoystickButton(m_operator_controller, 4).whileTrue(
      new score2(m_elevator_subsystem, m_pivot_subsystem,m_robotDrive,m_shooter_subsystem, 3)
    ); // Level 3 Y
    new JoystickButton(m_operator_controller, 5).whileTrue(
      new score2(m_elevator_subsystem, m_pivot_subsystem,m_robotDrive,m_shooter_subsystem, 4)
    ); // Level 4 LB
    new JoystickButton(m_operator_controller, 6).whileTrue(
      new score2(m_elevator_subsystem, m_pivot_subsystem,m_robotDrive,m_shooter_subsystem, 5)
    ); // Level 1 RB
    new JoystickButton(m_operator_controller, 8).whileTrue(
      new score2(m_elevator_subsystem, m_pivot_subsystem,m_robotDrive,m_shooter_subsystem, 6)
    ); // intake position RT
    new JoystickButton(m_operator_controller, 1).whileTrue(
      new score2(m_elevator_subsystem, m_pivot_subsystem,m_robotDrive,m_shooter_subsystem, 0)
    ); // rest posistion ; x

    // new JoystickButton(m_driverController, Button.kSquare.value).whileTrue(4
    //   new resetPID(m_pivot_subsystem, m_elevator_subsystem, "Elevator")
    // );
    new JoystickButton(m_operator_controller, 2).whileTrue(
      new shoot(m_shooter_subsystem)
    ); // intake/outake A
    new JoystickButton(m_operator_controller, 3).whileTrue(
      new suck(m_shooter_subsystem)
      ); // Reverse B

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
    //config.setReversed(true);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(180)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1,0 , new Rotation2d(180)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

     return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, DriverStation.getAlliance()));

    // Run path following command, then stop at the end.
     //return autoChooser.getSelected();
  }
}
