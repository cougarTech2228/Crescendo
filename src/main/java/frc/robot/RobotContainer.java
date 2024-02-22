// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ButtonBoardSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
  private double MaxSpeed = 4; // 6 meters per second desired top speed
  private double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity
  private Servo driverCameraTiltPWM = new Servo(Constants.kDriverCameraPWMID);
  private double driverCameraTilt;
  private final double driverCameraTiltChange = 0.01;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  public final DrivebaseSubsystem drivetrain = DrivebaseSubsystem.getInstance();
  public final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public final AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();
  public final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
  public final ButtonBoardSubsystem buttonBoardSubsystem = ButtonBoardSubsystem.getInstance();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Path follower */
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final SendableChooser<Command> autoChooser;

  Command shootFrontCommand = new ShootSpeakerCommand(true);
  Command shootSideCommand = new ShootSpeakerCommand(false);

  public void autonomousInit() {
    shooter.initStateMachine(true);
  }

  public void teleopInit() {
    shooter.initStateMachine(shooter.isHoldingNote());
  }

  private int invertForColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return 1;
    }
    return -1;
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        new ConditionalCommand(
            (drivetrain.applyRequest(() -> forwardStraight.withVelocityX(buttonBoardSubsystem.getJoystickX() * MaxSpeed)
                .withVelocityY(buttonBoardSubsystem.getJoystickY() * MaxSpeed))),
            (drivetrain.applyRequest(() -> drive.withVelocityX(invertForColor() * joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                  // forward
                                                                                                                  // with
                // negative Y (forward)
                .withVelocityY(invertForColor() * joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X
                                                                             // (left)
                .ignoringDisable(true)),
            new BooleanSupplier() {
              @Override
              public boolean getAsBoolean() {
                return buttonBoardSubsystem.isDriveOperationMode() &&
                    ((buttonBoardSubsystem.getJoystickX() < -0.1 || buttonBoardSubsystem.getJoystickX() > 0.1) ||
                        (buttonBoardSubsystem.getJoystickY() < -0.1 || buttonBoardSubsystem.getJoystickY() > 0.1));
              };
            }));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    // .applyRequest(() -> point.withModuleDirection(new
    // Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> {
      drivetrain.seedFieldRelative(new Pose2d());
    }));

    // joystick.y().onTrue(new InstantCommand(() -> {
    // shooter.setBender(BenderPosition.SHOOT_SPEAKER);
    // }));
    // joystick.x().onTrue(new InstantCommand(() -> {
    // shooter.setBender(BenderPosition.LOAD_INTERNAL);
    // }));

    // joystick.x().onTrue(new InstantCommand(() -> shooter.loadNote()));
    // joystick.x().onFalse(new InstantCommand(() -> shooter.stopMotors()));

    // joystick.rightBumper().onTrue(new InstantCommand(() -> {
    // var ampPose = aprilTagSubsystem.getAmpPose();
    // var currentPose = drivetrain.getCurrentPose();
    // System.out.println("ampPose: " + ampPose + ", currentPose: " + currentPose);
    // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    // currentPose,
    // ampPose);

    // System.out.println("*****************************");
    // for (Translation2d translation2d : bezierPoints) {
    // System.out.println("point: " + translation2d);
    // }
    // System.out.println("*****************************");

    // // Create the path using the bezier points created above
    // PathPlannerPath path = new PathPlannerPath(
    // bezierPoints,
    // new PathConstraints(0.5, 0.5, 2 * Math.PI, 4 * Math.PI), // The constraints
    // for this path. If using a differential drivetrain, the angular constraints
    // have no effect.
    // new GoalEndState(0.0, Rotation2d.fromDegrees(-90))); // Goal end state. You
    // can set a holonomic rotation here. If using a differential drivetrain, the
    // rotation will have no effect.

    // // Prevent the path from being flipped if the coordinates are already correct
    // path.preventFlipping = true;
    // CommandScheduler.getInstance().schedule(drivetrain.getFollowPathCommand(path,
    // true));
    // }));
    // if (Utils.isSimulation()) {
    // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
    // Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
    // forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
    // forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  }

  public RobotContainer() {
    NamedCommands.registerCommand("shootSpeakerFront", shootFrontCommand);
    NamedCommands.registerCommand("shootSpeakerSide", shootSideCommand);

    configureBindings();
    buttonBoardSubsystem.configureButtonBindings();

    ShuffleboardTab sbTab = Shuffleboard.getTab("Driver");

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    sbTab.add("Auto Mode", autoChooser)
      .withSize(2, 1)
      .withPosition(6, 0);

    autoChooser.addOption("Shoot Front", shootFrontCommand);
    autoChooser.addOption("Shoot Side", shootSideCommand);

    // FIXME get the right video URL here
    sbTab.addCamera("Driver Camera", "test", "mjpg:http://10.22.28.11:1181/?action=stream4")
      .withProperties(Map.of("showControls", false))
      .withPosition(0, 0)
      .withSize(5, 5);

      driverCameraTilt = 0.3;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void periodic() {
    if (!buttonBoardSubsystem.isDriveOperationMode()) {
      if (buttonBoardSubsystem.getJoystickY() >= 0.1 && driverCameraTilt > 0) {
        driverCameraTilt -= driverCameraTiltChange;
      } else if (buttonBoardSubsystem.getJoystickY() <= -0.1 && driverCameraTilt < 1) {
        driverCameraTilt += driverCameraTiltChange;
      }
      driverCameraTiltPWM.setPosition(driverCameraTilt);
    }
  }
}
