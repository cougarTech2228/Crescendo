// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private double MaxSpeed = 4; // 6 meters per second desired top speed
  private double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final DrivebaseSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem(drivetrain);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final SendableChooser<Command> autoChooser;

  Command shootCommand = new SequentialCommandGroup(
      new InstantCommand(() -> shooter.startFlywheel()),
      new WaitCommand(1),
      new InstantCommand(() -> shooter.feedNote()),
      new WaitCommand(1),
      new InstantCommand(() -> shooter.stopMotors())
    );
  Command loadNote = new SequentialCommandGroup(
      new InstantCommand(() -> shooter.loadNote()),
      new WaitCommand(1)
    );

  private int invertForColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return 1;
    }
    return -1;
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(invertForColor() * joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(invertForColor() * joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> {
      var rotatedPose = drivetrain.getCurrentPose().transformBy(new Transform2d(0,0, new Rotation2d(180)));
       drivetrain.seedFieldRelative(rotatedPose);
    }));

    joystick.y().onTrue(shootCommand);

    joystick.x().onTrue(new InstantCommand(() -> shooter.loadNote()));
    joystick.x().onFalse(new InstantCommand(() -> shooter.stopMotors()));
  
    joystick.rightBumper().onTrue(new InstantCommand(() -> {
      var ampPose = aprilTagSubsystem.getAmpPose();
      var currentPose = drivetrain.getCurrentPose();
      System.out.println("ampPose: " + ampPose + ", currentPose: " + currentPose);
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        currentPose,
        ampPose);

      System.out.println("*****************************");
      for (Translation2d translation2d : bezierPoints) {
        System.out.println("point: " + translation2d);
      }
      System.out.println("*****************************");

      // Create the path using the bezier points created above
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(0.5, 0.5, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90))); // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      
      // Prevent the path from being flipped if the coordinates are already correct
      path.preventFlipping = true;
      CommandScheduler.getInstance().schedule(drivetrain.getFollowPathCommand(path, true));
    }));
    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  }

  public RobotContainer() {
    NamedCommands.registerCommand("shootSpeaker", shootCommand);
    NamedCommands.registerCommand("loadNote", loadNote);
    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
