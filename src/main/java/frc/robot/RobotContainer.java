// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import java.sql.Driver;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.subsystems.apriltags.AprilTagSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.OperatorEvent;

public class RobotContainer {
  private double MaxSpeed = 4.4; // 6 meters per second desired top speed
  private double MaxAngularRate = Math.PI*2; // Half a rotation per second max angular velocity
  private Servo driverCameraTiltPWM = new Servo(Constants.kDriverCameraPWMID);
  private double driverCameraTilt;
  private final double driverCameraTiltChange = 0.01;


  private final LoggedDashboardChooser<String> m_auto_chooser = new LoggedDashboardChooser<>("SelectedAuto");
  private static final String kAutoNone = "None";
  private static final String kAutoLeave = "DriveForward_2m";
  private static final String kAutoShoot_F = "Shoot_F";
  private static final String kAutoShoot_S = "Shoot_S";
  private static final String kAutoPos1_1_4 = "Pos1_1_4";
  private static final String kAutoPos2_2_1 = "Pos2_2_1";
  private static final String kAutoPos2_2_3 = "Pos2_2_3";
  private static final String kAutoPos2_2_5 = "Pos2_2_5";
  private static final String kAutoPos2_2_3_1 = "Pos2_2_3_1";
  private static final String kAutoPos3_3 = "Pos3_3";
  private static final String kAutoPos3_3_8 = "Pos3_3_8";
  private static final String kAutoPos3_8 = "Pos3_8";
  

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  public final DrivebaseSubsystem drivetrain = DrivebaseSubsystem.getInstance();
  public final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public final AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();
  public final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
  public final ButtonBoard buttonBoardSubsystem = ButtonBoard.getInstance();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 3% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private boolean aimSpeaker = false;
  private PhoenixPIDController m_thetaController;

  private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                              // driving in open loop

  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
    drivetrain.setDefaultCommand(
        (drivetrain.applyRequest(() -> {
          if (!joystick.getHID().getRightBumper()) {
            return drive.withVelocityX(invertForColor() * joystick.getLeftY() * MaxSpeed) 
                  .withVelocityY(invertForColor() * joystick.getLeftX() * MaxSpeed) 
                  .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
          } else {
            Pose2d target;
            Pose2d currentPose = drivetrain.getCurrentPose();
            if (DriverStation.getAlliance().get() == Alliance.Red) {
              target = aprilTagSubsystem.getTargetPosition2d(4);
            } else {
              target = aprilTagSubsystem.getTargetPosition2d(7);
            }

            Translation2d errorToTarget = currentPose.getTranslation().minus(target.getTranslation());
            // Logger.recordOutput("ErrorToTarget/", errorToTarget);
            // Logger.recordOutput("TargetPosition/", target);
            return driveAngle
              .withVelocityX(invertForColor() * joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                // negative Y (forward)
              .withVelocityY(invertForColor() * joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
              .withTargetDirection(errorToTarget.getAngle()); // Drive counterclockwise with negative X (left)
          }
        }
        ) 
      .ignoringDisable(true))
    );

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> {
      drivetrain.seedFieldRelative(new Pose2d());
    }));

    joystick.a().onTrue(new InstantCommand(() -> {
      shooter.forceLoaded();
    }));
    joystick.y().onTrue(new InstantCommand(() -> {
      shooter.forceEmpty();
    }));
    joystick.start().onTrue(new InstantCommand(() -> {
      System.out.println("Prep Trap Button Pressed");
      ShooterSubsystem.getInstance().operatorEvent(OperatorEvent.PREP_TRAP);
    }));
  }

  public RobotContainer() {
    NamedCommands.registerCommand("shootSpeakerFront", shootFrontCommand);
    NamedCommands.registerCommand("shootSpeakerSide", shootSideCommand);

    m_thetaController = new PhoenixPIDController(15.0, 0.0, 0.0);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveAngle.HeadingController = m_thetaController;

    configureBindings();
    buttonBoardSubsystem.configureButtonBindings();

    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

    m_auto_chooser.addDefaultOption("None", kAutoNone);
    m_auto_chooser.addOption("Leave", kAutoLeave);
    m_auto_chooser.addOption("Shoot_F", kAutoShoot_F);
    m_auto_chooser.addOption("Shoot_S", kAutoShoot_S);
    m_auto_chooser.addOption("Pos1_1_4", kAutoPos1_1_4);
    m_auto_chooser.addOption("Pos2_2_1", kAutoPos2_2_1);
    m_auto_chooser.addOption("Pos2_2_3", kAutoPos2_2_3);
    m_auto_chooser.addOption("Pos2_2_5", kAutoPos2_2_5);
    m_auto_chooser.addOption("Pos2_2_3_1", kAutoPos2_2_3_1);
    m_auto_chooser.addOption("Pos3_3", kAutoPos3_3);
    m_auto_chooser.addOption("Pos3_8", kAutoPos3_8);
    m_auto_chooser.addOption("Pos3_3_8", kAutoPos3_3_8);
    driverTab.add("Auto Chooser", m_auto_chooser.getSendableChooser())
      .withPosition(6, 0)
      .withSize(2,1);

    driverTab.addCamera("Driver Camera", "Driver Camera", "mjpg:http://10.22.28.11:1182/?action=stream")
      .withProperties(Map.of("showControls", false))
      .withPosition(0, 0)
      .withSize(5, 5);

      driverCameraTilt = 0.3;
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }

  private String lastAutoChoice;
  private Command autoCommand = null;
  public void disabledPeriodic(){
    if(m_auto_chooser.get() != lastAutoChoice){
      lastAutoChoice = m_auto_chooser.get();
      System.out.println("Building Auto: " + lastAutoChoice);
      autoCommand = AutoBuilder.buildAuto(lastAutoChoice);
    }
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
