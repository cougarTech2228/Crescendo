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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ButtonBoardSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.OperatorEvent;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
  private double MaxSpeed = 4.4; // 6 meters per second desired top speed
  private double MaxAngularRate = Math.PI*2; // Half a rotation per second max angular velocity
  private Servo driverCameraTiltPWM = new Servo(Constants.kDriverCameraPWMID);
  private double driverCameraTilt;
  private final double driverCameraTiltChange = 0.01;


  private final SendableChooser<String> m_auto_chooser = new SendableChooser<>();
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
  public final ButtonBoardSubsystem buttonBoardSubsystem = ButtonBoardSubsystem.getInstance();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 3% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Path follower */
  private final Telemetry logger = new Telemetry(MaxSpeed);
  //private final SendableChooser<Command> autoChooser;

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

    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public RobotContainer() {
    NamedCommands.registerCommand("shootSpeakerFront", shootFrontCommand);
    NamedCommands.registerCommand("shootSpeakerSide", shootSideCommand);

    configureBindings();
    buttonBoardSubsystem.configureButtonBindings();

    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

    m_auto_chooser.setDefaultOption("None", kAutoNone);
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
    driverTab.add("Auto Chooser", m_auto_chooser)
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
    if(m_auto_chooser.getSelected() != lastAutoChoice){
      lastAutoChoice = m_auto_chooser.getSelected();
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
