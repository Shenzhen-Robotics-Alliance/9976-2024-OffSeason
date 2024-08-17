// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private static final double k_driveSpeedLimitPercent = 1;
  private static final double k_rotationalSpeedLimitPercent = 1;
  private final SendableChooser<Command> autoChooser;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * k_driveSpeedLimitPercent; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 2 * Math.PI * k_rotationalSpeedLimitPercent; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController pilotJoystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController copilotJoystick = new CommandXboxController(1);//Vice joystick

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /* subsystems */
  public static final Pitch pitch = new Pitch();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-pilotJoystick.getRightY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-pilotJoystick.getRightX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-pilotJoystick.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    pilotJoystick.x().whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    pilotJoystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    /* intake commands */
    pilotJoystick.leftTrigger(0.5).whileTrue(intake.runIntakeUntilNotePresent(pilotJoystick.getHID()));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
  }

  private void configureAutoCommands() {

  }

  private final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

  public RobotContainer() {
    configureBindings();
    configureAutoCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Select Auto", autoChooser);
    SmartDashboard.putData(pdp);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
