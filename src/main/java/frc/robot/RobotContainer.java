// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAtAMP;
import frc.robot.commands.AimAtSpeaker;
import frc.robot.commands.NoteStucked;
import frc.robot.commands.ShootSequence;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
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
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController viceJoystick = new CommandXboxController(1);//Vice joystick

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

      /* subsystems */
  public static final Arm arm = new Arm(); 
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();

  private void configureBindings() {
   /* drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
   */
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getRightY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getRightX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    /* arm and shooter commands */
    // joystick.b().whileTrue(new AimAtSpeaker(arm, shooter));
    joystick.leftBumper().whileTrue(new AimAtSpeaker(arm, shooter));    //Starting Shooter, Ready to Shoot
    joystick.rightTrigger(0.5).whileTrue(intake.launchNote());

    viceJoystick.leftTrigger(0.5).whileTrue(new AimAtAMP(arm, shooter));
    viceJoystick.rightTrigger(0.5).whileTrue(intake.launchNote());
    
    joystick.y().whileTrue(new NoteStucked(arm, intake));
    
    

    /* intaker commands */
    joystick.leftTrigger(0.5).whileTrue(intake.runIntakeUntilNotePresent(joystick.getHID()));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
  }

  private void configureAutoCommands() {
    NamedCommands.registerCommand("shoot sequence", new ShootSequence(arm, intake, shooter));
    NamedCommands.registerCommand("intake note", intake.runIntakeUntilNotePresent());
  }

  private final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

  public RobotContainer() {
    configureBindings();
    configureAutoCommands();
    drivetrain.configureAuto();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Select Auto", autoChooser);
    SmartDashboard.putData(pdp);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
