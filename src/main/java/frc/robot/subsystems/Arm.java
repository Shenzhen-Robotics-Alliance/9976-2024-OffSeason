package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Arm extends SubsystemBase{
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    private final TalonFX armMotor = new TalonFX(10);

    private final ArmFeedforward k_armFeedForward_Rad = new ArmFeedforward(0.05, 0.15, 7.73);
    private final PIDController k_armFeedBack = new PIDController(7.0/35.0, 0, 0);
    private final TrapezoidProfile k_armProfile = new TrapezoidProfile(new Constraints(120, 240));
    private final Rotation2d k_armEncoderOffSet = Rotation2d.fromDegrees(250);
    private final boolean k_armEncoderInverted = true;

    /* arm positions, in degrees */
    public static final double INTAKE_POSITION_DEG = -12.8,
        SCORE_POSITION_DEG = 0,
        NOTE_STUCK_DEG = 10,
        AMP_POSITION_DEG = 90;

    private State currentStateDeg = new State(INTAKE_POSITION_DEG, 0);
    private double setPointDeg = INTAKE_POSITION_DEG;
    public Arm() {
        armMotor.setNeutralMode(NeutralModeValue.Coast);
        armMotor.setInverted(true);

        setDefaultCommand(Commands.run(() -> runSetPointProfiled(INTAKE_POSITION_DEG), this));
    }

    @Override
    public void periodic() {
        // if (Math.abs(xboxController.getRightY()) > 0)
        //     armMotor.set(-xboxController.getRightY());
        SmartDashboard.putNumber("Arm/arm angle (deg)", getArmAngleDeg());
    }

    public double getArmAngleDeg() {
        return Rotation2d.fromRotations(armEncoder.get()).minus(k_armEncoderOffSet).times(k_armEncoderInverted ? -1:1).getDegrees();
    }

    public void runSetPointProfiled(double setPointDegrees) {
        this.setPointDeg = setPointDegrees;
        final State goalStateDeg = new State(setPointDegrees, 0);
        this.currentStateDeg = k_armProfile.calculate(Robot.kDefaultPeriod, currentStateDeg, goalStateDeg);

        final double ffVoltage = k_armFeedForward_Rad.calculate(
            Math.toRadians(getArmAngleDeg()), 
            Math.toRadians(currentStateDeg.velocity)
        );
        final double fbVoltage = k_armFeedBack.calculate(getArmAngleDeg(), currentStateDeg.position);

        final VoltageOut vOut = new VoltageOut(ffVoltage + fbVoltage).withEnableFOC(true);
        SmartDashboard.putNumber("Arm/Correction Effort (volts)", ffVoltage + fbVoltage);
        SmartDashboard.putNumber("Arm/FeedBackVolts", fbVoltage);

        armMotor.setControl(vOut);
    }

    private static final double ARM_TOLERANCE_DEG = 5;
    public boolean inPosition() {
        return Math.abs(getArmAngleDeg() - setPointDeg) < 5; 
    }
}
