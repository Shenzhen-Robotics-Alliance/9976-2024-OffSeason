package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Pitch extends SubsystemBase{
    private static final double GEAR_RATIO = 222.2133;
    private static final ArmFeedforward k_feedForward = new ArmFeedforward(0.03, 0.26, 2.99);
    private static final PIDController k_feedBack = new PIDController(7.0/Math.toRadians(26), 0, 0);
    private static final TrapezoidProfile k_profile = new TrapezoidProfile(new Constraints(360, 720));
    private static final double tolerance = Math.toRadians(3);
    /* arm positions, in degrees */
    public static final double LOWEST_POSITION = Math.toRadians(2),
        SHOOTING_POSITION = Math.toRadians(40),
        AMP_POSITION_DEG = Math.toRadians(90);

    private final TalonFX pitchFalcon = new TalonFX(17);
    private final StatusSignal<Double> pitchPositionRevolutions, supplyCurrent;
    private State currentState = new State(LOWEST_POSITION, 0);
    private double setPoint = LOWEST_POSITION;

    public Pitch() {
        this.pitchPositionRevolutions = pitchFalcon.getPosition();
        this.supplyCurrent = pitchFalcon.getSupplyCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(100, supplyCurrent, pitchPositionRevolutions);
        final CurrentLimitsConfigs pitchCurrentLimit = new CurrentLimitsConfigs();
        pitchCurrentLimit.SupplyCurrentLimit = 40;
        pitchCurrentLimit.SupplyCurrentLimitEnable = true;
        pitchFalcon.getConfigurator().apply(pitchCurrentLimit);
        pitchFalcon.setNeutralMode(NeutralModeValue.Coast);
        pitchFalcon.setInverted(false);
        pitchFalcon.optimizeBusUtilization();
        setDefaultCommand(Commands.run(() -> runSetPointProfiled(LOWEST_POSITION), this));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(pitchPositionRevolutions, supplyCurrent);
        SmartDashboard.putNumber("Arm/arm angle measured (deg)", Math.toDegrees(getArmAngleRad()));
        SmartDashboard.putNumber("Arm/arm supply current (A)", supplyCurrent.getValue());
    }

    public double getArmAngleRad() {
        return Units.rotationsToRadians(
                pitchPositionRevolutions.getValue() / GEAR_RATIO
        ) - LOWEST_POSITION;
    }

    public void runSetPointProfiled(double setPoint) {
        this.setPoint = setPoint;
        final State goalStateDeg = new State(setPoint, 0);
        this.currentState = k_profile.calculate(Robot.kDefaultPeriod, currentState, goalStateDeg);

        final double feedForwardVoltage = k_feedForward.calculate(
            getArmAngleRad(),
            currentState.velocity
        );
        final double feedBackVoltage = k_feedBack.calculate(getArmAngleRad(), currentState.position);

        final VoltageOut vOut = new VoltageOut(feedForwardVoltage + feedBackVoltage).withEnableFOC(true);
        SmartDashboard.putNumber("Arm/FeedForwardVoltage", feedForwardVoltage);
        SmartDashboard.putNumber("Arm/FeedBackVoltage", feedBackVoltage);

        pitchFalcon.setControl(vOut);
    }

    public boolean inPosition() {
        return Math.abs(getArmAngleRad() - setPoint) < tolerance;
    }
}
