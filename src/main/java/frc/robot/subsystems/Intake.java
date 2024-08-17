package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Intake extends SubsystemBase {
    private final TalonFX lowerIntakeFalcon = new TalonFX(9), upperIntakeFalcon = new TalonFX(11);
    private final DigitalInput beamBreaker = new DigitalInput(2);

    public Intake() {
        // super.setDefaultCommand();
        lowerIntakeFalcon.setNeutralMode(NeutralModeValue.Coast);
        lowerIntakeFalcon.setInverted(true);

        upperIntakeFalcon.setNeutralMode(NeutralModeValue.Coast);
        upperIntakeFalcon.setInverted(true);

        setDefaultCommand(Commands.run(this::runIdle, this));
    }

    public void runIdle() {
        lowerIntakeFalcon.set(0);
        upperIntakeFalcon.set(0);
    }

    public void runIntake() {
        lowerIntakeFalcon.set(0.3);
        upperIntakeFalcon.set(0.3);
    }

    //Shooter out the Node when note is stucked in.
    public void runReverse(){
        lowerIntakeFalcon.set(-0.3);
        upperIntakeFalcon.set(-0.3);
    }
    public void runShoot() {
        lowerIntakeFalcon.set(0);
        upperIntakeFalcon.set(0.3);
    }

    public Command runIntakeUntilNotePresent() {
        return Commands.run(this::runIntake, this)
            .onlyIf(() -> !this.hasNote())
            .until(this::hasNote)
            .finallyDo(this::runIdle);
    }

    public Command runIntakeUntilNotePresent(XboxController hid) {
        final SequentialCommandGroup group = new SequentialCommandGroup();

        group.addCommands(runIntakeUntilNotePresent());
        group.addCommands(Commands.run(() -> hid.setRumble(RumbleType.kBothRumble, 1)).withTimeout(0.5));
        return group.finallyDo(() -> hid.setRumble(RumbleType.kBothRumble, 0));
    }

    public Command launchNote() {
        return Commands.run(this::runShoot, this)
            .onlyIf(this::hasNote)
            .until(()-> !this.hasNote());
    }

    //  //Shooter out the Node when note is stucked in.
    // public Command outNote() {
    //     return Commands.run(this::outIntake, this)
    //         .onlyIf(this::hasNote)
    //         .until(()-> !this.hasNote());
    // }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("BeamBreak value", beamBreaker.get());
    }

    public boolean hasNote() {
        return !beamBreaker.get();
    }
}
