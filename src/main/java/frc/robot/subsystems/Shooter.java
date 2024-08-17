// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterFalcon1 = new TalonFX(-1), shooterFalcon2 = new TalonFX(-1);
  /** Creates a new Shooter. */
  public Shooter() {
        shooterFalcon1.setNeutralMode(NeutralModeValue.Coast);
        shooterFalcon1.setInverted(false);

        shooterFalcon2.setNeutralMode(NeutralModeValue.Coast);
        shooterFalcon2.setInverted(false);
        setDefaultCommand(Commands.run(this::runIdle, this));
  }

  public void runIdle() {
    shooterFalcon1.set(0);
    shooterFalcon2.set(0);
  }

  public void runSpeakerShot() {
    shooterFalcon1.set(0.7);
    shooterFalcon2.set(0.7);
  }

  public void runAmpShot() {
    shooterFalcon1.set(0.25);
    shooterFalcon2.set(0.35);
  }
}
