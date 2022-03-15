// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  WPI_TalonFX m_midmotor = new WPI_TalonFX(51);
  WPI_TalonFX m_midmotor2 = new WPI_TalonFX(52);
  WPI_TalonFX m_pitch = new WPI_TalonFX(53);

  DoubleSolenoid midpistonlift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid midpistongrab = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  DoubleSolenoid traversalpistonlift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
  DoubleSolenoid traversalpistongrab = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  DigitalInput midclawengaged = new DigitalInput(0);
  DigitalInput traversalclawengaged = new DigitalInput(1);

  


  /** Creates a new Climber. */
  public Climber() {
    compressor.enableDigital();
  }

  public void midLift() {
    midpistonlift.set(Value.kForward);
  }

  public void midGrab() {
    midpistongrab.set(Value.kForward);
  }

  public void traverseLift() {
    traversalpistonlift.set(Value.kForward);
  }

  public void traverseGrab() {
    traversalpistongrab.set(Value.kForward);
  }

  public boolean returnMidClawEngaged() {
    return midclawengaged.get();
  }

  public boolean returnTraverseClawEngaged() {
    return traversalclawengaged.get();
  }

  public void PositionalPID() {

  }

  public void pitch() {

  }

  public void lockMidWinch() {
    midpistonlift.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
