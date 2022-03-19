// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  TalonSRX m_midmotor = new TalonSRX(2);
  TalonSRX m_pitch = new TalonSRX(4);
 // TalonFX m_midmotor = new TalonFX(51);
  TalonFX m_midmotor2 = new TalonFX(52);
 // TalonFX m_pitch = new TalonFX(53);
 
  int counter = 0;
  boolean actionfinished = false;

  DoubleSolenoid midpistonlift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid midpistongrab = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);

  DoubleSolenoid traversalpistonlift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
  DoubleSolenoid traversalpistongrab = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 6);

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  DigitalInput midclawengaged = new DigitalInput(1);
  DigitalInput traversalclawengaged = new DigitalInput(2);

  Timer timer = new Timer();

  double positionTicks = 12288.0;

  /** Creates a new Climber. */
  public Climber() {
    compressor.enableDigital();

    m_midmotor.configFactoryDefault();
    m_midmotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_midmotor.setNeutralMode(NeutralMode.Brake);
    m_midmotor.setSensorPhase(true);

    m_pitch.configFactoryDefault();
    m_pitch.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_pitch.setNeutralMode(NeutralMode.Brake);
    m_pitch.setSensorPhase(true);

    m_midmotor.config_kF(0, 0.0);
    m_midmotor.config_kP(0, 5.0);
    m_midmotor.config_kI(0, 0.00000001);
    m_midmotor.config_kD(0, 100.0);

    m_pitch.config_kF(0, 0);
    m_pitch.config_kP(0, 6.0);
    m_pitch.config_kI(0, 0.01);
    m_pitch.config_kD(0, 300.0);
    m_pitch.config_IntegralZone(0, 50.0);
    m_pitch.configAllowableClosedloopError(0, 10.0);
  }

  public void midWinchPiston() {
    midpistonlift.set(Value.kForward);
  }

  public void midClawGrab() {
    if(midclawengaged.get()) {
      midpistongrab.set(Value.kForward);
      actionfinished = true;
    } else if(timer.hasElapsed(10)) {
      midpistonlift.set(Value.kReverse);
      Timer.delay(2);
      m_midmotor.set(ControlMode.Position, -6144.0);
      if (m_midmotor.getSelectedSensorPosition() >= -6144.0) {
        counter = counter - 1; 
        actionfinished = true;
      }
    }
  }

  public boolean getActionFinished() {
    return actionfinished; 
  }

  public void actionFinishedFalse() {
    actionfinished = false;
  }

  public void resetBooleans() {
    m_midmotor.setSelectedSensorPosition(0);
    m_pitch.setSelectedSensorPosition(0);
    m_midmotor.set(ControlMode.Position, 0.0);
    m_pitch.set(ControlMode.Position, 0.0);
    timer.reset();
    timer.start();
    actionfinished = false; 

  }

  public void climbToTraversal() {
    switch(counter) {
      case 1: 
             midWinchPiston();
             midClawGrab();
             break;
      case 2:
             lockMidWinch();
             break;
      case 3: 
            midPull();
             break;
      case 4: 
            pitch();
             break;
      case 5:
             traverseWinchPiston();
             traverseClawGrab();
             break;
      case 6:
            releaseMidClaw();
            setZero();
            break;
          }
  }

  public void traverseWinchPiston() {
    traversalpistonlift.set(Value.kForward);
  }

  public void traverseClawGrab() {
    if(traversalclawengaged.get()) {
    traversalpistongrab.set(Value.kForward);
    actionfinished = true;
    } else if(timer.hasElapsed(10)) {
      midpistonlift.set(Value.kReverse);
      Timer.delay(2);
      m_midmotor.set(ControlMode.Position, -6144.0);
      if (m_midmotor.getSelectedSensorPosition() >= -6144.0) {
        counter = counter - 1; 
        actionfinished = true;
      }
    }
  }

  public void resetEncoderValues() {
    m_midmotor.setSelectedSensorPosition(0);
    m_pitch.setSelectedSensorPosition(0);
   }

  public void midPull() {
    m_midmotor.set(ControlMode.Position, positionTicks);
    
    if (m_midmotor.getSelectedSensorPosition() >= positionTicks) {
      actionfinished = true;
    } else if(timer.hasElapsed(10)) {
      m_midmotor.set(ControlMode.Position, 0);
      if(m_midmotor.getSelectedSensorPosition() <= 3.0) {
        counter = counter - 1;
        actionfinished = true;
      }
    }
   }

  public void pitch() {
    double degtotick = 3413.0;
    m_pitch.set(ControlMode.Position, degtotick);

    if (m_pitch.getSelectedSensorPosition() >= 3400.0) {
      actionfinished = true;
    } else if(timer.hasElapsed(10)) {
      m_pitch.set(ControlMode.Position, 0);
      if(m_midmotor.getSelectedSensorPosition() <= 3.0) {
        counter = counter - 1;
        actionfinished = true;
      }
    }
  }

  public void lockMidWinch() {
    midpistonlift.set(Value.kReverse);
    actionfinished = true;
  }

  public void releaseMidClaw() {
    midpistongrab.set(Value.kReverse);
    m_midmotor.setSelectedSensorPosition(0);
    m_pitch.setSelectedSensorPosition(0);
    m_midmotor.set(ControlMode.Position, 0);
    m_pitch.set(ControlMode.Position, 0);

  }

  public void setZero() {
    counter = 0;
  }

  public void incrementCount() {
    counter++;
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Counter", counter);
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putBoolean("Action Finished", actionfinished);
    SmartDashboard.putBoolean("Mid Limit Switch", midclawengaged.get());
    SmartDashboard.putBoolean("Travs Limit Switch", traversalclawengaged.get());
    SmartDashboard.putNumber("encoder value per tick", m_midmotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Pitch encoder value", m_pitch.getSelectedSensorPosition());

    // This method will be called once per scheduler run
  }
}
