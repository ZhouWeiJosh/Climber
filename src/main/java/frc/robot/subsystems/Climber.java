// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Climber extends SubsystemBase {
  TalonFX m_midmotor = new TalonFX(51);
  TalonFX m_midmotor2 = new TalonFX(52);
  TalonFX m_pitch = new TalonFX(53);
  int counter = 0;
  boolean checkcounter = false; 
  boolean check = false;
  

  DoubleSolenoid midpistonlift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid midpistongrab = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  DoubleSolenoid traversalpistonlift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
  DoubleSolenoid traversalpistongrab = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  DigitalInput midclawengaged = new DigitalInput(1);
  DigitalInput traversalclawengaged = new DigitalInput(2);

  /** Creates a new Climber. */
  public Climber() {
    compressor.enableDigital();

    m_midmotor.configFactoryDefault();
    m_midmotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_midmotor.setNeutralMode(NeutralMode.Brake);

    SmartDashboard.putNumber("Position With Ticks", 0);
  }

  public void midWinchPiston() {
   if(counter == 0) { 
    midpistonlift.set(Value.kForward);
   } else if(counter == 1) {
    checkcounter = true;
  }
  }

  public void midClawGrab() {
    if(counter == 0) {
    midpistongrab.set(Value.kForward);
    counter++;
    }
    // } else if(counter == 1) {
    //   checkcounter = true;
    // }
  }

  public boolean getCount() {
    return counter >= 0;
  }

  public boolean getCheckCountTrue() {
    return checkcounter;
  }

  public boolean getCheckCountFalse() {
    return !checkcounter;
  }

  public void traverseWinchPiston() {
    if(counter == 4) {
    traversalpistonlift.set(Value.kForward);
    }
  }

  public void traverseClawGrab() {
    if(counter == 4) {
    traversalpistongrab.set(Value.kForward);
    counter++;
    } else if(counter == 5 && !checkcounter) {
      checkcounter = true; 
    }
  }

  public boolean getMidClawEngaged() {
    return midclawengaged.get();
  }

  public boolean getTraverseClawEngaged() {
    return  traversalclawengaged.get();
  }

  public void midPull() {
    double postionTicks = SmartDashboard.getNumber("Motor Position With Ticks", 0);
    m_midmotor.set(ControlMode.Position, postionTicks);

    SmartDashboard.putNumber("encoder value per tick", m_midmotor.getSelectedSensorPosition());
    if (counter == 2) {
      m_midmotor.set(ControlMode.Position, 0);
      counter++;
    } else if(counter == 3 && !checkcounter) {
      checkcounter = true;
    }
  }

  public void pitch() {
    m_pitch.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    double ticks = m_pitch.getSelectedSensorPosition();
    double angle = 0;
    double degtotick = angle*2048/360;
    double kP = SmartDashboard.getNumber("kP", 0);
    double kI = SmartDashboard.getNumber("kI", 0);
    double kD = SmartDashboard.getNumber("kD", 0);
    double ticktodeg = ticks*360/2048;

    
    
    //double output = kP*encoderVal;
    SmartDashboard.putNumber("MotorSpeed", degtotick);

    

    
    if (counter == 3) {
      counter++;
      m_pitch.set(ControlMode.Position, degtotick);
    } else if(counter == 4 && checkcounter) {
      checkcounter = false;
    }
  }

  public void lockMidWinch() {
    if(counter == 1) {
    midpistonlift.set(Value.kReverse);
    counter++;
    } else if(counter == 2 && checkcounter) {
      checkcounter = false; 
    }
  }

  public void releaseMidGrab() {
    midpistongrab.set(Value.kReverse);
    counter = 0;
  }

  public void setZero() {
    counter = 0;
  }



  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Counter-Boolean", checkcounter);
    SmartDashboard.putNumber("Counter", counter);

    // This method will be called once per scheduler run
  }
}
