// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.indexConstants;
import frc.robot.Constants.motorPorts;

public class Index extends SubsystemBase {
  
  SparkMax indexMotor;
  CANrange sensor;

  public Index() {
    indexMotor = new SparkMax(motorPorts.indexMotor, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(indexConstants.currentLimit);
    
    sensor = new CANrange(motorPorts.photoEye);
  }

  public void runMotor(boolean intake){
    double motorSpeed = intake ? indexConstants.indexSpeed : -indexConstants.indexSpeed;
    indexMotor.set(motorSpeed);
  }

  public void stopMotor(){
    indexMotor.set(0);
  }

  public boolean getSensor(){
    return sensor.getDistance().getValueAsDouble() < indexConstants.distanceTolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
