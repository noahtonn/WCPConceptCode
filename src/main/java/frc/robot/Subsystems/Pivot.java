// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorPorts;
import frc.robot.Constants.pivotConstants;
import frc.robot.Constants.pivotConstants.PivotPosition;

public class Pivot extends SubsystemBase {

  int pivotPosition;
  SparkMax pivotMotor;
  PIDController pivotPID;
  PivotPosition position;
  
  public Pivot() {
    pivotMotor = new SparkMax(motorPorts.pivotMotor, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(pivotConstants.currentLimit);
    pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    pivotPID = new PIDController(pivotConstants.p, pivotConstants.i, pivotConstants.d);
  }

  public void switchPivot(PivotPosition position){
    this.position = position;
    switch(position){
      case ALGAE:
        pivotPosition = pivotConstants.algae;
        break;

      case CORALSCORE:
        pivotPosition = pivotConstants.coralintake;
        break;

      case CLIMB:
        pivotPosition = pivotConstants.climb;
        break;
      
      case CORALINTAKE:
        pivotPosition = pivotConstants.coralscore;
        break;

      default:
        pivotPosition = pivotConstants.coralintake;
    }
  }

  public PivotPosition getPosition(){
    return position;
  }

  @Override
  public void periodic() {
    pivotMotor.set(pivotPID.calculate(pivotMotor.getAbsoluteEncoder().getPosition(), pivotPosition));
  }
}
