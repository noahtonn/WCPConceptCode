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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorPorts;
import frc.robot.Constants.pivotConstants;
import frc.robot.Constants.pivotConstants.PivotPosition;

public class Pivot extends SubsystemBase {

  double pivotPosition;
  SparkMax pivotMotor;
  PIDController pivotPID;
  PivotPosition Sposition;
  
  public Pivot() {
    pivotMotor = new SparkMax(motorPorts.pivotMotor, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(pivotConstants.currentLimit);
    config.inverted(true);
    pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    pivotPID = new PIDController(pivotConstants.p, pivotConstants.i, pivotConstants.d);
    switchPivot(PivotPosition.CORALINTAKE);
  }

  public void switchPivot(PivotPosition position){
    Sposition = position;
    switch(position){
      case ALGAE:
        pivotPosition = pivotConstants.algae;
        break;

      case CORALSCORE:
        pivotPosition = pivotConstants.coralscore;
        break;
        
      case CLIMB:
        pivotPosition = pivotConstants.climb;
        break;
      
      case CORALINTAKE:
        pivotPosition = pivotConstants.coralintake;
        break;

      default:
        pivotPosition = pivotConstants.coralintake;
    }
  }

  public PivotPosition getPosition(){
    return Sposition;
  }

  @Override
  public void periodic() {
    pivotMotor.set(pivotPID.calculate(pivotMotor.getAbsoluteEncoder().getPosition(), pivotPosition));
    SmartDashboard.putString("PivotPosition", getPosition().toString());
  }
}
