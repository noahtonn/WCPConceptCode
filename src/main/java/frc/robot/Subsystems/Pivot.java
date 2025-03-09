// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorPorts;
import frc.robot.Constants.pivotConstants;
import frc.robot.Constants.pivotConstants.PivotPosition;

public class Pivot extends SubsystemBase {
  SparkMax pivotMotor;
  PIDController pivotPID;
  PivotPosition position;
  SparkClosedLoopController pivotController;
  
  public Pivot() {
    pivotMotor = new SparkMax(motorPorts.pivotMotor, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(pivotConstants.currentLimit);
    config.inverted(true);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(pivotConstants.p, pivotConstants.i, pivotConstants.d)
      .velocityFF(0.001)
      .outputRange(-1, 1);
    
    pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    pivotController = pivotMotor.getClosedLoopController();
    switchPivot(PivotPosition.CORALINTAKE);
  }

  public void switchPivot(PivotPosition position){
    this.position = position;
    switch(position){
      case ALGAE:
        pivotController.setReference(pivotConstants.algae, ControlType.kPosition);
        break;

      case CORALSCORE:
      pivotController.setReference(pivotConstants.coralscore, ControlType.kPosition);
        break;
        
      case CLIMB:
      pivotController.setReference(pivotConstants.climb, ControlType.kPosition);
        break;
      
      case CORALINTAKE:
      pivotController.setReference(pivotConstants.coralintake, ControlType.kPosition);
        break;

      default:
      pivotController.setReference(pivotConstants.coralintake, ControlType.kPosition);
    }
  }

  public PivotPosition getPosition(){
    return position;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Pivot Position", position.toString());
    SmartDashboard.putNumber("Pivot Applied Power", pivotMotor.getAppliedOutput());
  }
}
