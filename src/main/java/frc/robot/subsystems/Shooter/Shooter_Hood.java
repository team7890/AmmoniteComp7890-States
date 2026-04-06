// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class Shooter_Hood extends SubsystemBase {

  private TalonFX objHood = new TalonFX(MotorIDs.iHoodMotor, "MechCAN");
  private CANcoder objHoodEncoder = new CANcoder(MotorIDs.iEncoderHoodID, "MechCAN");
  private StatusCode objTalonFXStatusCode;
  private double dReportedPos;
  /** Creates a new Shooter_Hood. */
  public Shooter_Hood() {
    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;

    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objHood.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Sub/Hood Position", getHoodPosition());
    // This method will be called once per scheduler run
  }


  public Command stopHood(){
    return run(() -> objHood.stopMotor());
  }


  public double getHoodPosition() {
    dReportedPos = objHoodEncoder.getAbsolutePosition().getValueAsDouble();

    if (dReportedPos > 0.5 && dReportedPos <= 1.0) {
      dReportedPos = dReportedPos - 0.4;
    }

    else {
      dReportedPos = dReportedPos + 0.6;
    }
    return dReportedPos;
  }
}
