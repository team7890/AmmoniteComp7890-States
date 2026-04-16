// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class Intake extends SubsystemBase {

  private TalonFX objIntake = new TalonFX(MotorIDs.iIntake, "MechCAN");
  private StatusCode objTalonFXStatusCode;
  private StatusSignal objStatusSignal;

  public double dStatorCurrent, dSupplyCurrent;

  
  /** Creates a new Intake. */
  public Intake() {

    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 100.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;

    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objIntake.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getCurrents();
    SmartDashboard.putNumber("Hopper Sub/Intake Speed", getSpeed());
    SmartDashboard.putNumber("Hopper Sub/Intake Supply", dSupplyCurrent);
  }

   public void stopIntake(){
    objIntake.stopMotor();
  }

  public void runIntake(double dSpeed){
    objIntake.set(dSpeed);
  }


  public double getSpeed() {
    objStatusSignal = objIntake.getVelocity();
    return objStatusSignal.getValueAsDouble() * 60.0;  
  }

  public void getCurrents() {
    dStatorCurrent = objIntake.getStatorCurrent().getValueAsDouble();
    dSupplyCurrent = objIntake.getSupplyCurrent().getValueAsDouble();
  }

}
