// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;
import java.util.List;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ports;

public class Shooter extends SubsystemBase {
  private TalonFX objShooter = new TalonFX(MotorIDs.iShooterLeader, "MechCAN");
  private TalonFX objFollowShooter = new TalonFX(MotorIDs.iShooterFollower, "MechCAN");

  private double dControl, dError;
  private double dStatorCurrent, dSupplyCurrent;

  

  private StatusCode objTalonFXStatusCode;
  private StatusSignal objStatusSignal;
  public boolean bShooterSpeed = false;
  StatusSignal objStatSig;

  private double distTest;
  private double dTempRPM;

  private final TalonFX LeaderMotor, FollowerMotor;
  private final List<TalonFX> motors;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private double dkP, dkI, dkD, dkV, dkS;

  private double dTweakRPM = 0.0;

  private static final AngularVelocity kVelocityTolerance = RPM.of(100);

  TalonFXConfiguration objTalonFXConfig;


  Sendable shooterPIDSendable;
  
  /** Creates a new Shooter. */
  public Shooter() {

    shooterPIDSendable = new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder){
          builder.addDoubleProperty("Shooter P", () -> dkP, (dNewKP) -> editShooterP(dNewKP));
          builder.addDoubleProperty("Shooter I", () -> dkI, (dnewKI) -> editShooterI(dnewKI));

          builder.addDoubleProperty("Shooter V", () -> dkV, (dnewKV) -> editShooterV(dnewKV));
          builder.addDoubleProperty("Shooter S", () -> dkS, (dnewKS) -> editShooterS(dnewKS));

      }
    };


    objTalonFXConfig = new TalonFXConfiguration();

    objTalonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
    objTalonFXConfig.Slot0.kP = 0.7763;
    objTalonFXConfig.Slot0.kI = 0.0;
    objTalonFXConfig.Slot0.kV = 0.11455;
    objTalonFXConfig.Slot0.kS = 0.2;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;
    LeaderMotor = new TalonFX(MotorIDs.iShooterLeader, ports.kMechCAN);
    FollowerMotor = new TalonFX(MotorIDs.iShooterFollower, ports.kMechCAN);
    motors = List.of(LeaderMotor, FollowerMotor);

    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objShooter.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }

     for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objFollowShooter.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
     }

  
    objFollowShooter.setControl(new Follower(objShooter.getDeviceID(), MotorAlignmentValue.Opposed));


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Sub/Shooter Speed RPM", getSpeedRPM());
    distTest = SmartDashboard.getNumber("Shooter Sub/distance test", 1.0);
    SmartDashboard.putNumber("Shooter Sub/RPM Interp result", getDistance2RPM(distTest));
    dTempRPM = SmartDashboard.getNumber("Shooter Sub/Test RPM", 3500.0);
    SmartDashboard.putNumber("Test RPM", dTempRPM);
    getCurrents();
    SmartDashboard.putNumber("Shooter Supply Current", dSupplyCurrent);
    SmartDashboard.putNumber("Shooter Stator Current", dStatorCurrent);

    SmartDashboard.putData("Shooter Hub/Shooter PID Adjuster", shooterPIDSendable);
    SmartDashboard.putNumber("Tweak RPM", dTweakRPM);
  }

  public BooleanSupplier bsShooterFast(){
    if (getSpeedRPM() > 1000.0) {
      return () -> true;
    }
    
    else return () -> false;
  }
   
  public void stopShooter(){
    objShooter.stopMotor();
  }

  public void runShooter(double dSpeed){
    objShooter.set(dSpeed);

  }

  public void runShooterRPM(double dTargetRPM){
    // dTargetRPM = dTempRPM;

    // dControl = dTargetRPM / 5000.0;
    // dError = dTargetRPM - getSpeedRPM();
    // dControl = dControl + dError * 0.05 / 200.0; // was 0.04
    // objShooter.set(dControl);

    objShooter.setControl(velocityRequest.withVelocity((dTargetRPM + dTweakRPM) / 60.0));
  } 

  public boolean isVelocityWithinTolarance() {
    return motors.stream().allMatch(motor -> {
      final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
      final AngularVelocity currentVelocity = motor.getVelocity().getValue();
      final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
      return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
    });
  }

  public double getDistance2RPM(double distanceToHub){
    return distanceToHub;
  } 

  public double getSpeedRPM() {
    objStatSig = objShooter.getVelocity();
    return objStatSig.getValueAsDouble() * 60.0;  
  }

  public Command shooterRunning(double dSpeed){
    return run(() -> objShooter.set(dSpeed));
  }

  public void getCurrents() {
    dStatorCurrent = objShooter.getStatorCurrent().getValueAsDouble();
    dSupplyCurrent = objShooter.getSupplyCurrent().getValueAsDouble();
  }


  // === Shooter PID+ Adjuster === Help from Hawk \\ 
  public void editShooterP(double dNewKP){
    dkP = dNewKP;

    objTalonFXConfig.Slot0.kP = dkP;

    objShooter.getConfigurator().apply(objTalonFXConfig);
    objFollowShooter.getConfigurator().apply(objTalonFXConfig);
  }

  public void editShooterI(double dNewKI){
    dkI = dNewKI;

    objTalonFXConfig.Slot0.kI = dkI;

    objShooter.getConfigurator().apply(objTalonFXConfig);
    objFollowShooter.getConfigurator().apply(objTalonFXConfig);
  }

  public void editShooterV(double dNewKV){
    dkV = dNewKV;

    objTalonFXConfig.Slot0.kV = dkV;

    objShooter.getConfigurator().apply(objTalonFXConfig);
    objFollowShooter.getConfigurator().apply(objTalonFXConfig);
  }

  public void editShooterS(double dNewKS){
    dkS = dNewKS;

    objTalonFXConfig.Slot0.kS = dkS;

    objShooter.getConfigurator().apply(objTalonFXConfig);
    objFollowShooter.getConfigurator().apply(objTalonFXConfig);
  }



  // === INTERP UPDATER === \\

  public double incrTableVar(double dTableVar){

    dTableVar = dTableVar + 25;

    return dTableVar;
  }

  public double decrTableVar(double dTableVar){

    dTableVar = dTableVar - 5;
    
    return dTableVar;
  }

  public void tweakRPM(double dTweakVal) {
    dTweakRPM = dTweakRPM + dTweakVal;
  }
}
