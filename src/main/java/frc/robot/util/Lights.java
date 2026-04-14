// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LYNKutil.Shift;

public class Lights extends SubsystemBase {

  private static final int iPort = 9;
  private static final int iLength = 45;

  private static final Distance distLightSpacing = Meters.of(2.299 / 45.0);

  private final AddressableLED objLights;
  private final AddressableLEDBuffer objBuffer;
  private Shift objShift;

  private Color stationColor;
  
  /** Creates a new Lights. */
  public Lights() {
    objLights = new AddressableLED(iPort);
    objBuffer = new AddressableLEDBuffer(iLength);

    objLights.setLength(iLength);
    objLights.start();

    


    setDefaultCommand(runPattern(LEDPattern.solid(setStationColor()).atBrightness(Percent.of(25))));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    objLights.setData(objBuffer);
  }

  public Command runPattern(LEDPattern pattern){
    return run(() -> pattern.applyTo(objBuffer));
  }

  public Command matchTimeProgressBar(){

    LEDPattern base = LEDPattern.solid(Color.kGreen);
    LEDPattern mask = LEDPattern.progressMaskLayer(() -> DriverStation.getMatchTime());

    LEDPattern pattern = base.mask(mask);

    return run(() ->
      pattern.applyTo(objBuffer)
    );
  }

  

  public Color setStationColor() {
    if (DriverStation.getAlliance().get() != null)
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          stationColor = Color.kFirstBlue;
        }
        else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
          stationColor = Color.kFirstRed;
        }
      else stationColor = Color.kGreen;

      return stationColor;
  }

  public Command shootingLights(){
    // LEDPattern pattern = LEDPattern.gradient(GradientType.kContinuous, Color.kGreen, Color.kLightYellow)
    LEDPattern pattern = LEDPattern.gradient(GradientType.kContinuous, setStationColor(), Color.kGreen)
      .scrollAtAbsoluteSpeed(MetersPerSecond.of(1.5), distLightSpacing)
      .atBrightness(Percent.of(25));

    return run(() ->
      pattern.applyTo(objBuffer)
    );
  }

  public Command intakeLights(){
    // LEDPattern pattern = LEDPattern.gradient(GradientType.kContinuous, Color.kGreen, Color.kLightYellow)
    LEDPattern pattern = LEDPattern.gradient(GradientType.kContinuous, setStationColor(), Color.kGreen)
      .scrollAtAbsoluteSpeed(MetersPerSecond.of(2), distLightSpacing)
      .atBrightness(Percent.of(25));

    return run(() ->
      pattern.applyTo(objBuffer)
    );
  }






  // public Command shiftProgress(){
    
  //   LEDPattern pattern;
    
  //   LEDPattern shiftPattern = LEDPattern.solid(Color.kBlack);

  //   // === AUTON PHASE === \\
  //   if (objShift == Shift.AUTO) {
  //     shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kBlack).
  //     scrollAtAbsoluteSpeed(MetersPerSecond.of(1), distLightSpacing);
  //   }

  //   // === TRANSITION PHASE === \\
  //   else if (objShift == Shift.TRANSITION) {
  //     if (Shift.wonAuto().get()){
  //       shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kWhiteSmoke).
  //       scrollAtAbsoluteSpeed(MetersPerSecond.of(-1.5), distLightSpacing);
  //     }
  //     else if (Shift.wonAuto().get() == false) {
  //       shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kBlack).
  //         scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), distLightSpacing);
  //     }
  //   }

  //   // === SHIFT 1 === \\
  //   else if (objShift == Shift.SHIFT_1) {
  //     if (Shift.wonAuto().get() == false){
  //       shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kWhiteSmoke).
  //       scrollAtAbsoluteSpeed(MetersPerSecond.of(1.5), distLightSpacing);
  //     }
  //     else if (Shift.wonAuto().get() == true) {
  //       shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kBlack).
  //         scrollAtAbsoluteSpeed(MetersPerSecond.of(0.05), distLightSpacing);
  //     }
  //   }

  //   else if (objShift == Shift.SHIFT_2) {
  //     if (Shift.wonAuto().get() == true){
  //       shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kWhiteSmoke).
  //       scrollAtAbsoluteSpeed(MetersPerSecond.of(1.5), distLightSpacing);
  //     }
  //     else if (Shift.wonAuto().get() == false) {
  //       shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kBlack).
  //         scrollAtAbsoluteSpeed(MetersPerSecond.of(0.05), distLightSpacing);
  //     }
  //   }

  //   else if (objShift == Shift.SHIFT_3) {
  //     if (Shift.wonAuto().get() == false){
  //       shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kWhiteSmoke).
  //       scrollAtAbsoluteSpeed(MetersPerSecond.of(1.5), distLightSpacing);
  //     }
  //     else if (Shift.wonAuto().get() == true) {
  //       shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kBlack).
  //         scrollAtAbsoluteSpeed(MetersPerSecond.of(0.05), distLightSpacing);
  //     }
  //   }

  //   else if (objShift == Shift.SHIFT_4) {
  //     if (Shift.wonAuto().get() == true){
  //       shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kWhiteSmoke).
  //       scrollAtAbsoluteSpeed(MetersPerSecond.of(1.5), distLightSpacing);
  //     }
  //     else if (Shift.wonAuto().get() == false) {
  //       shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kBlack).
  //         scrollAtAbsoluteSpeed(MetersPerSecond.of(0.05), distLightSpacing);
  //     }
  //   }

  //   else if (objShift == Shift.END_GAME){
  //     shiftPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, stationColor).
  //       scrollAtAbsoluteSpeed(MetersPerSecond.of(1.0), distLightSpacing);
  //   }

    
      
  //   pattern = shiftPattern;

  //   return run(() ->
  //     pattern.applyTo(objBuffer)
  //   );
  // }
}
