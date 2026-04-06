// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }

  public static class ports{
    public static final CANBus kMechCAN = new CANBus("MechCAN");
  }

  public static class MotorIDs {
    public static final int iIntake = 13;
    public static final int iIntakePivot = 14;
    public static final int iIndexerLeader = 15;
    public static final int iIndexerFollower = 21;
    public static final int iFeederLeader = 16;
    public static final int iFeederFollower = 17;
    public static final int iHoodMotor = 18;
    public static final int iShooterLeader = 19;
    public static final int iShooterFollower = 20;
    public static final int iEncoderPivotID = 22;
    public static final int iEncoderHoodID = 23;

  }

  public static class MotorSpeeds{

    ///Shooter RPM values///
    public static final double dShooter3M = 3000;
    public static final double dShooterRPM = 3600;
    public static final double dPassingRPM = 6000;

    ///TEST RPM value///
    public static final double dtestRPM = 4550;

    ///Hopper system speeds///
    public static final double dFeederSpeed = 1.0; 
    public static final double dIndexerSpeed = 1.0;

    ///Intake system speeds///
    public static final double dIntakeSpeed = 0.375; //Started at 0.4
    public static final double dPivotSpeed = -0.2; //change back to 0.25
    public static final double dPivSlow = -0.15;

  }

  public static class Shooting {

    /// INTERPOLATION TABLE VALUES ///      /// 1 relates to 1, 2 relates to 2, and so on ///

    /// If you want more values you have to add a key and a RPM ///

    /// Distance in meters ///              //////ADDD THE REST OF THE NUMBERS IN THE TABLE/////!!!!!!!
    public static final double dKey1 = 2.0;
    public static final double dKey2 = 2.5;
    public static final double dKey3 = 3.0;
    public static final double dkey4 = 3.5;
    public static final double dkey5 = 4.0;
    public static final double dKey6 = 4.5;
    public static final double dKey7 = 5.0;

    public static final double dRPM1 = 3200;
    public static final double dRPM2 = 3550;
    public static final double dRPM3 = 3900;
    public static final double dRPM4 = 4175;
    public static final double dRPM5 = 4550;
    public static final double dRPM6 = 4750;
    public static final double dRPM7 = 5000;
  }

  public static class MotorPositions{
   // public static final Pos dshooterHood = 
    public static final double dPivotMax = 90;
    public static final double dPivotMin = 0;


  }

  /// PHOTON STUFF ///
   
// public static class Vision {
//   public static final String sCameraName = "Dragon";
//   public static final Transform3d kRobotToCam = 
//         new Transform3d(new Translation3d(0.1397, 0.2921, 0.4953), new Rotation3d(0.0, 0.35, 0.443));
// public static final AprilTagFieldLayout kTagLayout = 
//       AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
// public static final Matrix<N3, N1> kSingleTagStdDevs = 
//       VecBuilder.fill(4.0, 4.0, 8.0);
// public static final Matrix<N3, N1> kMultiTagStdDevs = 
//       VecBuilder.fill(0.5, 0.5, 1.0);

//    }
  
  
  //  public class Landmarks {
  //   public static Translation2d hubPosition() {
  //     final Optional<Alliance> alliance = DriverStation.getAlliance();
  //     if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
  //       return new Translation2d(Inches.of(469.115), Inches.of(158.845));
  //     }
  //     return new Translation2d(Inches.of(469.115), Inches.of(158.845));
  //   }
  //  }

   public static class Driving {
    public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
    public static final AngularVelocity kPIDRotaionDeadband = kMaxRotationalRate.times(0.005);
   }









}