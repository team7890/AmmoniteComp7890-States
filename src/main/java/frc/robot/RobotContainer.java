// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.ejml.equation.IntegerSequence.Range;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Driving;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.commands.PivotIntake;
import frc.robot.commands.ShooterFull;
import frc.robot.generated.TunerConstants;
// import frc.robot.generated.TunerConstants_comp;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper.Pivot;
import frc.robot.subsystems.Hopper.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter_Hood;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Hopper.Intake;
import frc.robot.commands.AimandShoot2;
import frc.robot.commands.Feed;
import frc.robot.commands.Passing;
import frc.robot.commands.ShooterSolo;
import frc.robot.commands.TrenchShotAuto;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import java.util.HashMap;




public class RobotContainer {
     private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxSpeed = 1.0 * TunerConstants_beta.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Telemetry logger = new Telemetry(Driving.kMaxSpeed.in(MetersPerSecond));

    // === CONTROLLERS === \\
    private final CommandXboxController xboxDriver = new CommandXboxController(0);

    // public final CommandSwerveDrivetrain drivetrain = TunerConstants_beta.createDrivetrain(); 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // comment out the other one for the comp chassi

    // === SUBSYSTEM OBJECTS === \\
    
///added///
    // private final CommandSwerveDrivetrain swerve = new CommandSwerveDrivetrain();

    private final Shooter objShooter = new Shooter();
    private final Shooter_Hood objHood = new Shooter_Hood();
    private final Feeder objFeeder = new Feeder();

    private final Indexer objIndexer = new Indexer();

    private final Intake objIntake = new Intake();
    private final Pivot objPivot = new Pivot();

    /// added ///
    private final Limelight limelight = new Limelight("limelight-genesis");

    private final Field2d field;

    // === PathPlanner === \\
    private final SendableChooser<Command> autoChooser;
 


    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        //autoChooser.addOption("Chaos with Trench","Chaos with trench");
        // Command Chaos = AutoBuilder.buildAuto("Take 5");

        drivetrain.registerTelemetry(logger::telemeterize);

        // autoChooser.addOption("Take 5", Chaos);

        NamedCommands.registerCommand("stop Shooter", getAutonomousCommand());
        NamedCommands.registerCommand("Intake Pivot", getAutonomousCommand());
        NamedCommands.registerCommand("Print Message", getAutonomousCommand());
        NamedCommands.registerCommand("Run Intake", getAutonomousCommand());
        NamedCommands.registerCommand("stop Shooter", new ShooterFull(objShooter, MotorSpeeds.dShooter3M, objFeeder, objIndexer, objIntake, objPivot));
        NamedCommands.registerCommand("Run Intake", new RunCommand(()-> objIntake.runIntake(MaxSpeed)).withTimeout(5.0));
        NamedCommands.registerCommand("Auto shots", new TrenchShotAuto(objShooter, MotorSpeeds.dShooter3M, objFeeder, objIndexer, objIntake, objPivot));
        NamedCommands.registerCommand("Auto Shoot", new TrenchShotAuto(objShooter, MotorSpeeds.dShooter3M, objFeeder, objIndexer, objIntake, objPivot));
        
        new EventTrigger("Run Intake").whileTrue(new RunCommand(()-> objIntake.runIntake(MotorSpeeds.dIntakeSpeed), objIntake));
        //new EventTrigger("Auto Shoot").whileTrue(new RunCommand(()-> objShooter.runShooterRPM(MotorSpeeds.dShooter3M), objShooter));
        new EventTrigger("Intake Pivot").whileTrue(new PivotIntake(objPivot, MotorSpeeds.dPivotSpeed));
        new EventTrigger("Shooter Run").whileTrue(new ShooterFull(objShooter, 2500, objFeeder, objIndexer, objIntake, objPivot));
        new EventTrigger("Shoot Slower").whileTrue(new ShooterFull(objShooter, 3850, objFeeder, objIndexer, objIntake, objPivot));
        new EventTrigger("Second Shot").whileTrue(new ShooterFull(objShooter, 3850, objFeeder, objIndexer, objIntake, objPivot));
        
        configureBindings();

      

        SmartDashboard.putNumber("Util/Match Time", DriverStation.getMatchTime());
        // SmartDashboard.putNumber("null", DriverStation.getAlliance());

  
   

        field = new Field2d();
            SmartDashboard.putData("Util/Field", field);

        
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        limelight.setDefaultCommand(updateVisionCommand());

                                             ///SET DEFAULTCOMANDS ///
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-xboxDriver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xboxDriver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-xboxDriver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


        objFeeder.setDefaultCommand(
            new RunCommand(()-> objFeeder.stopFeeder(), objFeeder)
        );

        objIndexer.setDefaultCommand(
            new RunCommand(()-> objIndexer.stopIndexer(), objIndexer)
        );

        objShooter.setDefaultCommand(
            new RunCommand(()-> objShooter.runShooterRPM(4000.0), objShooter) //Original 0.15
        );

        objPivot.setDefaultCommand(
            new RunCommand(() -> objPivot.stopPivot(), objPivot)
        );

        objIntake.setDefaultCommand(
            new RunCommand(() -> objIntake.stopIntake(), objIntake)
        );

        objHood.setDefaultCommand(
            objHood.stopHood()
        );


        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

              

        // === OFFICIAL CONTROLS === \\

        // == Drive == \\
        xboxDriver.button(8).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        xboxDriver.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.k180deg)));
        // == Reset Forward Direction

        xboxDriver.x().whileTrue(drivetrain.applyRequest(() -> brake)); 
        // == Wheels X for no moving

        // === Shooter == \\
        //xboxDriver.axisGreaterThan(3, 0.25).whileTrue(new ShooterFull(objShooter, MotorSpeeds.dShooterRPM, objFeeder, objIndexer, objIntake, objPivot));

        xboxDriver.axisGreaterThan(3, 0.25).whileTrue(
            new AimandShoot2(  
                limelight, drivetrain, objShooter, objFeeder, objIndexer, objIntake, objPivot,
                () -> xboxDriver.getLeftY(),
                () -> xboxDriver.getLeftX()
            )
        );
        

        ///TEST SHOOT FOR INTERP TABLE///
        xboxDriver. b().whileTrue(new ShooterFull(objShooter, MotorSpeeds.dtestRPM, objFeeder, objIndexer, objIntake, objPivot));



          ///Far shooter///
          /// 
        //xboxDriver. b().whileTrue(new ShooterFull(objShooter, MotorSpeeds.dPassingRPM, objFeeder, objIndexer, objIntake, objPivot));

         ///Passing///
        xboxDriver. leftBumper().whileTrue(new Passing(objShooter, objFeeder, objIndexer, objIntake));

        xboxDriver. y().toggleOnTrue(new RunCommand(()-> objShooter.runShooterRPM(0.0), objShooter)); //Original 0.15


        // === Intake === \\ 
        xboxDriver.axisGreaterThan(2, 0.25).whileTrue(new RunCommand(
                ()-> objIntake.runIntake(MotorSpeeds.dIntakeSpeed), objIntake));

        xboxDriver.a().toggleOnTrue(new PivotIntake(objPivot, MotorSpeeds.dPivotSpeed));

        xboxDriver.povDown().whileTrue(new RunCommand(
                () -> objIntake.runIntake(-MotorSpeeds.dIntakeSpeed), objIntake));

        xboxDriver.povUp().whileTrue(new RunCommand(
                () -> objFeeder.runFeeder(-MotorSpeeds.dFeederSpeed), objFeeder));

       
       drivetrain.registerTelemetry(logger::telemeterize);

       // Logger.recordOutput("Swerve/Pose", getPose());
      
    }

    ///// ASK KRAUSS ABOUT Swerve TO drivetrain //////

    private Command updateVisionCommand() {
        return limelight.run(() ->{
            final Pose2d currentRobotPose = drivetrain.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                drivetrain.addVisionMeasurement(
                    m.poseEstimate.pose,
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        });
    }

// Pose2d poseA = new Pose2d();
// Pose2d poseB = new Pose2d();

// StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
//   .getStructTopic("MyPose", Pose2d.struct).publish();
// StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
//   .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

// private void periodic() {
//   publisher.set(poseA);
//   arrayPublisher.set(new Pose2d[] {poseA, poseB});
// }
 
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
