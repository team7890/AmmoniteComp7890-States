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
import frc.robot.util.Lights;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Hopper.Intake;
import frc.robot.commands.AimandShoot2;
import frc.robot.commands.AimandShootXLock;
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

    private double dTableVar = 0;

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
    private final Limelight objLimelight = new Limelight("limelight-genesis");

    private final Field2d field = new Field2d();;

    private final Lights objLights = new Lights();

    

    // === PathPlanner === \\
    private final SendableChooser<Command> autoChooser;
 


    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putNumber("Shooter Sub/Interp Variable", dTableVar);
        //autoChooser.addOption("Chaos with Trench","Chaos with trench");
        // Command Chaos = AutoBuilder.buildAuto("Take 5");

        drivetrain.registerTelemetry(logger::telemeterize);

        // autoChooser.addOption("Take 5", Chaos);

        NamedCommands.registerCommand("stop Shooter", getAutonomousCommand());
        NamedCommands.registerCommand("Intake Pivot", getAutonomousCommand());
        NamedCommands.registerCommand("Print Message", getAutonomousCommand());
        NamedCommands.registerCommand("Run Intake", getAutonomousCommand());
        NamedCommands.registerCommand("stop Shooter", new ShooterFull(objShooter, MotorSpeeds.dShooterTDist, objFeeder, objIndexer, objIntake, objPivot));
        NamedCommands.registerCommand("Run Intake", new RunCommand(()-> objIntake.runIntake(MaxSpeed)).withTimeout(5.0));
        NamedCommands.registerCommand("Auto shots", new TrenchShotAuto(objShooter, MotorSpeeds.dShooterTDist, objFeeder, objIndexer, objIntake, objPivot));
        NamedCommands.registerCommand("Auto Shoot", new TrenchShotAuto(objShooter, MotorSpeeds.dShooterTDist, objFeeder, objIndexer, objIntake, objPivot));
        
        new EventTrigger("Run Intake").whileTrue(new RunCommand(()-> objIntake.runIntake(0.425), objIntake));
        //new EventTrigger("Auto Shoot").whileTrue(new RunCommand(()-> objShooter.runShooterRPM(MotorSpeeds.dShooter3M), objShooter));
        new EventTrigger("Intake Pivot").whileTrue(new PivotIntake(objPivot, MotorSpeeds.dPivotSpeed));
        new EventTrigger("Shooter Run").whileTrue(new ShooterFull(objShooter, 2500, objFeeder, objIndexer, objIntake, objPivot));
        new EventTrigger("Shoot Slower").whileTrue(new ShooterFull(objShooter, 4150, objFeeder, objIndexer, objIntake, objPivot)); // === 3975
        new EventTrigger("Second Shot").whileTrue(new ShooterFull(objShooter, 4550, objFeeder, objIndexer, objIntake, objPivot)); // === 4515
        new EventTrigger("Lime Shoot").whileTrue(new AimandShoot2(objLimelight, drivetrain, objShooter, objFeeder, objIndexer, objIntake, objPivot, () -> 0.0, () -> 0.0));

        configureBindings();

      

        SmartDashboard.putNumber("Util/Match Time", DriverStation.getMatchTime());
        // SmartDashboard.putNumber("null", DriverStation.getAlliance());

  
   

    
        field.getRobotPose();
            SmartDashboard.putData("Util/BadField", field);

        
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        objLimelight.setDefaultCommand(updateVisionCommand());

        

                                             /// SET DEFAULTCOMANDS ///
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
            new RunCommand(()-> objShooter.runShooterRPM(4000), objShooter) //Original 0.15
        );

        objPivot.setDefaultCommand(
            new RunCommand(() -> objPivot.stopPivot(), objPivot)
        );

        objIntake.setDefaultCommand(
            new RunCommand(() -> objIntake.stopIntake(), objIntake)
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

        // xboxDriver.axisGreaterThan(3, 0.25).whileTrue(
        //     new AimandShoot2(  
        //         objLimelight, drivetrain, objShooter, objFeeder, objIndexer, objIntake, objPivot,
        //         () -> -xboxDriver.getLeftX(),
        //         () -> -xboxDriver.getLeftY()
        //     )
            // .alongWith(
            //     new SequentialCommandGroup(
            //         new WaitCommand(5), drivetrain.applyRequest(() -> brake)))
        // );

        xboxDriver.axisGreaterThan(3, 0.25).whileTrue(
            new AimandShootXLock(
                objLimelight, drivetrain, objShooter, objFeeder, objIndexer, objIntake, objPivot, 
                () -> -xboxDriver.getLeftX(),
                () -> -xboxDriver.getLeftY()
                )
                // .alongWith(objLights.shootingLights())
        );
      
        

        ///TEST SHOOT FOR INTERP TABLE///
        // xboxDriver. b().whileTrue(new ShooterFull(objShooter, MotorSpeeds.dtestRPM, objFeeder, objIndexer, objIntake, objPivot));

       
          ///Far shooter///
          /// 
        xboxDriver. b().whileTrue(new ShooterFull(objShooter, MotorSpeeds.dPassingRPM, objFeeder, objIndexer, objIntake, objPivot));

         ///Passing///
        xboxDriver. leftBumper().whileTrue(new Passing(objShooter, objFeeder, objIndexer, objIntake));

        xboxDriver.rightBumper().whileTrue(new ShooterFull(objShooter, MotorSpeeds.dShooterTDist, objFeeder, objIndexer, objIntake, objPivot));

        xboxDriver. y().toggleOnTrue(new RunCommand(()-> objShooter.stopShooter(), objShooter));



        // === Intake === \\ 
        xboxDriver.axisGreaterThan(2, 0.1).whileTrue(new RunCommand(
                ()-> objIntake.runIntake(MotorSpeeds.dIntakeSpeed), objIntake));

        xboxDriver.a().toggleOnTrue(new PivotIntake(objPivot, MotorSpeeds.dPivotSpeed));

        xboxDriver.povDown().whileTrue(new RunCommand(
                () -> objIntake.runIntake(-MotorSpeeds.dIntakeSpeed), objIntake)
                            .alongWith(new RunCommand(
                () -> objIndexer.runIndexer(0.5), objIndexer)));
                    

        xboxDriver.povUp().whileTrue(new RunCommand(
                () -> objFeeder.runFeeder(-MotorSpeeds.dFeederSpeed), objFeeder));

        xboxDriver.povLeft().whileTrue(objShooter.runOnce(() -> objShooter.tweakRPM(25.0)));

        xboxDriver.povRight().whileTrue(objShooter.runOnce(() -> objShooter.tweakRPM(-5.0)));



        //  = PsuedoCode = \\ xboxDriver.povLeft/Right().onTrue(interp var +/- 25);

        // xboxDriver.povLeft().onTrue(new RunCommand(() -> incTabVar()));
        // xboxDriver.povRight().onTrue(new RunCommand(() -> decTabVar()));

       
       drivetrain.registerTelemetry(logger::telemeterize);

       // Logger.recordOutput("Swerve/Pose", getPose());
      
    }

    private Command updateVisionCommand() {
        return objLimelight.run(() ->{
            final Pose2d currentRobotPose = drivetrain.getState().Pose;
            final Optional<Limelight.Measurement> measurement = objLimelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                drivetrain.addVisionMeasurement(
                    m.poseEstimate.pose,
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        });
    }

 
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // public double incTabVar (){
    //     return dTableVar += 25;
    // }

    // public double decTabVar (){
    //     return dTableVar -= 5;
    // }

    

}
