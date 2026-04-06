package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimAndDrive;
import frc.robot.commands.PrepareShot;
import frc.robot.commands.FireNTheHole;

import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Hopper.Indexer;
import frc.robot.subsystems.Hopper.Intake;
import frc.robot.subsystems.Hopper.Pivot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;



public class AimandShoot2 extends SequentialCommandGroup {


  /** Creates a new AimandShoot2. */
  public AimandShoot2(Limelight objLimelight, CommandSwerveDrivetrain objSwerve, Shooter objShooter, Feeder objFeeder, Indexer objIndexer, Intake objIntake, Pivot objPivot, DoubleSupplier dsForward, DoubleSupplier dsLeft) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ParallelCommandGroup(
        new PrepareShot(objShooter, objLimelight),
        new AimAndDrive(objSwerve, dsForward, dsLeft),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.75),
          new FireNTheHole(objFeeder, objIndexer, objIntake, objPivot)
          
        )
      )
    );
  }
}
