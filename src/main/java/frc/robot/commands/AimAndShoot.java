// package frc.robot.commands;

// import java.util.function.DoubleSupplier;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Hopper.Indexer;
// import frc.robot.subsystems.Hopper.Intake;
// import frc.robot.subsystems.Hopper.Pivot;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.subsystems.Shooter.Feeder;
// import frc.robot.subsystems.Shooter.Shooter;
// import frc.robot.commands.AimAndDrive;
// import frc.robot.commands.PrepareShot;
// import frc.robot.commands.FireNTheHole;

// public class AimAndShoot extends Command{


//     private final CommandSwerveDrivetrain swerve;
//     private final Limelight limelight;
//     private final DoubleSupplier forwardInput;
//     private final DoubleSupplier leftInput;


//     public Command AimAndShoot(Limelight objLimelight_in, CommandSwerveDrivetrain objSwerve_in, Shooter objShooter, Feeder objFeeder, Indexer objIndexer, Intake objIntake, Pivot objPivot) {
//         final AimAndDrive aimAndDrive = new AimAndDrive(swerve, forwardInput, leftInput);
//         final PrepareShot prepareShot = new PrepareShot(null, limelight);
//         return Commands.parallel(
//             aimAndDrive,
//             prepareShot,
//             Commands.waitUntil(() -> aimAndDrive.isAimed() && prepareShot.isReadyToShoot())
//                 .andThen(new FireNTheHole(objFeeder, objIndexer, objIntake, objPivot))
//         );
//     }   
// }
