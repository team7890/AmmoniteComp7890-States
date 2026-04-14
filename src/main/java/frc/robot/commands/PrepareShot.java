package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.Constants.Shooting;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;


public class PrepareShot extends Command {
    
    private InterpolatingDoubleTreeMap tableSpeed = new InterpolatingDoubleTreeMap();


    private final Shooter shooter;
    private final Limelight objlimelight;
    private double dInitSpeedAdjustment;
    private double dTimeStart;
    private double dElapsedTime;

    private double dVar;

    public PrepareShot(Shooter shooter, Limelight objLimelight) {
        this.shooter = shooter;
        this.objlimelight = objLimelight;
        addRequirements(shooter, objLimelight);

        tableSpeed.put(Shooting.dKey1, Shooting.dRPM1);
        tableSpeed.put(Shooting.dKey2, Shooting.dRPM2);
        tableSpeed.put(Shooting.dKey3, Shooting.dRPM3);
        tableSpeed.put(Shooting.dkey4, Shooting.dRPM4);
        tableSpeed.put(Shooting.dkey5, Shooting.dRPM5);
        tableSpeed.put(Shooting.dKey6, Shooting.dRPM6);
        tableSpeed.put(Shooting.dKey7, Shooting.dRPM7);

    }

    public boolean isReadyToShoot() {

        return shooter.isVelocityWithinTolarance();
    }

    @Override
    public void initialize() {
        dTimeStart = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        final double distanceToHub = objlimelight.getDistanceToHub();
        
        
        dElapsedTime = Timer.getFPGATimestamp() - dTimeStart;

        shooter.runShooterRPM(tableSpeed.get(distanceToHub)); // === ADD TABLEVAR HERE? 
    }

    @Override 
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }

}
