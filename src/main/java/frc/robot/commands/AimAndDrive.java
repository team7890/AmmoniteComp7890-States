package frc.robot.commands;



import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.DriveInputSmoother;
import frc.robot.util.GeometryUtil;
import frc.robot.util.ManualDriveInput;



public class AimAndDrive extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);
    
    private final CommandSwerveDrivetrain swerve;
    // private final DriveInputSmoother inputSmoother;
    private DoubleSupplier dsFwd, dsLeft;



    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotaionDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    public AimAndDrive(
        CommandSwerveDrivetrain swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        // this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        dsFwd = forwardInput;
        dsLeft = leftInput;
        addRequirements(swerve);
    }

//// ASK KRAUSS WHY IS IT MAD WHEN UNCOMMENTED////

    // public AimandDrive(CommandSwerveDrivetrain swerve) {
    //     this(swerve, () -> 0, () -> 0);
    // }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }

    @Override
    public void execute() {
        // final ManualDriveInput input = inputSmoother.getSmoothInput();
        swerve.setControl(
            fieldCentricFacingAngleRequest
            // .withVelocityX(Driving.kMaxSpeed.times(input.forward))
            // .withVelocityY(Driving.kMaxSpeed.times(input.left))
            .withVelocityX(Driving.kMaxSpeed.times(dsLeft.getAsDouble()))
            .withVelocityY(Driving.kMaxSpeed.times(dsFwd.getAsDouble()))
            .withTargetDirection(getDirectionToHub())
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
