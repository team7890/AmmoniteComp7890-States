package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Landmarks;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;


public class Limelight extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    private double dLastKnownDistance = 0.0;
    private boolean bHasTarget = false;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (
            poseEstimate_MegaTag1 == null 
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
        poseEstimate_MegaTag2.pose = new Pose2d(
            poseEstimate_MegaTag1.pose.getTranslation(), // === Switched from 2
            poseEstimate_MegaTag1.pose.getRotation()
        );
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);

        posePublisher.set(poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    public double getDistanceToHub() {
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (
                poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            bHasTarget = false;
            return dLastKnownDistance;
        }
        final Translation2d robotPosition = poseEstimate_MegaTag2.pose.getTranslation();
        final Translation2d hubPosition = Landmarks.hubPosition(); 
        dLastKnownDistance = robotPosition.getDistance(hubPosition); 
        bHasTarget = true;
        return dLastKnownDistance;
    }

    public double getDistanceToHub2(double dOffsetX, double dOffsetY) {
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (
                poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            return dLastKnownDistance;
        }
        final Translation2d robotPosition = poseEstimate_MegaTag2.pose.getTranslation();
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d hubDelta = new Translation2d(dOffsetX, dOffsetY);
        final Translation2d hubAdjPosition = hubPosition.minus(hubDelta);
        dLastKnownDistance = robotPosition .getDistance(hubAdjPosition);
        return dLastKnownDistance;
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Distance to Hub", getDistanceToHub());
        SmartDashboard.putBoolean("LL has target", bHasTarget);
    }


    
}
