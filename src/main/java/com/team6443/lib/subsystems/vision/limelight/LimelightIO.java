// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.vision.limelight;

import java.util.List;

import com.team6443.lib.constants.FieldConstants;
import com.team6443.lib.logging.interfaces.Loggable;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.subsystems.vision.util.AprilTagCornerPosition;
import com.team6443.lib.subsystems.vision.interfaces.CameraIO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** 
 * Interface to represent data and functionality that can be preformed by a Limelight camera
 */
public interface LimelightIO extends CameraIO, Loggable {

    class VisionPoseEstimation {
        public Pose2d latencyCompensatedRobotFieldPose; // Robot field pose compensating for camera latency
        public Pose2d uncompensatedRobotFieldPose;      // Robot field pose not compensating for camera latency

        public VisionPoseEstimation(Pose2d compensated, Pose2d uncompensated, Translation2d cameraLocation){
            this.latencyCompensatedRobotFieldPose = compensated;
            this.uncompensatedRobotFieldPose = uncompensated;
        }
    }

    /**
     * Does this limelight currently see any targets
     * @return True if targets are seen false if not
     */
    public boolean hasTarget();

    // ---- Active Target ---
    // A limelight has one single active tag that it uses for these values 

    /**
     * Get the offset from the limelight crosshair to the active chosen tag (limelight picks the closest/largest area tag or the primary tag id if set)
     * @return
     */
    public double getXOffset();

    /**
     * Get the offset from the limelight crosshair to the active chosen tag as a a Rotation2D
     * @return The X offset as a rotation
     */
    public default Rotation2d getXRotationOffset() { return Rotation2d.fromDegrees(getXOffset()); };

    /**
     * Get the tag ID of the currently selected tag
     * @return the ID of the tag we are currently tracking
     */
    public int getTagID();

    /**
     * Get the xy-pairs representing the corner positions 
     * @return List of 4 xy-pairs that represent the corner positions of the currently active tag
     */
    public List<AprilTagCornerPosition> getTagCornerPositions();

    /**
     * How much do we want to throttle or unthrone the processor 0 (is completely untroubled) - 100-200 (in robot disabled)
     * @param throttle Amount to throttle the camera to
     * @return True on success false on failure
     */
    public boolean setThrottle(int throttle);

    // --- '2910 Style' Trigonometric Robot Pose Computation ---

    /**
     * Compute the height of the april tag in pixels based on the positions of the tag corners utilizing contours to achieve these positions
     * @param cornerPositions The position of the tag on the screen
     * @return The height of the tag in pixels
     */
    static double computeTagHeightInPixels(List<AprilTagCornerPosition> cornerPositions) {
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
    
        // Loop over list of corners
        for (int i = 0; i < 4; i++) {
            double y = cornerPositions.get(i).y;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
        }
    
        return (maxY - minY);
    }

    /**
     * Compute the rotational based height of a tag (describes in VisionInputs.java)
     * @param tagHeightPixels The height of the tag in pixels
     * @param cameraVerticalFOVDegrees The vertical FOV of the camera used to determine the pixel height
     * @param verticalResolutionPixels The vertical resolution of the camera used to determine the pixel height
     * @return Rotation2D representing the height of the tag
     */
    static Rotation2d computeTagHeightInRotations(double tagHeightPixels, double cameraVerticalFOVDegrees, double verticalResolutionPixels) {
        return Rotation2d.fromDegrees(tagHeightPixels * cameraVerticalFOVDegrees / verticalResolutionPixels);
    }

    /**
     * Compute the horizontal distance from the camera to the tag in meters using trig
     * Utilizes basic trig to derive the distance
     * @param tagHeightRotations The height of the tag represented as a rotation2D (how tall the tag looks)
     * @param tagBottomToCameraOffsetMeters The offset between the bottom of the tag and the camera in meters
     * @return The distance from the camera to the tag in meters
     */
    static double computeDistanceToTagInMeters(Rotation2d tagHeightRotations, double tagBottomToCameraOffsetMeters){

        // This tangent represents how much vertical space the AprilTag occupies per unit of horizontal distance
        double tanTheta = tagHeightRotations.getTan();

        // Prevent divide-by-zero or impossible angle cases.
        if (Math.abs(tanTheta) < 1e-9) {
            return Double.POSITIVE_INFINITY;
        }

        double tagHeight = FieldConstants.APRIL_TAG_HEIGHT_METERS;

        // The height to the bottom of the tag from the camera
        double cameraToTagBottomHeight = tagBottomToCameraOffsetMeters;

        // The height to the top of the tag from the camera
        double cameraToTagTopHeight = cameraToTagBottomHeight + tagHeight;

        // Quadratic formula discriminant b^2 - 4ac
        // Broken down the (physical height of the tag)^2 - ()  * (vertical spread between camera and tag)
        // Effectively measures whether the angular size of the AprilTag is physically possible with the given state
        // It is able to measure this because bad inputs will result in invalid angles 
        double discriminant = Math.pow(tagHeight, 2) - 4 * Math.pow(tanTheta, 2) * cameraToTagTopHeight * cameraToTagBottomHeight;

        // If discriminant is significantly negative, the geometry is invalid → reject.
    
        discriminant = Math.max(0, discriminant);

        // This sqrt term in our quadratic formula effectively represents how valid our distance measurement is
        // sqrtTerm is largest when the tag is very small, there are many distances that could apply, as the angle increases the tag becomes larger and reduces the number of valid distnaces
        // When the term is 0 this means there is exactly 1 possible distance and we are as close as possible to the tag
        double sqrtTerm = Math.sqrt(discriminant);

        // This is the completion of the quadratic formula and the horizontal distance from the camera to the AprilTag in meters
        double distance = (tagHeight + sqrtTerm) / (2 * tanTheta);
        return Math.abs(distance);

    }

    static double computeDistanceToTagInMetersSimple(Rotation2d tagHeightRotations) {

        double tanTheta = tagHeightRotations.getTan();
    
        if (Math.abs(tanTheta) < 1e-9) {
            return Double.POSITIVE_INFINITY;
        }
    
        double tagHeight = FieldConstants.APRIL_TAG_HEIGHT_METERS;
    
        return Math.abs(tagHeight / tanTheta);
    }

    /**
     * Compute the translation from the camera position to the center of the robot
     * @param cameraConfiguration Configuration of the camera
     * @param robotRotation Current rotation of the robot
     * @param cameraRotationToTarget Rotation of the camera to the tag
     * @return Translation of the camera to the center of the robot factoring in rotation
     */
    static Translation2d computeCameraToRobotCenter(
        CameraConfiguration cameraConfiguration, 
        Rotation2d robotRotation, 
        Rotation2d cameraRotationToTarget
    ){
        // Apply the current robot rotation to the translation from the camera position to the robot to get the current position of the camera with the robots current rotation applied
        Translation2d cameraToRobotCenter = cameraConfiguration.CameraLocation.TranslationToRobotCenter.rotateBy(robotRotation);

        return cameraToRobotCenter;
    }


    /**
     * Compute the translation from the camera to the tag
     * @param cameraConfiguration The configuration for the camera
     * @param robotRotation The rotation of the robot at time of computation
     * @param cameraRotationToTarget The rotation from the camera to the tag
     * @param distanceToTagMeters The distance to the tag in meters
     * @return Translation from the camera to the tag
     */
    static Translation2d computeCameraToTag(CameraConfiguration cameraConfiguration, Rotation2d robotRotation, Rotation2d cameraRotationToTarget, double distanceToTagMeters){
        // Scale the camera rotation by the camera rotation ratio scalar set in the configuration, invert the system because limelight left is positive and wpilib is inverted.
        Rotation2d scaledCameraRotationToTarget = Rotation2d.fromDegrees(-(cameraRotationToTarget.getDegrees() / cameraConfiguration.CameraXRotationScalar));

        // Add rotation of camera to the robot position so that we can use the actual rotation of the camera in our computations
        Rotation2d robotRotationWithLimelightYaw = robotRotation.plus(cameraConfiguration.CameraLocation.MountingYaw);

        // The angle from the camera to the tag as reported by the limelight
        Rotation2d cameraAngleToTag = scaledCameraRotationToTarget.plus(robotRotationWithLimelightYaw);

        // The translation from the camera to the tag as computed by our distance and angle
        Translation2d cameraToTagTranslation = new Translation2d(distanceToTagMeters, cameraAngleToTag);

        return cameraToTagTranslation;
    }

    /**
     * Calculate the estimate for the robot pose based on the current Limelight camera feed 
     * @param tagPose The pose2d of the tag we are currently detecting
     * @param cameraConfiguration The configuration of the camera being used to detect it
     * @param distanceToTagMeters The distance to the tag in meters computed by computeDistanceToTagInMeters
     * @param cameraRotationToTarget The rotational offset from the camera to the tag
     * @return The latency compensated and uncompensated robot poses
     */
    static VisionPoseEstimation computeRobotPose(
        Pose2d tagPose, 
        CameraConfiguration cameraConfiguration, 
        double distanceToTagMeters, 
        Rotation2d cameraRotationToTarget,
        Rotation2d robotRotation,
        Translation2d cameraToRobotCenter,
        Translation2d cameraToTag,
        ChassisSpeeds latestFieldChassisSpeeds
    ){

        // Compute the field location of the camera relative to the tag
        Translation2d cameraFieldPosition = tagPose.getTranslation().minus(cameraToTag);

        // Add our rotation aware offset from our camera to the center of the robot to our field camera position to get the robot field position
        Translation2d robotFieldPosition = cameraFieldPosition.minus(cameraToRobotCenter);

        // Based on current robot speed we can infer where the robot is on the field at this given moment of time if we know how much latency occurred
        ChassisSpeeds fieldRelativeChassisSpeeds  = latestFieldChassisSpeeds;
        Translation2d latencyCompensatedRobotFieldPosition = new Translation2d(
                        robotFieldPosition.getX()
                        + (fieldRelativeChassisSpeeds.vxMetersPerSecond * 0 /* latency */),
                        robotFieldPosition.getY()
                        + (fieldRelativeChassisSpeeds.vyMetersPerSecond * 0 /* latency, TODO: add from net tables? */));

    
        // The vision pose estimation with both the latency compensated pose and the uncompensated pose
        return new VisionPoseEstimation(
            new Pose2d(latencyCompensatedRobotFieldPosition, robotRotation), 
            new Pose2d(robotFieldPosition, robotRotation),
            latencyCompensatedRobotFieldPosition.plus(cameraToRobotCenter).plus(cameraToTag)
        );

    }

}
