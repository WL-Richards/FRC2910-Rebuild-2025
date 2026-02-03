package com.team6443.lib.subsystems.vision.interfaces;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import com.team6443.lib.constants.RobotStateConstants;
import com.team6443.lib.constants.fields.interfaces.YearFieldConstantable;
import com.team6443.lib.subsystems.vision.LimelightVisionInputs;
import com.team6443.lib.subsystems.vision.util.FiducialObservation;
import com.team6443.lib.subsystems.vision.util.MegatagPoseEstimate;
import com.team6443.lib.subsystems.vision.util.VisionFieldPoseEstimate;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.util.Units;


public abstract class LimelightIO extends CameraIO {

    /**
     * Update the state of this camera
     * @param inputs Vision inputs to be updated
     */
    public abstract void updateInputs(LimelightVisionInputs inputs);


    public static Optional<VisionFieldPoseEstimate> proccessVisionInputs(
        LimelightVisionInputs inputs, 
        CameraIO camera, 
        YearFieldConstantable yearSpecificFieldConstants,
        Supplier<Double> lastUsedMegatagTimestampSupplier,
        Function<Double, Optional<Pose2d>> fieldRobotPoseSupplier,
        BiFunction<Double, Double, Optional<Double>> maxYawSpeedInRangeSupplier
        ){

        // Return guard for no tags
        if(!inputs.hasTag){
 
            return Optional.empty();
        }


        if(inputs.megatag1PoseEstimate != null){

            Optional<VisionFieldPoseEstimate> mt1Estimate = proccessMegatag1Estimate(inputs, camera, lastUsedMegatagTimestampSupplier, fieldRobotPoseSupplier, maxYawSpeedInRangeSupplier);

            if(mt1Estimate.isPresent()){
                return mt1Estimate;
            }

            Optional<VisionFieldPoseEstimate> gyroFusedEstimate = LimelightIO.fuseGyroWithGyro(
                inputs, 
                inputs.megatag1PoseEstimate, 
                camera, 
                yearSpecificFieldConstants, 
                lastUsedMegatagTimestampSupplier, 
                maxYawSpeedInRangeSupplier, 
            fieldRobotPoseSupplier);

            if (gyroFusedEstimate.isPresent()){
                return gyroFusedEstimate;
            }

            
        }

        return Optional.empty();
    }

    public static Optional<VisionFieldPoseEstimate> fuseEstimates(
        Function<Double, Optional<Pose2d>> fieldRobotPoseSupplier,
        List<VisionFieldPoseEstimate> estimates) {

        if (estimates == null || estimates.isEmpty()) {
            return Optional.empty();
        }

        if (estimates.size() == 1) {
            return Optional.of(estimates.get(0));
        }

        // Find the latest timestamp
        double latestTimestamp = estimates.stream()
            .mapToDouble(VisionFieldPoseEstimate::getTimestampSeconds)
            .max()
            .orElse(0.0);

        // Get the reference pose at latest timestamp
        Optional<Pose2d> latestPose = fieldRobotPoseSupplier.apply(latestTimestamp);
        if (latestPose.isEmpty()) {
            return Optional.empty();
        }

        // Transform all estimates to the latest timestamp and accumulate weights
        double weightedXSum = 0.0;
        double weightedYSum = 0.0;
        double weightedCosSum = 0.0;
        double weightedSinSum = 0.0;
        double totalWeightX = 0.0;
        double totalWeightY = 0.0;
        double totalWeightHeading = 0.0;
        int totalNumTags = 0;
        boolean hasValidHeading = false;

        for (VisionFieldPoseEstimate estimate : estimates) {
            // Get pose at estimate's timestamp for transformation
            Optional<Pose2d> poseAtEstimateTime = fieldRobotPoseSupplier.apply(estimate.getTimestampSeconds());
            if (poseAtEstimateTime.isEmpty()) {
                continue;
            }

            // Transform estimate pose to latest timestamp
            Transform2d transform = latestPose.get().minus(poseAtEstimateTime.get());
            Pose2d transformedPose = estimate.getVisionRobotPoseMeters().transformBy(transform);

            // Calculate variance (stdDev^2)
            var stdDevs = estimate.getVisionMeasurementStdDevs();
            double varianceX = stdDevs.get(0, 0) * stdDevs.get(0, 0);
            double varianceY = stdDevs.get(1, 0) * stdDevs.get(1, 0);
            double varianceHeading = stdDevs.get(2, 0) * stdDevs.get(2, 0);

            // Inverse-variance weights
            double weightX = 1.0 / varianceX;
            double weightY = 1.0 / varianceY;

            weightedXSum += transformedPose.getTranslation().getX() * weightX;
            weightedYSum += transformedPose.getTranslation().getY() * weightY;
            totalWeightX += weightX;
            totalWeightY += weightY;

            // Only include heading if variance is reasonable
            if (varianceHeading < RobotStateConstants.Vision.kLargeVariance) {
                double weightHeading = 1.0 / varianceHeading;
                weightedCosSum += transformedPose.getRotation().getCos() * weightHeading;
                weightedSinSum += transformedPose.getRotation().getSin() * weightHeading;
                totalWeightHeading += weightHeading;
                hasValidHeading = true;
            }

            totalNumTags += estimate.getNumTags();
        }

        // Ensure we have valid weights
        if (totalWeightX == 0.0 || totalWeightY == 0.0) {
            return Optional.empty();
        }

        // Calculate fused position
        double fusedX = weightedXSum / totalWeightX;
        double fusedY = weightedYSum / totalWeightY;

        // Calculate fused heading - use latest estimate's heading if no valid heading weights
        Rotation2d fusedHeading;
        double fusedHeadingStdDev;
        if (hasValidHeading && totalWeightHeading > 0.0) {
            fusedHeading = new Rotation2d(weightedCosSum, weightedSinSum);
            fusedHeadingStdDev = Math.sqrt(1.0 / totalWeightHeading);
        } else {
            // Fall back to latest estimate's heading
            VisionFieldPoseEstimate latestEstimate = estimates.stream()
                .max(Comparator.comparingDouble(VisionFieldPoseEstimate::getTimestampSeconds))
                .get();
            fusedHeading = latestEstimate.getVisionRobotPoseMeters().getRotation();
            fusedHeadingStdDev = latestEstimate.getVisionMeasurementStdDevs().get(2, 0);
        }

        Pose2d fusedPose = new Pose2d(new Translation2d(fusedX, fusedY), fusedHeading);

        Matrix<N3, N1> fusedStdDev = VecBuilder.fill(
            Math.sqrt(1.0 / totalWeightX),
            Math.sqrt(1.0 / totalWeightY),
            fusedHeadingStdDev);

        return Optional.of(new VisionFieldPoseEstimate(fusedPose, latestTimestamp, fusedStdDev, totalNumTags));
    }

    protected static Optional<VisionFieldPoseEstimate> fuseGyroWithGyro(
        LimelightVisionInputs inputs, 
        MegatagPoseEstimate poseEstimate, 
        CameraIO camera,
        YearFieldConstantable yearSpecificFieldConstants,
        Supplier<Double> lastUsedMegatagTimestampSupplier,
        BiFunction<Double, Double, Optional<Double>> maxYawSpeedInRangeSupplier,
        Function<Double, Optional<Pose2d>> fieldRobotPoseSupplier
    ){
        // 1. ADD CHECK TO ENSURE THIS TIME IS AFTER THE LAST UPDATED TIME
        if(inputs.megatag1PoseEstimate.timestampSeconds() <= lastUsedMegatagTimestampSupplier.get()){
            return Optional.empty();
        }

        // Use Megatag directly when 2 or more tags are visible
        if (poseEstimate.fiducialIds().length > 1) {
            return Optional.empty();
        }

        // If the rotational speed of the robot yaw is greater than some Radians per second over some recent period we want to filter out these values as they are unreliable
        if(maxYawSpeedInRangeSupplier.apply(
            poseEstimate.timestampSeconds() - RobotStateConstants.Vision.kHighYawLookbackS,  
            poseEstimate.timestampSeconds()
        ).orElse(Double.POSITIVE_INFINITY) > RobotStateConstants.Vision.kHighYawVelocityRadS){
            return Optional.empty();
        }

        Optional<Pose2d> robotPoseAtEstimateTime = fieldRobotPoseSupplier.apply(poseEstimate.timestampSeconds());
        if (robotPoseAtEstimateTime.isEmpty()){
            return Optional.empty();
        }

        Optional<Pose3d> maybeFieldToTag = yearSpecificFieldConstants.getFieldLayout().getTagPose(poseEstimate.fiducialIds()[0]);
        if (maybeFieldToTag.isEmpty()) {
            return Optional.empty();
        }

        Pose2d fieldToTag =
            new Pose2d(maybeFieldToTag.get().toPose2d().getTranslation(), Rotation2d.kZero);
        Pose2d robotToTag = fieldToTag.relativeTo(poseEstimate.fieldToRobot());

        Pose2d estimatedPose =
                new Pose2d(
                        fieldToTag
                                .getTranslation()
                                .minus(
                                        robotToTag
                                                .getTranslation()
                                                .rotateBy(robotPoseAtEstimateTime.get().getRotation())),
                        robotPoseAtEstimateTime.get().getRotation());

        Matrix<N6, N1> estimatedPoseStdDevs = poseEstimate.stdDevs();
        return Optional.of(
            new VisionFieldPoseEstimate(
                estimatedPose, 
                poseEstimate.timestampSeconds(),
                VecBuilder.fill(
                    estimatedPoseStdDevs.get(0, 0),
                    estimatedPoseStdDevs.get(1, 0),
                    estimatedPoseStdDevs.get(5, 0)
                ), 
                poseEstimate.fiducialIds().length
            )
        );
    }

    protected static Optional<VisionFieldPoseEstimate> proccessMegatag1Estimate(
        LimelightVisionInputs inputs,
        CameraIO camera,
        Supplier<Double> lastUsedMegatagTimestampSupplier,
        Function<Double, Optional<Pose2d>> fieldRobotPoseSupplier,
        BiFunction<Double, Double, Optional<Double>> maxYawSpeedInRangeSupplier
    ){
        // 1. ADD CHECK TO ENSURE THIS TIME IS AFTER THE LAST UPDATED TIME
        if(inputs.megatag1PoseEstimate.timestampSeconds() <= lastUsedMegatagTimestampSupplier.get()){

            return Optional.empty();
        }

        // 2. REJECT IF SPINNING TOO FAST - vision estimates are unreliable at high yaw rates

        double yawSpinRate = maxYawSpeedInRangeSupplier.apply(
            inputs.megatag1PoseEstimate.timestampSeconds() - RobotStateConstants.Vision.kHighYawLookbackS,
            inputs.megatag1PoseEstimate.timestampSeconds()
        ).orElse(Double.POSITIVE_INFINITY);
        System.out.println(yawSpinRate);
        if(yawSpinRate > RobotStateConstants.Vision.kHighYawVelocityRadS){
            return Optional.empty();
        }

        // 3. DO SINGLE TAG CHECKS
        if (!LimelightIO.checkSingleTagRequirements(inputs, inputs.megatag1PoseEstimate, camera, fieldRobotPoseSupplier)){

            return Optional.empty();
        }

        // 4. CHECK IF THE TRANSLATION NORMAL IS LESS THAN 1
        if (inputs.megatag1PoseEstimate.fieldToRobot().getTranslation().getNorm()
                < RobotStateConstants.Vision.kDefaultNormThreshold) {

            return Optional.empty();
        }

        // 5. Ensure robot Z is less than some threshold
        if (Math.abs(inputs.estimatedRobotPose3d.getZ()) > RobotStateConstants.Vision.kDefaultZThreshold) {

            return Optional.empty();
        }

        // (Optional) 6. Check if we have an exclusive tag set to filter given some state

        // 7. Ensure we have some pose at the time of this estimate
        Optional<Pose2d> loggedPose = fieldRobotPoseSupplier.apply(inputs.megatag1PoseEstimate.timestampSeconds());
        if (loggedPose.isEmpty()) {

            return Optional.empty();
        }

        // 8. Take estimated pose and apply standard devs
        Pose2d estimatePose = inputs.megatag1PoseEstimate.fieldToRobot();
        Matrix<N6, N1> estimatedPoseStdDevs = inputs.megatag1PoseEstimate.stdDevs();
        return Optional.of(
            new VisionFieldPoseEstimate(
                estimatePose, 
                inputs.megatag1PoseEstimate.timestampSeconds(),
                VecBuilder.fill(
                    estimatedPoseStdDevs.get(0, 0),
                    estimatedPoseStdDevs.get(1, 0),
                    RobotStateConstants.Vision.kLargeVariance
                ), 
                inputs.megatag1PoseEstimate.fiducialIds().length
            )
        );
    }

    /**
     * Runs checks if we only have one tag to handle filtering and such
     * @param inputs The vision inputs, do not use the megatag pose values as this function is meant to be generic
     * @param poseEstimate The estimated megara pose we are running the check on
     * @param camera The cameraIO instance this check is for
     * @return True if all checks pass false if not
     */
    protected static boolean checkSingleTagRequirements( 
        LimelightVisionInputs inputs, 
        MegatagPoseEstimate poseEstimate, 
        CameraIO camera,
        Function<Double, Optional<Pose2d>> fieldRobotPoseSupplier
    ){
        // If we have 0 or 1 tags we want to run this
        if(poseEstimate.fiducialIds().length < 2){

            // Ensure all estimates are less than our set ambiguity
            for(FiducialObservation fiducial : inputs.fiducialObservations){
                if (fiducial.ambiguity() > RobotStateConstants.Vision.kDefaultAmbiguityThreshold) {

                    return false;
                }
            }

            if(poseEstimate.avgTagArea() < RobotStateConstants.Vision.kTagMinAreaForSingleTagMegatag){


                return false;
            }

            Optional<Pose2d> robotPoseAtEstimateTime = fieldRobotPoseSupplier.apply(poseEstimate.timestampSeconds());
            if (poseEstimate.avgTagArea() < RobotStateConstants.Vision.kTagAreaThresholdForYawCheck && robotPoseAtEstimateTime.isPresent()){
                double yawDiff =
                        Math.abs(
                                MathUtil.angleModulus(
                                    robotPoseAtEstimateTime.get().getRotation().getRadians()
                                                - poseEstimate
                                                        .fieldToRobot()
                                                        .getRotation()
                                                        .getRadians()));


                if (yawDiff > Units.degreesToRadians(RobotStateConstants.Vision.kDefaultYawDiffThreshold)) {

                    return false;
                }
                
            }
        }

        return true;
    }

}
