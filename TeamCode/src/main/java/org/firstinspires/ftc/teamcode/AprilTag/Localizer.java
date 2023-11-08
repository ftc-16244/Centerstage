package org.firstinspires.ftc.teamcode.AprilTag;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.json.JSONException;
import org.json.JSONObject;
import java.util.List;

public class Localizer {
    private AprilTagProcessor processor = new AprilTagProcessor.Builder()
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            .build();
    private VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
    private VisionPortal visionPortal;

    private void init() {
        visionPortalBuilder.addProcessor(processor);
        visionPortal = visionPortalBuilder.build();
    }
    public Pose2d getPosition() {
        JSONObject main = new JSONObject();
        List<AprilTagDetection> detections = processor.getDetections();
        for(AprilTagDetection detection : detections) {
            JSONObject currentDetection = new JSONObject();
            try {
                currentDetection.put("yaw", detection.ftcPose.yaw);
                currentDetection.put("distance", detection.ftcPose.range);
                main.put(String.valueOf(detection.id), currentDetection);
            }
            catch (JSONException ignored) {}
        }
        return new Pose2d();
    }
}