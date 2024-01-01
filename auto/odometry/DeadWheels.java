package org.firstinspires.ftc.teamcode.mollusc.auto.odometry;

import org.firstinspires.ftc.teamcode.mollusc.wrapper.Encoder;

public class DeadWheels {

    public Encoder left, right, center;
    public Pose pose;

    private double leftPrev, rightPrev, centerPrev;

    // Distance between left and right encoders.
    public double trackWidth;
    // Distance between the center encoder and the robot's center of rotation.
    // Should be negative if the center encoder is in the back, and positive if in the front (of the robot).
    public double centerOffset;

    public DeadWheels(
        Pose initialPose, 
        Encoder left, 
        Encoder right, 
        Encoder center, 
        double trackWidth, 
        double centerOffset
    ) {
        this.pose = initialPose;
        this.left = left;
        this.right = right;
        this.center = center;
        this.trackWidth = trackWidth;
        this.centerOffset = centerOffset;

        leftPrev = left.getDisplacement();
        rightPrev = right.getDisplacement();
        centerPrev = center.getDisplacement();
    }

    // Call continuously in control loop to get updated positions in `pose`.
    public void update() {
        double leftDis = left.getDisplacement();
        double rightDis = right.getDisplacement();
        double centerDis = center.getDisplacement();

        double deltaLeft = leftDis - leftPrev;
        double deltaRight = rightDis - rightPrev;
        double deltaCenter = centerDis - centerPrev;

        leftPrev = leftDis;
        rightPrev = rightDis;
        centerPrev = centerDis;

        

        // double deltaHeading = (deltaLeft - deltaRight) / trackWidth;
        // double newHeading = pose.z + deltaHeading;// / 2?

        // double deltaX = (deltaLeft + deltaRight) / 2;
        // double deltaY = deltaCenter - (centerOffset * deltaHeading);

        // double sinHeading = Math.sin(deltaHeading); // newHeading?
        // double cosHeading = Math.cos(deltaHeading); // newHeading?
        
        // // ===
        // // Taken from https://github.com/FTCLib/FTCLib/blob/master/core/src/main/java/com/arcrobotics/ftclib/geometry/Pose2d.java#L155.
        // // I don't know what this is doing.
        // double s;
        // double c;
        // if (Math.abs(deltaHeading) < 1E-9) {
        //     s = 1.0 - 1.0 / 6.0 * deltaHeading * deltaHeading;
        //     c = 0.5 * deltaHeading;
        // } else {
        //     s = sinHeading / deltaHeading;
        //     c = (1 - cosHeading) / deltaHeading;
        // }
        // // ===
        
        // pose.x = deltaX * s - deltaY * c;
        // pose.y = deltaX * c + deltaY * s;
        // pose.z = newHeading;
    }

    public void zero() {
        pose = new Pose(0, 0, 0);
        left.reset();
        right.reset();
        center.reset();
        leftPrev = rightPrev = centerPrev = 0;
    }
}

/*
Copyright 2023 Trobotix 8696

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
