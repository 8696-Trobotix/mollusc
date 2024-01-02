package org.firstinspires.ftc.teamcode.mollusc.auto.odometry;

import org.firstinspires.ftc.teamcode.mollusc.wrapper.Encoder;

package org.firstinspires.ftc.robotcore.external.navigation;

public class DeadWheels {

    public Pose pose;
    public Encoder left, right, center;

    private double leftPrev, rightPrev, centerPrev;
    private double headingPrev;

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

        this.leftPrev = left.getDisplacement();
        this.rightPrev = right.getDisplacement();
        this.centerPrev = center.getDisplacement();
        this.headingPrev = pose.z;
    }

    // Call continuously in control loop to get updated positions in `pose`.
    public void update() {
        // Modified from https://github.com/FTCLib/FTCLib/blob/master/core/src/main/java/com/arcrobotics/ftclib/kinematics/HolonomicOdometry.java.

        double leftDis = left.getDisplacement();
        double rightDis = right.getDisplacement();
        double centerDis = center.getDisplacement();

        double deltaLeft = leftDis - leftPrev;
        double deltaRight = rightDis - rightPrev;
        double deltaCenter = centerDis - centerPrev;

        // Calculate change in heading using the difference between the displacements of the two parallel wheels.
        // This calculation cannot be done using the center wheel because linear displacement is not rotational displacement.
        double deltaHeading = AngleUtils.normalizeRadians((deltaLeft - deltaRight) / trackWidth);
        double heading = AngleUtils.normalizeRadians(headingPrev + deltaHeading);

        double deltaX = (deltaLeft + deltaRight) / 2;
        double deltaY = deltaCenter - (centerOffset * deltaHeading);

        double sinDeltaHeading = Math.sin(deltaHeading);
        double cosDeltaHeading = Math.cos(deltaHeading);

        // For small values of deltaHeading, limitations in floating-point calculations can lead to inaccuracies.
        // This section uses a Taylor series approximation when those small values are computed.
        double s;
        double c;
        if (Math.abs(deltaHeading) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * deltaHeading * deltaHeading;
            c = 0.5 * deltaHeading;
        } else {
            s = sinDeltaHeading / deltaHeading;
            c = (1 - cosDeltaHeading) / deltaHeading;
        }

        double transformX = deltaX * s - deltaY * c;
        double transformY = deltaX * c + deltaY * s;

        leftPrev = leftDis;
        rightPrev = rightDis;
        centerPrev = centerDis;
        headingPrev = heading;

        pose.x += transformX * Math.cos(heading) - transformY * Math.sin(heading)
        pose.y += transformX * Math.sin(heading) + transformY * Math.cos(heading)
        pose.z = heading;

        /*
        // Use these calculations if the first method above doesn't work.

        // Modified from https://github.com/Beta8397/virtual_robot/blob/master/TeamCode/src/org/firstinspires/ftc/teamcode/EncBot.java.

        double leftDis = left.getDisplacement();
        double rightDis = right.getDisplacement();
        double centerDis = center.getDisplacement();

        double deltaLeft = leftDis - leftPrev;
        double deltaRight = rightDis - rightPrev;
        double deltaCenter = centerDis - centerPrev;

        double deltaX = (deltaLeft + deltaRight) / 2;

        double deltaHeading = (deltaLeft - deltaRight) / trackWidth;
        double avgHeading = pose.z + deltaHeading / 2;

        double sinAvg = Math.sin(avgHeading);
        double cosAvg = Math.cos(avgHeading);

        pose.x += deltaX * sinAvg - centerDis * cosAvg;
        pose.y += deltaX * cosAvg + centerDis * sinAvg;
        pose.z = AngleUtils.normalizeRadians(pose.z + deltaHeading);
        */
    }

    public void zero() {
        setPose(0, 0, 0);
        left.reset();
        right.reset();
        center.reset();
        leftPrev = rightPrev = centerPrev = 0;
    }

    public void setPose(double x, double y, double z) {
        pose.x = x;
        pose.y = y;
        pose.z = z;
        headingPrev = z;
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
