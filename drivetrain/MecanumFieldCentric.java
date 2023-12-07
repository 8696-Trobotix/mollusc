package org.firstinspires.ftc.teamcode.mollusc.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumFieldCentric implements Drivetrain {

    public DrivetrainBaseFourWheel base;
    public IMU imu;

    public double driveScaleMax = 1.0, strafeScaleMax = 1.0, turnScaleMax = 1.0;
    public double yawOffset = 0;

    public MecanumFieldCentric(
        DrivetrainBaseFourWheel base, 
        IMU imu
    ) {
        this.base = base;
        this.imu = imu;
    }

    // Scales can be thought of as maximums, where increase is linear from 0 --> [scale] as 0 --> 1.
    // `yawOffset` is in radians.
    public void setDriveParams(double driveScaleMax, double strafeScaleMax, double turnScaleMax, double yawOffset) {
        this.driveScaleMax  = driveScaleMax;
        this.strafeScaleMax = strafeScaleMax;
        this.turnScaleMax   = turnScaleMax;
        this.yawOffset      = yawOffset;
    }

    public void drive(double drive, double strafe, double turn) {
        drive  *= driveScaleMax;
        strafe *= strafeScaleMax;
        turn   *= turnScaleMax;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + yawOffset;

        // Calculations based on GM0.
        double rotX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
        // Normalize. Also prevents power values from exceeding 1.0.
        double max = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double fl = (rotY + rotX + turn) / max;
        double fr = (rotY - rotX - turn) / max;
        double rl = (rotY - rotX + turn) / max;
        double rr = (rotY + rotX - turn) / max;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        rearLeft.setPower(rl);
        rearRight.setPower(rr);
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
