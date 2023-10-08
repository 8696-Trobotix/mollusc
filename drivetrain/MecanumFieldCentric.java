/*
Mecanum Field Centric

Drivetrain hardware class.

vX-XII-XXIII
*/

package org.firstinspires.ftc.teamcode.mollusc.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumFieldCentric implements Drivetrain {

    public DrivetrainBase base;
    public IMU imu;

    public double turnSpeedMax = 0.9, strafeCorrection = 1.1;

    public MecanumFieldCentric(
        HardwareMap hardwareMap, 
        Telemetry telemetry, 
        String fl, DcMotorEx.Direction fld, 
        String fr, DcMotorEx.Direction frd, 
        String rl, DcMotorEx.Direction rld, 
        String rr, DcMotorEx.Direction rrd, 
        String imuName, 
            RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection, 
            RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection
    ) {
        base = new DrivetrainBase(hardwareMap, fl, fld, fr, frd, rl, rld, rr, rrd);

        // Connect IMU.
        imu = hardwareMap.get(IMU.class, "imu");

        // Configure IMU.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = logoFacingDirection;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = usbFacingDirection;
        RevHubOrientationOnRobot IMUOrientation = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(IMUOrientation));

        if (telemetry != null) {
            telemetry.log().add("Initialized field centric hardware.");
            telemetry.update();
        }
    }

    public void setDriveParams(double turnSpeedMax, double strafeCorrection) {
        this.turnSpeedMax = turnSpeedMax;
        this.strafeCorrection = strafeCorrection;
    }

    public void drive(double drive, double strafe, double turn) {
        strafe *= strafeCorrection;
        turn   *= turnSpeedMax;

        // Quadratic controller sensitivity.
        drive  *= Math.abs(drive);
        strafe *= Math.abs(strafe);
        turn   *= Math.abs(turn);

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Calculations based on GM0.
        double rotX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
        // Normalize.
        double max = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double fl = (rotY + rotX + turn) / max;
        double fr = (rotY - rotX - turn) / max;
        double rl = (rotY - rotX + turn) / max;
        double rr = (rotY + rotX - turn) / max;

        // Act.
        base.frontLeft.setPower(fl);
        base.frontRight.setPower(fr);
        base.rearLeft.setPower(rl);
        base.rearRight.setPower(rr);
    }

    public IMU getIMU() {
        return imu;
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
