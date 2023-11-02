package org.firstinspires.ftc.teamcode.mollusc.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DrivetrainBaseFourWheel {

    public DcMotorEx frontLeft, frontRight, rearLeft, rearRight;

    public double drivePowerMax = 1, turnPowerMax = 1;

    public DrivetrainBase(
        HardwareMap hardwareMap, 
        String fl, DcMotorEx.Direction fld, 
        String fr, DcMotorEx.Direction frd, 
        String rl, DcMotorEx.Direction rld, 
        String rr, DcMotorEx.Direction rrd
    ) {
        // Connect motors.
        frontLeft  = hardwareMap.get(DcMotorEx.class, fl);
        frontRight = hardwareMap.get(DcMotorEx.class, fr);
        rearLeft   = hardwareMap.get(DcMotorEx.class, rl);
        rearRight  = hardwareMap.get(DcMotorEx.class, rr);

        // Set motor directions.
        frontLeft.setDirection(fld);
        frontRight.setDirection(frd);
        rearLeft.setDirection(rld);
        rearRight.setDirection(rrd);

        // Set motors to brake.
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void zeroEncoders() {
        // Reset encoder counts to zero and set run mode to by power.

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int[] getEncoderCounts() {
        return new int[] {
            frontLeft.getCurrentPosition(), 
            frontRight.getCurrentPosition(), 
            rearLeft.getCurrentPosition(), 
            rearRight.getCurrentPosition()
        };
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
