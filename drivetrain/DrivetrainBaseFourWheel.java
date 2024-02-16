package org.firstinspires.ftc.teamcode.mollusc.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DrivetrainBaseFourWheel {

    public DcMotorEx frontLeft, frontRight, rearLeft, rearRight;

    public DrivetrainBaseFourWheel(
        DcMotorEx frontLeft, 
        DcMotorEx frontRight, 
        DcMotorEx rearLeft, 
        DcMotorEx rearRight
    ) {
        this.frontLeft  = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft   = rearLeft;
        this.rearRight  = rearRight;
    }

    // Reset encoder counts to zero and set run mode to by power.
    public void zeroEncoders() {
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    // Order: `frontLeft`, `frontRight`, `rearLeft`, `rearRight`
    public int[] getEncoderCounts() {
        return new int[] {
            frontLeft.getCurrentPosition(), 
            frontRight.getCurrentPosition(), 
            rearLeft.getCurrentPosition(), 
            rearRight.getCurrentPosition()
        };
    }

    public void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
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
