package org.firstinspires.ftc.teamcode.mollusc.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumRobotCentric {

    public DrivetrainBaseFourWheel base;

    public double driveScaleMax = 1.0, strafeScaleMax = 1.0, turnScaleMax = 1.0;

    public MecanumRobotCentric(
        DcMotorEx frontLeft, 
        DcMotorEx frontRight, 
        DcMotorEx rearLeft, 
        DcMotorEx rearRight
    ) {
        base = new DrivetrainBaseFourWheel(frontLeft, frontRight, rearLeft, rearRight);
    }

    // Scales can be thought of as maximums, where increase is linear from 0 --> [scale] as 0 --> 1.
    public void setDriveParams(double driveScaleMax, double strafeScaleMax, double turnScaleMax) {
        driveScaleMax  = driveScaleMax;
        strafeScaleMax = strafeScaleMax;
        turnScaleMax   = turnScaleMax;
    }

    public void drive(double drive, double strafe, double turn) {
        drive  *= driveScaleMax;
        strafe *= strafeScaleMax;
        turn   *= turnScaleMax;

        // Calculations. Also prevents power values from exceeding 1.0.
        double max = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        double fl = (drive + strafe + turn) / max;
        double fr = (drive - strafe - turn) / max;
        double rl = (drive - strafe + turn) / max;
        double rr = (drive + strafe - turn) / max;

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
