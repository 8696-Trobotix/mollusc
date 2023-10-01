/*
Mecanum Robot Centric

Drivetrain hardware class.

vIX-XXX-XXIII
*/

package org.firstinspires.ftc.teamcode.mollusc.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumRobotCentric implements Drivetrain {

    public DcMotorEx frontLeft, frontRight, rearLeft, rearRight;
    public double turnSpeedMax = 0.9, strafeCorrection = 1.1;

    public MecanumRobotCentric(
        HardwareMap hardwareMap, 
        Telemetry telemetry, 
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

        if (telemetry != null) {
            telemetry.log().add("Initialized robot centric hardware.");
            telemetry.update();
        }
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

        // Calculations.
        double max = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        double fl = (drive + strafe + turn) / max;
        double fr = (drive - strafe - turn) / max;
        double rl = (drive - strafe + turn) / max;
        double rr = (drive + strafe - turn) / max;

        // Act.
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        rearLeft.setPower(rl);
        rearRight.setPower(rr);
    }

    public int[] getEncoderCounts() {
        return new int[] {
            frontLeft.getCurrentPosition(), 
            frontRight.getCurrentPosition(), 
            rearLeft.getCurrentPosition(), 
            rearRight.getCurrentPosition()
        };
    }

    public IMU getIMU() {
        return null;
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
