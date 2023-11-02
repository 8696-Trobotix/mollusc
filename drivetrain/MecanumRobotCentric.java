/*
Mecanum Robot Centric

Drivetrain hardware class.

vXI-II-XXIII
*/

package org.firstinspires.ftc.teamcode.mollusc.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumRobotCentric implements Drivetrain {

    public DrivetrainBaseFourWheel base;
    public double strafeCorrection = 1.0;

    public MecanumRobotCentric(
        HardwareMap hardwareMap, 
        Telemetry telemetry, 
        String fl, DcMotorEx.Direction fld, 
        String fr, DcMotorEx.Direction frd, 
        String rl, DcMotorEx.Direction rld, 
        String rr, DcMotorEx.Direction rrd
    ) {
        base = new DrivetrainBaseFourWheel(hardwareMap, fl, fld, fr, frd, rl, rld, rr, rrd);

        if (telemetry != null) {
            telemetry.log().add("Initialized robot centric hardware.");
            telemetry.update();
        }
    }

    public void setDriveParams(double drivePowerMax, double turnPowerMax, double strafeCorrection) {
        base.drivePowerMax    = drivePowerMax;
        base.turnPowerMax     = turnPowerMax;
        this.strafeCorrection = strafeCorrection;
    }

    public void drive(double drive, double strafe, double turn) {
        // Quadratic controller sensitivity.
        drive  *= Math.abs(drive);
        strafe *= Math.abs(strafe);
        turn   *= Math.abs(turn);

        drive  *= base.drivePowerMax;
        turn   *= base.turnPowerMax;
        strafe *= strafeCorrection;

        // Calculations.
        double max = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        double fl = (drive + strafe + turn) / max;
        double fr = (drive - strafe - turn) / max;
        double rl = (drive - strafe + turn) / max;
        double rr = (drive + strafe - turn) / max;

        // Act.
        base.frontLeft.setPower(fl);
        base.frontRight.setPower(fr);
        base.rearLeft.setPower(rl);
        base.rearRight.setPower(rr);
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
