/*
From GM0:

    The most common method for tuning a PID controller is as follows:

        Set the I and D gains to zero

        Increase the P gain until there are oscillations around the target

        Increase the D gain until no overshoot occurs

        If there is steady state error, increase the I gain until it is corrected

From CTRL ALT FTC.

    For FTC motor control I recommend making it so that your integralSumLimit * Ki is around ~0.25.

Remember to reset when reference changes.
*/

package org.firstinspires.ftc.teamcode.mollusc.utility;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

public class PIDF {

    public double Kp, Ki, Kd, Kf, integralLimit, magnitude;

    private double errorPrev = 0, integral = 0, t = 0;

    private ElapsedTime runtime = new ElapsedTime();
    public Filter.LowPass filter = new Filter.LowPass(0, 0.8);

    public PIDF(double Kp, double Ki, double Kd, double Kf, double integralLimit, double magnitude) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.integralLimit = integralLimit;
        this.magnitude = magnitude;
    }

    public PIDF(PIDF reference) {
        this.Kp = reference.Kp;
        this.Ki = reference.Ki;
        this.Kd = reference.Kd;
        this.Kf = reference.Kf;
        this.integralLimit = reference.integralLimit;
        this.magnitude = reference.magnitude;
    }

    public double out(double error) {
        double seconds = runtime.seconds();
        double dt = seconds - t;

        integral = Range.clip(integral + error * dt, -integralLimit, integralLimit);
        double derivative = filter.out(error - errorPrev) / dt;

        errorPrev = error;
        t = seconds;

        double ret = Kp * error + Ki * integral + Kd * derivative;
        return Range.clip(ret + Math.signum(ret) * Kf, -magnitude, magnitude);
    }

    public void restart() {
        integral = 0;
        t = runtime.seconds();
    }

    public static void bulkMode() {
        List<LynxModule> allHubs = Mollusc.instance().hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            module.clearBulkCache();
        }
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
