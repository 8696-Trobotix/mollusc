package org.firstinspires.ftc.teamcode.mollusc.auto;

import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;
import org.firstinspires.ftc.teamcode.mollusc.auto.odometry.Pose;

public interface Auto {

    public static double TIMEOUT = 5.0;
    public static double STATIC_TIMOUT_MILLISECONDS = 500.0;

    public void driveTo(Pose newPose) throws ParityException;
    public double[] drivePowers(Pose newPose);
    public void register() throws ParityException;
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
