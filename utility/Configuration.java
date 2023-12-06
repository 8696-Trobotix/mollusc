package org.firstinspires.ftc.teamcode.mollusc.utility;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.HashMap;

public class Configuration {

    private OpMode opMode;

    private HashMap<String, String> configData = new HashMap<>();

    public Configuration() {
        this(Mollusc.opMode, "config.txt")
    }
    public Configuration(OpMode opMode, String configFileName) {
        this.opmode = opMode;

        Asset asset = new Asset(opMode.hardwareMap.appContext, "mollusc/" + configFileName);
        String[] lines = asset.getLines();

        for (String line : lines) {
            line = line.trim();
            if (line.isEmpty() || line.startsWith("//")) {
                continue;
            }
            String[] parsed = line.split(":");
            String key = parsed[0].trim();
            String value = parsed.length > 1 ? parsed[1].trim() : null;
            configData.put(key, value);
        }
    }

    public HashMap<String, String> getConfigData() {
        return configData;
    }
    public boolean containsKey(String key) {
        return configData.containsKey(key);
    }
    // Returns true if `key` is present but maps to null.
    public boolean noValue(key) {
        return containsKey() ? get(key) == null : false;
    }
    public String get(String key) {
        return configData.get(key);
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
