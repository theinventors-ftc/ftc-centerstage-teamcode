package org.firstinspires.ftc.teamcode;

import org.yaml.snakeyaml.Yaml;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.HashMap;

public class yaml_test {
    private static HashMap load_constants() {
        Yaml yaml = new Yaml();

        InputStream inputStream = null;

        try {
            inputStream = new FileInputStream("C:\\Users\\user\\Dev\\Teams\\TheInventors\\robot-teamcodes\\2023-24\\ftc-centerstage-teamcode\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode");
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        return yaml.load(inputStream);
    }

    public static void main(String[] args) {
        HashMap constants = load_constants();
//        for (Object o : constants.entrySet()) {
//            System.out.println(o);
//        }
//        System.out.println("Root filder is: " + System.getProperty("user.dir"));
    }
}

