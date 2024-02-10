package frc.subsystem;

import edu.wpi.first.wpilibj.Filesystem;
import frc.subsystem.shooter.Shooter;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class Superstructure extends AbstractSubsystem {
    public Superstructure() {
        super();
    }

    private String path = Filesystem.getDeployDirectory().getPath(); // need to append exact location of CSV file
    HashMap<String, ShooterConfiguration> map = new HashMap<String, ShooterConfiguration>();

    public void toHashMap() {


        String line = "";
        String splitBy = ",";
        try {
            BufferedReader br = new BufferedReader(new FileReader(path));
            br.readLine();
            while ((line = br.readLine()) != null) {
                String[] fields = line.split(splitBy);

                String distanceDirectionHeight = fields[0];
                String location = fields[1];
                double shooterAngle = Double.parseDouble(fields[2]);
                double shooterVelocity = Double.parseDouble(fields[3]);

                ShooterConfiguration shooterConfiguration = new ShooterConfiguration(location, shooterAngle, shooterVelocity);
                map.put(distanceDirectionHeight, shooterConfiguration);

            }

            br.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }



    private String convertToKey(double distance, String direction, String height) {
        return distance + direction + height;

    }

    public String getLocationOfRobot(double distance, String direction, String height){
        return map.get(convertToKey(distance, direction, height)).getLocation();
    }
    public double getShooterAngle(double distance, String direction, String height){
        return map.get(convertToKey(distance, direction, height)).getShooterVelocity();
    }
    public double getShooterVelocity(double distance, String direction, String height){
        return map.get(convertToKey(distance, direction, height)).getShooterAngle();
    }

    static class ShooterConfiguration {
        private String location;
        private double shooterAngle;
        private double shooterVelocity;

        public ShooterConfiguration(String location, double shooterAngle, double shooterVelocity) {
            this.location = location;
            this.shooterAngle = shooterAngle;
            this.shooterVelocity = shooterVelocity;
        }

        public String getLocation() {
            return location;
        }

        public double getShooterAngle() {
            return shooterAngle;
        }

        public double getShooterVelocity() {
            return shooterVelocity;
        }

    }
}