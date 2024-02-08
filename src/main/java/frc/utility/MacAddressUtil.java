package frc.utility;

import frc.robot.Robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;
import java.util.Enumeration;

public class MacAddressUtil {
    //TODO: fill out Mac Addresses
    public static final byte[] PROTOTYPE_BOT = new byte[]{
            (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00
    };
    public static final byte[] PRACTICE_BOT = new byte[]{
            (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00
    };
    public static final byte[] COMPETITION_BOT = new byte[]{
            (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00
    };

    public static byte[] getMacAddress() throws SocketException {
        Enumeration<NetworkInterface> networkInterface = NetworkInterface.getNetworkInterfaces();

        NetworkInterface temp;

        while (networkInterface.hasMoreElements()) {
            temp = networkInterface.nextElement();

            byte[] mac = temp.getHardwareAddress();
            if (mac != null) {
                return mac;
            }
        }
        return null;
    }

    public static String macToString(byte[] mac) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < mac.length; i++) {
            sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
        }
        return sb.toString();
    }

    public enum RobotIdentity {
        PROTOTYPE_BOT,
        PRACTICE_BOT,
        COMPETITION_BOT;

        public static RobotIdentity getRobotIdentity(byte[] mac) {
            if (Arrays.compare(mac, MacAddressUtil.PROTOTYPE_BOT) == 0) {
                return PROTOTYPE_BOT;
            } else if (Arrays.compare(mac, MacAddressUtil.PRACTICE_BOT) == 0) {
                return PRACTICE_BOT;
            } else if (Arrays.compare(mac, MacAddressUtil.COMPETITION_BOT) == 0) {
                return COMPETITION_BOT;
            } else {
                System.out.println("Unknown MAC Address: " + macToString(mac));
                return PROTOTYPE_BOT; // assume prototype bot for now, change this to comp later
            }
        }
    }
}
