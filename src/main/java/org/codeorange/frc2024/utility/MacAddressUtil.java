package org.codeorange.frc2024.utility;

import org.codeorange.frc2024.robot.Robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;
import java.util.Enumeration;

/**
 * Utility class for getting the MAC address of the robot radio and determining the robot's identity.
 *
 * @author 2910 & 1678
 */
public class MacAddressUtil {
    public static final byte[] PROTOTYPE_BOT = new byte[]{
            (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x34, (byte) 0x8f, (byte) 0xa7
    };
    public static final byte[] PRACTICE_BOT = new byte[]{
            (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00
    };
    public static final byte[] COMPETITION_BOT = new byte[]{
            (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00
    };

    /**
     * Gets the MAC address of the robot's radio.
     *
     * @return the MAC address of the robot's radio
     * @throws SocketException if no MAC address is found
     */
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

    /**
     * Converts a raw MAC address byte array to a string.
     *
     * @param mac the MAC address byte array
     * @return the MAC address as a string
     */
    public static String macToString(byte[] mac) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < mac.length; i++) {
            sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
        }
        return sb.toString();
    }

    /**
     * Enum representing the possible robot identities.
     */
    public enum RobotIdentity {
        PROTOTYPE_BOT,
        PRACTICE_BOT,
        COMPETITION_BOT;

        /**
         * Gets the robot identity based on the MAC address.
         *
         * @param mac the MAC address
         * @return the robot identity
         */
        public static RobotIdentity getRobotIdentity(byte[] mac) {
            if (Arrays.compare(mac, MacAddressUtil.PROTOTYPE_BOT) == 0) {
                return PROTOTYPE_BOT;
            } else if (Arrays.compare(mac, MacAddressUtil.PRACTICE_BOT) == 0) {
                return PRACTICE_BOT;
            } else if (Arrays.compare(mac, MacAddressUtil.COMPETITION_BOT) == 0) {
                return COMPETITION_BOT;
            } else {
                System.out.println("Unknown MAC Address: " + macToString(mac));
                System.out.println("Assuming Prototype Bot");
                return PROTOTYPE_BOT; // assume prototype bot for now, change this to comp later
            }
        }
    }
}
