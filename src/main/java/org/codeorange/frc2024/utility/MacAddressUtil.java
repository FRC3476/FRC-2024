package org.codeorange.frc2024.utility;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;
import java.util.Enumeration;

/**
 * Utility class for getting the MAC address of the RoboRIO and determining the robot's identity.
 *
 * @author 2910 and 1678
 */
public class MacAddressUtil {
    public static final byte[] WOBBLES = new byte[]{
            (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x34, (byte) 0x8f, (byte) 0xa7
    };
    public static final byte[] JON = new byte[]{
            (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x34, (byte) 0x8f, (byte) 0xb9
    };
    public static final byte[] HALEIWA = new byte[]{
            (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x39, (byte) 0x0f, (byte) 0xc1
    };

    /**
     * Gets the MAC address of the RoboRIO.
     *
     * @return the MAC address of the RoboRIO
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
        WOBBLES,
        JON,
        HALEIWA;

        /**
         * Gets the robot identity based on the MAC address.
         *
         * @param mac the MAC address
         * @return the robot identity
         */
        public static RobotIdentity getRobotIdentity(byte[] mac) {
            if (Arrays.compare(mac, MacAddressUtil.WOBBLES) == 0) {
                return WOBBLES;
            } else if (Arrays.compare(mac, MacAddressUtil.JON) == 0) {
                return JON;
            } else if (Arrays.compare(mac, MacAddressUtil.HALEIWA) == 0) {
                return HALEIWA;
            } else {
                System.out.println("Unknown MAC Address: " + macToString(mac));
                System.out.println("Assuming Comp Bot");
                return HALEIWA;
            }
        }
    }
}
