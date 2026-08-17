#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node


class LocalToGPSNode(Node):

    EARTH_RADIUS = 6371000.0

    def __init__(self):
        super().__init__('local_to_gps_node')

        print("\n==============================================")
        print("       LOCAL COORDINATE → GPS CONVERTER")
        print("==============================================")
        print()
        print("Spreadsheet convention:")
        print("    S1 = (X=0, Y=0)")
        print("    S1 → S2 defines the +Y direction")
        print()

        # --------------------------------------------------
        # STEP 1: S1 GPS = LOCAL ORIGIN
        # --------------------------------------------------

        self.s1_lat = self.read_float(
            "Enter S1 latitude  : "
        )

        self.s1_lon = self.read_float(
            "Enter S1 longitude : "
        )

        print()

        # --------------------------------------------------
        # STEP 2: S2 GPS = LOCAL +Y DIRECTION
        # --------------------------------------------------

        self.s2_lat = self.read_float(
            "Enter S2 latitude  : "
        )

        self.s2_lon = self.read_float(
            "Enter S2 longitude : "
        )

        print()

        # --------------------------------------------------
        # Calculate orientation of local +Y
        # --------------------------------------------------

        self.local_y_bearing = self.calculate_bearing(
            self.s1_lat,
            self.s1_lon,
            self.s2_lat,
            self.s2_lon
        )

        self.get_logger().info(
            f"Local +Y bearing = "
            f"{self.local_y_bearing:.6f}° from true North"
        )

        print()
        print("----------------------------------------------")
        print("CALIBRATION")
        print("----------------------------------------------")
        print(
            f"S1 = ({self.s1_lat:.8f}, "
            f"{self.s1_lon:.8f})"
        )
        print(
            f"S2 = ({self.s2_lat:.8f}, "
            f"{self.s2_lon:.8f})"
        )
        print(
            f"Local +Y bearing = "
            f"{self.local_y_bearing:.6f}°"
        )
        print("----------------------------------------------")
        print()

        # --------------------------------------------------
        # Start interactive conversion
        # --------------------------------------------------

        self.get_logger().info(
            "Converter ready."
        )

        self.timer = self.create_timer(
            0.1,
            self.process_input
        )

        self.input_active = False

    # ======================================================
    # INPUT
    # ======================================================

    def read_float(self, prompt):

        while True:

            try:
                return float(input(prompt))

            except ValueError:

                print(
                    "Invalid number. Please enter a "
                    "valid numeric value."
                )

    # ======================================================
    # GPS BEARING
    # ======================================================

    @staticmethod
    def calculate_bearing(
        lat1,
        lon1,
        lat2,
        lon2
    ):
        """
        Calculate initial bearing from GPS point 1
        to GPS point 2.

        0°   = North
        90°  = East
        180° = South
        270° = West
        """

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)

        delta_lon = math.radians(
            lon2 - lon1
        )

        y = (
            math.sin(delta_lon)
            * math.cos(phi2)
        )

        x = (
            math.cos(phi1)
            * math.sin(phi2)
            -
            math.sin(phi1)
            * math.cos(phi2)
            * math.cos(delta_lon)
        )

        bearing = math.degrees(
            math.atan2(y, x)
        )

        return bearing % 360.0

    # ======================================================
    # LOCAL → GPS
    # ======================================================

    def local_to_gps(
        self,
        local_x,
        local_y
    ):
        """
        Convert spreadsheet local X/Y coordinates
        into latitude/longitude.

        Local coordinate definition:

            S1 = (0, 0)

            +Y = S1 → S2

            +X = 90° clockwise from +Y
        """

        # --------------------------------------------------
        # Local X/Y → East/North
        # --------------------------------------------------

        alpha = math.radians(
            self.local_y_bearing
        )

        east = (
            local_x * math.cos(alpha)
            +
            local_y * math.sin(alpha)
        )

        north = (
            -local_x * math.sin(alpha)
            +
            local_y * math.cos(alpha)
        )

        # --------------------------------------------------
        # East/North → latitude/longitude
        # --------------------------------------------------

        lat0 = math.radians(
            self.s1_lat
        )

        d_lat = (
            north /
            self.EARTH_RADIUS
        )

        d_lon = (
            east /
            (
                self.EARTH_RADIUS
                * math.cos(lat0)
            )
        )

        latitude = (
            self.s1_lat
            +
            math.degrees(d_lat)
        )

        longitude = (
            self.s1_lon
            +
            math.degrees(d_lon)
        )

        return latitude, longitude

    # ======================================================
    # INTERACTIVE LOOP
    # ======================================================

    def process_input(self):

        if self.input_active:
            return

        self.input_active = True

        try:

            print()
            print("==============================================")
            print("ENTER LOCAL COORDINATE")
            print("==============================================")
            print("Enter X/Y from your spreadsheet.")
            print("Example W1:")
            print("    X = 1.951")
            print("    Y = 9.759")
            print()
            print("Enter 'q' to quit.")
            print()

            x_input = input("Local X [m]: ").strip()

            if x_input.lower() == 'q':
                rclpy.shutdown()
                return

            y_input = input("Local Y [m]: ").strip()

            if y_input.lower() == 'q':
                rclpy.shutdown()
                return

            local_x = float(x_input)
            local_y = float(y_input)

            latitude, longitude = self.local_to_gps(
                local_x,
                local_y
            )

            print()
            print("----------------------------------------------")
            print("CONVERSION RESULT")
            print("----------------------------------------------")
            print(
                f"Local X       : "
                f"{local_x:.3f} m"
            )
            print(
                f"Local Y       : "
                f"{local_y:.3f} m"
            )
            print()
            print(
                f"Latitude      : "
                f"{latitude:.8f}"
            )
            print(
                f"Longitude     : "
                f"{longitude:.8f}"
            )
            print("----------------------------------------------")

            print()
            print("Use these in the existing planner:")
            print(
                f"Goal latitude  = "
                f"{latitude:.8f}"
            )
            print(
                f"Goal longitude = "
                f"{longitude:.8f}"
            )
            print()

        except ValueError:

            print()
            print(
                "Invalid coordinate. "
                "Please enter numbers."
            )

        except Exception as e:

            self.get_logger().error(
                f"Conversion error: {e}"
            )

        finally:

            self.input_active = False


def main(args=None):

    rclpy.init(args=args)

    node = LocalToGPSNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
