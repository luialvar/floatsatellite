#!/usr/bin/env python3
"""Back-of-the-envelope PID gain estimates from measured inertia values."""

INERTIA_KG_M2 = {
    "roll": 0.00547,
    "pitch": 0.00541,
    "yaw": 0.00685,
}

NATURAL_FREQUENCY_RAD_S = 3.0
DAMPING_RATIO = 0.8


def main():
    print("FloatSat PID gain estimate")
    print(f"wn={NATURAL_FREQUENCY_RAD_S:.2f} rad/s, zeta={DAMPING_RATIO:.2f}")

    for axis, inertia in INERTIA_KG_M2.items():
        kp_velocity = 2.0 * DAMPING_RATIO * NATURAL_FREQUENCY_RAD_S * inertia
        kd_velocity = inertia
        kp_position = NATURAL_FREQUENCY_RAD_S**2 * inertia
        kd_position = kp_velocity

        print(f"\n{axis.upper()}")
        print(f"  velocity loop: Kp={kp_velocity:.5f}, Ki=0.00000, Kd={kd_velocity:.5f}")
        print(f"  position loop: Kp={kp_position:.5f}, Ki=0.00000, Kd={kd_position:.5f}")


if __name__ == "__main__":
    main()
