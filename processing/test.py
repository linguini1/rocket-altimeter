import math
import sys
import matplotlib.pyplot as plt

import csv

SEA_PRESSURE: float = 1013.25
GAS_CONSTANT: float = 8.31432
GRAVITY: float = 9.80665
MOLAR_MASS: float = 0.0289644
CELSIUS_TO_KELVIN: float = 273.0
TIMESTEP: float = 70  # Milliseconds


def alt_from_baro(pressure: float, temp: float) -> float:
    return (
        -(GAS_CONSTANT * (CELSIUS_TO_KELVIN + temp))
        / (MOLAR_MASS * GRAVITY)
        * math.log(pressure / SEA_PRESSURE)
    )


def moving_average(old: float, val: float, alpha: float) -> float:
    """Calculates the moving average."""
    return (alpha * old) + (1 - alpha) * val


def gen_avg(noisy: list[float], alpha: float, init: float) -> list[float]:
    """Generates the moving-averaged data-set from a noisy data set."""

    # Calculate a moving average and save the averaged value at each time step
    cur_avg = init
    averages: list[float] = []
    for v in noisy:
        cur_avg = moving_average(cur_avg, v, alpha)
        averages.append(cur_avg)

    return averages


def plot_data(plt, data: dict[str, list[float]], name: str, unit: str):
    plt.plot(data["time"], data[name], label=name)
    plt.title(f"{name} vs time")
    plt.ylabel(f"Value ({unit})")
    plt.xlabel("Time (ms)")
    plt.legend()


def main() -> None:

    # Open CSV and read in data

    rocket_data: dict[str, list[float]] = dict()

    with open(sys.argv[1], "r") as file:
        reader = csv.reader(file)

        next(reader)  # Skip interval
        headers = next(reader)  # Headers

        # Populate lists
        for header in headers:
            rocket_data[header] = []

        # Put raw data in the lists
        for line in reader:
            for i, value in enumerate(line):
                rocket_data[headers[i]].append(float(value))

    # Now we have the raw data, turn it into measurements

    rocket_data["altitude"] = []
    for p, t in zip(rocket_data["pressure"], rocket_data["temperature"]):
        rocket_data["altitude"].append(alt_from_baro(p, t))

    rocket_data["height"] = [
        a - rocket_data["altitude"][0] for a in rocket_data["altitude"]
    ]

    rocket_data["velocity"] = []
    last_alt = rocket_data["altitude"][0]
    for alt in rocket_data["altitude"]:
        rocket_data["velocity"].append((alt - last_alt) / (TIMESTEP / 1000))
        last_alt = alt

    rocket_data["time"] = []
    t = 0
    for _ in range(len(rocket_data["pressure"])):
        rocket_data["time"].append(t)
        t += TIMESTEP

    # Low-pass filtered data
    rocket_data["height-filtered"] = gen_avg(rocket_data["height"], 0.95, 0)
    rocket_data["altitude-filtered"] = gen_avg(rocket_data["altitude"], 0.95, 0)
    rocket_data["velocity-filtered"] = gen_avg(rocket_data["velocity"], 0.95, 0)

    # Velocity made from low-pass filtered data
    rocket_data["velocity-from-filtered"] = []
    last_alt = rocket_data["altitude-filtered"][0]
    for alt in rocket_data["altitude-filtered"]:
        rocket_data["velocity-from-filtered"].append(
            (alt - last_alt) / (TIMESTEP / 1000)
        )
        last_alt = alt

    # plot_data(plt, data=rocket_data, name="height", unit="meters")
    plot_data(plt, data=rocket_data, name="height-filtered", unit="meters")
    plot_data(plt, data=rocket_data, name="velocity", unit="meters/second")
    plot_data(
        plt, data=rocket_data, name="velocity-filtered", unit="meters/second"
    )
    plot_data(
        plt,
        data=rocket_data,
        name="velocity-from-filtered",
        unit="meters/second",
    )
    plt.show()
    return


if __name__ == "__main__":
    main()
