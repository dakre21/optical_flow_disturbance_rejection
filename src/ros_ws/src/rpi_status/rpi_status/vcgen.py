import subprocess
import time


def run_command(cmd):
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout.strip().split("=")[-1]
    except Exception as e:
        return "-1"


if __name__ == "__main__":
    while True:
        throttled = run_command("vcgencmd get_throttled")
        temperature = run_command("vcgencmd measure_temp")
        core_voltage = run_command("vcgencmd measure_volts")

        print(
            f"Status throttled: {throttled}, temperature: {temperature}, core_voltage: {core_voltage}"
        )

        time.sleep(1)
