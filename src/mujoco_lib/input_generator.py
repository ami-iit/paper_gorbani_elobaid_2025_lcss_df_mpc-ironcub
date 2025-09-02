import numpy as np
import matplotlib.pyplot as plt

def turb_identSignal_may2024(t):
    """
    Generates the input signals for identifying and validating the jetcat turbine model.

    Args:
        t (float): Time.

    Returns:
        float: Input signal 'u' which belongs to the set [0, 100].
    """

    u = 0

    # Define signal handles and their corresponding duration
    signals = [
        # (function(t, t_start, duration, parameters), duration)

        ## DYNAMIC SIGNAL SEQUENCE

        # sinusoid
        (lambda t, ts, dur: rampSignal(t, ts, dur, 0, 100), 30),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 50), 5),
        (lambda t, ts, dur: chirpSignal(t, ts, dur, 50, 50, 0.01, 0.08), 50),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 50), 5),
        (lambda t, ts, dur: rampSignal(t, ts, dur, 50, 0), 5),
        (lambda t, ts, dur: idleSignal(t, ts, dur), 10),
        # chirps
        (lambda t, ts, dur: rampSignal(t, ts, dur, 0, 90), 5),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 90), 5),
        (lambda t, ts, dur: chirpSignal(t, ts, dur, 90, 10, 0.08, 0.6), 60),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 90), 5),
        (lambda t, ts, dur: rampSignal(t, ts, dur, 90, 70), 3),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 70), 5),
        (lambda t, ts, dur: chirpSignal(t, ts, dur, 70, 10, 0.08, 0.6), 60),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 70), 5),
        (lambda t, ts, dur: rampSignal(t, ts, dur, 70, 50), 3),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 50), 5),
        (lambda t, ts, dur: chirpSignal(t, ts, dur, 50, 10, 0.08, 0.6), 60),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 50), 4),
        (lambda t, ts, dur: rampSignal(t, ts, dur, 50, 30), 3),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 30), 5),
        (lambda t, ts, dur: chirpSignal(t, ts, dur, 30, 10, 0.08, 0.6), 60),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 30), 4),
        (lambda t, ts, dur: rampSignal(t, ts, dur, 30, 10), 3),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 10), 5),
        (lambda t, ts, dur: chirpSignal(t, ts, dur, 10, 10, 0.08, 0.6), 60),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 10), 4),
        (lambda t, ts, dur: rampSignal(t, ts, dur, 10, 0), 3),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 10), 5),
        (lambda t, ts, dur: chirpSignal(t, ts, dur, 10, 10, 0.08, 0.6), 60),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 10), 4),
        (lambda t, ts, dur: rampSignal(t, ts, dur, 10, 50), 3),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 50), 5),
        (lambda t, ts, dur: chirpSignal(t, ts, dur, 50, 10, 0.1, 50), 60),
        (lambda t, ts, dur: stepSignal(t, ts, dur, 50), 4),
        (lambda t, ts, dur: rampSignal(t, ts, dur, 50, 0), 3),
    ]

    # Initialize u as 0
    u = 0

    # function start time
    start_time = 0

    # Iterate over each signal
    for i in range(len(signals)):
        duration = signals[i][1]
        # Check if t falls within the time interval of the current signal
        if start_time <= t < start_time + duration:
            # Evaluate the corresponding signal
            u = signals[i][0](t, start_time, duration)
            # No need to check further, exit the loop
            break
        start_time += duration

    if u < 0:
        u = 0
    elif u > 100:
        u = 100

    return u


def idleSignal(current_time, start_time, duration):
    """Sets throttle to 0%."""
    u = 0
    return u


def stepSignal(current_time, start_time, duration, u_des):
    """Sets constant throttle."""
    u = u_des
    return u


def rampSignal(current_time, start_time, duration, u_start, u_end):
    """Generates a ramp signal."""
    u = u_start + (current_time - start_time) * (u_end - u_start) / duration
    return u


def chirpSignal(current_time, start_time, duration, u_mean, delta, f_min, f_max):
    """Generates a chirp signal."""
    freq = f_min + (current_time - start_time) * (f_max - f_min) / duration
    u = u_mean + delta * np.sin(2 * np.pi * freq * (current_time - start_time))
    return u


def validationChirpSignal(current_time, start_time, duration, u_mean, delta, f_min, f_max):
    """Generates a validation chirp signal."""
    t = (current_time - start_time) / duration
    u_mean_1 = u_mean / 85 * (
        -0.00265625 * (current_time - start_time)**3
        + 0.15925 * (current_time - start_time)**2
        + 0.005 * (current_time - start_time)
    )
    u_mean_2 = u_mean / 85 * (
        +0.00265625 * (current_time - start_time)**3
        - 0.6375 * (current_time - start_time)**2
        + 47.8125 * (current_time - start_time)
        - 1062.5
    )
    freq1 = f_min + (current_time - start_time) * (f_max - f_min) / duration
    freq2 = f_max - (current_time - start_time) * (f_max - f_min) / duration
    u = 0
    if t < 0.4:
        u = u_mean_1 + delta * np.sin(2 * np.pi * freq1 * (current_time - start_time)) + delta * np.sin(2 * np.pi * freq2 * (current_time - start_time))
    elif 0.4 <= t < 0.6:
        u = u_mean + delta * np.sin(2 * np.pi * freq1 * (current_time - start_time)) + delta * np.sin(2 * np.pi * freq2 * (current_time - start_time))
    elif t >= 0.6:
        u = u_mean_2 + delta * np.sin(2 * np.pi * freq1 * (current_time - start_time)) + delta * np.sin(2 * np.pi * freq2 * (current_time - start_time))
    return u

if __name__ == '__main__':
    # Example Usage with Sampling Period

    # 1. Define Sampling Period (Ts)
    Ts = 0.01  # Sampling period of 0.01 seconds (100 Hz sampling frequency)
    fs = 1/Ts

    # 2. Create Time Vector
    total_duration = 30  # Total duration of the signal (as before)
    time_points = np.arange(0, total_duration, Ts)  # Create time points with the defined sampling period.

    # 3. Sample the Signal
    input_signals = [turb_identSignal_may2024(t) for t in time_points]

    # You can then plot the input signal or use it in further analysis

    plt.plot(time_points, input_signals)
    plt.xlabel('Time (s)')
    plt.ylabel('Input Signal (u)')
    plt.title(f'Turbine Input Signal (Sampling Frequency = {fs} Hz)') # Added sampling freq to title
    plt.grid(True)
    plt.show()