import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Allan variance calculation function
def allan_variance(data, sample_rate):
    n = len(data)
    max_num_of_clusters = int(np.floor(n / 2))
    tau = np.array([sample_rate * 2**i for i in range(int(np.log2(max_num_of_clusters)) + 1)])
    allan_variances = np.zeros(len(tau))
    
    for i, t in enumerate(tau):
        m = int(t * sample_rate)
        cluster_means = np.array([np.mean(data[j:j + m]) for j in range(0, n - m, m)])
        allan_variances[i] = 0.5 * np.mean(np.diff(cluster_means) ** 2)
    
    return tau, allan_variances

# Load the CSV file into a DataFrame
data = pd.read_csv('imu_data.csv')

# Define the columns you want to process (4th to 8th in MATLAB corresponds to 5th to 9th in Python)
columns_to_process = data.columns[5:11]

# Iterate over each column and compute Allan variance
for col in columns_to_process:
    # Extract data for the current column
    current_data = data[col].to_numpy()

    # Perform Allan variance calculation (using 40 Hz sampling rate)
    tau, avar_values = allan_variance(current_data, sample_rate=40)
    adev = np.sqrt(avar_values)

    # Noise parameter - slope for random walk (-0.5 slope)
    slope = -0.5
    log_tau = np.log10(tau)
    log_adev = np.log10(adev)
    dlog_adev = np.diff(log_adev) / np.diff(log_tau)
    i = np.argmin(np.abs(dlog_adev - slope))

    # Find the y-intercept of the line
    b = log_adev[i] - slope * log_tau[i]
    logN = slope * np.log10(1) + b
    N = 10 ** logN
    tauN = 1
    lineN = N / np.sqrt(tau)

    # Determine rate random walk (slope -0.5)
    logK = slope * np.log10(3) + b
    K = 10 ** logK
    tauK = 3
    lineK = K * np.sqrt(tau / 3)

    # Slope for bias instability (0 slope)
    slope = 0
    dlog_adev = np.diff(log_adev) / np.diff(log_tau)
    i = np.argmin(np.abs(dlog_adev - slope))

    # Find the y-intercept for bias instability
    b = log_adev[i] - slope * log_tau[i]

    # Bias instability coefficient
    scfB = np.sqrt(2 * np.log(2) / np.pi)
    logB = b - np.log10(scfB)
    B = 10 ** logB
    tauB = tau[i]
    lineB = B * scfB * np.ones_like(tau)

    # Plot the results for the current column
    tau_params = [tauN, tauK, tauB]
    params = [N, K, scfB * B]

    plt.figure()
    plt.loglog(tau, adev, label=r'$\sigma (\tau)$')
    plt.loglog(tau, lineN, '--', label=r'$\sigma_N$ (Random Walk)')
    plt.loglog(tau, lineK, '--', label=r'$\sigma_K$ (Rate Random Walk)')
    plt.loglog(tau, lineB, '--', label=r'$\sigma_B$ (Bias Instability)')
    plt.plot(tau_params, params, 'o')
    plt.title('Allan Deviation with Noise Parameters for ' + col)
    plt.xlabel(r'$\tau$ (s)')
    plt.ylabel(r'$\sigma(\tau)$')
    plt.legend()
    plt.grid(True)
    plt.show()

