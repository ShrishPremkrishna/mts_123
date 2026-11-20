import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfiltfilt, welch
import pandas as pd

FS = 250  #sampling frequency (Hz)
EPOCH_LEN = 256  #samples per epoch (~512 ms)
OVERLAP = 64     #overlap for Welch
alphaEMA = 0.3

CSV_PATH_1 = "/Users/thryambakganapathy/Desktop/ThryambakFOCUSeeg_data.csv"
CSV_PATH_2 = "/Users/thryambakganapathy/Desktop/Thryambakeeg_data.csv"  # Update with your second file path

BANDS = {
    "theta": (4, 8),
    "alpha": (8, 12),
    "beta":  (12, 16)
}

#bandpass filter, 4-30 Hz
def get_filter():
    return butter(
        N=2,
        Wn=[4, 30],
        btype="bandpass",
        fs=FS,
        output="sos"
    )

#calculate bandpower given Welch's FFT + Power Spectral Density
def bandpower(psd, freqs, fmin, fmax):
    mask = (freqs >= fmin) & (freqs <= fmax)
    df = freqs[1] - freqs[0]
    return np.sum(psd[:, mask], axis=1) * df  # per channel

#we need an exponential moving average to smooth our data --> elucidate general trend of bandpower values rather than getting jerky data
def applyEMA(data, alphaValue):
  smoothedData = np.zeros_like(data)
  smoothedData[0] = data[0] #initialize starting point

  #we're smoothing the data by applying some empirically-determined scalar to the next piece of data, then adding the product of the scalar's complement and the previous smoothed data
  for i in range(1, len(data)):
    smoothedData[i] = (data[i] * alphaValue) + (smoothedData[i-1] * (1-alphaValue))

  return smoothedData

def process_file(csv_path, file_label):
    global alphaEMA
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    data = data.T  # shape -> (channels, samples)

    sos = get_filter()
    filtered = sosfiltfilt(sos, data, axis=1)
    
    #epoch segmentation
    num_samples = filtered.shape[1]
    step = EPOCH_LEN
    metrics = {
        "theta_power": [],
        "alpha_power": [],
        "beta_power": [],
        "alpha+beta": [],
        "engagement_index": [],  # beta / (alpha + theta)
    }
    
    for start in range(0, num_samples - EPOCH_LEN, step):
        epoch = filtered[:, start:start+EPOCH_LEN]

        freqs, psd = welch(
            epoch,
            fs=FS,
            window="hann",
            nperseg=EPOCH_LEN,
            noverlap=OVERLAP,
            axis=1
        )

        #calculate bandpowers + metrics
        theta = bandpower(psd, freqs, *BANDS["theta"])
        alpha = bandpower(psd, freqs, *BANDS["alpha"])
        beta  = bandpower(psd, freqs, *BANDS["beta"])

        theta_mean = np.mean(theta)
        alpha_mean = np.mean(alpha)
        beta_mean  = np.mean(beta)

        metrics["theta_power"].append(theta_mean)
        metrics["alpha_power"].append(alpha_mean)
        metrics["beta_power"].append(beta_mean)
        metrics["alpha+beta"].append((alpha_mean + beta_mean) / 2)
        metrics["engagement_index"].append(beta_mean / (alpha_mean + theta_mean + 1e-8))

    for k in metrics:
        metrics[k] = np.array(metrics[k])

    skip_initial = 2  #skip first 3 epochs (edge effects from filtering)
    for k in metrics:
        metrics[k] = metrics[k][skip_initial:]

    #clip to 99th percentile to remove extreme outliers
    for k in metrics:
        if k in ["theta_power", "alpha_power", "beta_power", "alpha+beta"]:
            # Clip bandpower values to 100
            metrics[k] = np.clip(metrics[k], 0, 100)
        else:
            upper_limit = np.percentile(metrics[k], 99)
            metrics[k] = np.clip(metrics[k], 0, upper_limit)

    #apply EMA to each data array in metrics
    for k in metrics:
        metrics[k] = applyEMA(metrics[k], alphaEMA)
    
    return metrics

metrics_file1 = process_file(CSV_PATH_1, "File 1")
metrics_file2 = process_file(CSV_PATH_2, "File 2")

#plotting
fig, axes = plt.subplots(len(metrics_file1), 2, figsize=(16, 10))

metric_names = list(metrics_file1.keys())

for i, metric_name in enumerate(metric_names):
    #file 1 - left column
    axes[i, 0].plot(metrics_file1[metric_name], color='blue')
    axes[i, 0].set_title(f"File 1 - {metric_name.replace('_', ' ').title()}")
    axes[i, 0].set_ylabel("Power / Ratio")
    axes[i, 0].grid(True)
    
    #file 2 - right column
    axes[i, 1].plot(metrics_file2[metric_name], color='red')
    axes[i, 1].set_title(f"File 2 - {metric_name.replace('_', ' ').title()}")
    axes[i, 1].set_ylabel("Power / Ratio")
    axes[i, 1].grid(True)

# Add x-label to bottom plots
axes[-1, 0].set_xlabel("Epoch")
axes[-1, 1].set_xlabel("Epoch")

plt.tight_layout()
plt.show()