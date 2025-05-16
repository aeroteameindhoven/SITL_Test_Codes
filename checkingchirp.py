import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import chirp, spectrogram

# Chirp parameters
duration = 50
sampling_rate = 0.005
t_chirp = np.linspace(0, duration, int(duration / sampling_rate))

# Generate chirp signal
x_lin = 15 * 5 * chirp(t_chirp, f0=0, f1=100, t1=duration, method='linear')

# Optional: compute instantaneous frequency (for plotting)
instantaneous_freq = np.linspace(0, 100, len(t_chirp))

# Plot
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(t_chirp, x_lin, label='Linear Chirp Signal')
plt.title(r"Linear Chirp from $f(0)=0\,\mathrm{Hz}$ to $f(50)=100\,\mathrm{Hz}$")
plt.ylabel(r"Amplitude $x_{\mathrm{lin}}(t)$")
plt.grid(True)
plt.legend()


plt.tight_layout()
plt.show()
