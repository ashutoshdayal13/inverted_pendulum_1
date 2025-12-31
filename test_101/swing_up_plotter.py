import pandas as pd
import matplotlib.pyplot as plt

try:
    # Load Data
    df = pd.read_csv('swingup_robust_log.csv')

    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # Plot 1: Phase Space (Angle vs Velocity) - Crucial for Energy Analysis
    # We plot Time vs Angle here for simplicity, but I need to see the "homoclinic orbit"
    axs[0].plot(df['t'], df['theta'], label='Angle (deg)', color='blue')
    axs[0].axhline(0, color='red', linestyle='--', label='Upright Target')
    axs[0].axhline(180, color='gray', linestyle=':', alpha=0.5)
    axs[0].axhline(-180, color='gray', linestyle=':', alpha=0.5)
    axs[0].set_ylabel('Angle (deg)')
    axs[0].set_title('Pendulum Swing Up')
    axs[0].legend()
    axs[0].grid(True)

    # Plot 2: Energy Error & Control
    # If Energy Error doesn't go to zero, K_SWING is wrong.
    ax2 = axs[1]
    ax2.plot(df['t'], df['E_err'], label='Energy Error (J)', color='purple')
    ax2.axhline(0, color='black', linestyle='--')
    ax2.set_ylabel('Energy Error', color='purple')
    
    # Twin axis for current to see correlation
    ax2b = ax2.twinx()
    ax2b.plot(df['t'], df['current'], label='Motor Current (A)', color='orange', alpha=0.5)
    ax2b.set_ylabel('Current (A)', color='orange')
    axs[1].set_title('Energy Error & Control Effort')
    axs[1].grid(True)

    # Plot 3: Cart Position (Rail Usage)
    axs[2].plot(df['t'], df['x'], label='Cart Position (m)', color='green')
    axs[2].set_ylabel('Position (m)')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_title('Rail Usage')
    axs[2].grid(True)

    plt.tight_layout()
    plt.savefig('swingup_analysis.png')
    print("Plots saved to 'swingup_analysis.png'")
    plt.show()

except Exception as e:
    print(f"Could not plot: {e}")
    print("Ensure pandas and matplotlib are installed: pip install pandas matplotlib")