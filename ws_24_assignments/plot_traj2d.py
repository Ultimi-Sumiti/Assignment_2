import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('out.csv')

tag1 = [-0.03, -0.61]
tag10 = [0.57, -0.01]

plt.figure(figsize=(10, 8))

plt.plot(df['x'], df['y'], color='gray', alpha=0.5, label='Trajectory Path', linewidth=1.5)

plt.scatter(df['x'].iloc[0], df['y'].iloc[0], color='green', s=80, label='Start')
plt.scatter(df['x'].iloc[-1], df['y'].iloc[-1], color='green', s=80, label='End')

plt.scatter(tag1[0], tag1[1], color='red', s=120, marker='X', label='tag1')
plt.scatter(tag10[0], tag10[1], color='blue', s=120, marker='X', label='tag10')

plt.text(tag1[0] + 0.02, tag1[1], "tag1", color='red', fontweight='bold')
plt.text(tag10[0] + 0.02, tag10[1], "tag10", color='blue', fontweight='bold')

plt.title('2D Trajectory View (XY Plane)')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()
plt.axis('equal')
plt.show()
