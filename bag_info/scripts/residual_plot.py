import csv
import matplotlib.pyplot as plt

# Load the data
data = {'residual_x': [], '3x_variance_x': [], 'residual_y': [], '3x_variance_y': [], 'residual_theta': [], '3x_variance_theta': []}
with open('/home/user/bag_info/scripts/residual_data.csv', 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        data['residual_x'].append(float(row['residual_x']))
        data['3x_variance_x'].append(float(row['3x_variance_x']))
        data['residual_y'].append(float(row['residual_y']))
        data['3x_variance_y'].append(float(row['3x_variance_y']))
        data['residual_theta'].append(float(row['residual_theta']))
        data['3x_variance_theta'].append(float(row['3x_variance_theta']))

# Create time axis
frequency = 10  # Hz
time = [i / frequency for i in range(len(data['residual_x']))]

# Plot for x
plt.figure(figsize=(10, 6))
plt.plot(time, data['residual_x'], label='Residual X')
plt.plot(time, data['3x_variance_x'], label='Positive 3x Variance X', linestyle='--')
plt.plot(time, [-y for y in data['3x_variance_x']], label='Negative 3x Variance X', linestyle='--')
plt.axhline(0, color='black', linewidth=0.5)
plt.xlabel('Time (s)')
plt.ylabel('X')
plt.title('Residual X with 3x Variance')
plt.legend()
plt.grid(True)
plt.show()

# Plot for y
plt.figure(figsize=(10, 6))
plt.plot(time, data['residual_y'], label='Residual Y')
plt.plot(time, data['3x_variance_y'], label='Positive 3x Variance Y', linestyle='--')
plt.plot(time, [-y for y in data['3x_variance_y']], label='Negative 3x Variance Y', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Y')
plt.title('Residual Y with 3x Variance')
plt.legend()
plt.grid(True)
plt.show()

# Plot for theta
plt.figure(figsize=(10, 6))
plt.plot(time, data['residual_theta'], label='Residual Theta')
plt.plot(time, data['3x_variance_theta'], label='Positive 3x Variance Theta', linestyle='--')
plt.plot(time, [-y for y in data['3x_variance_theta']], label='Negative 3x Variance Theta', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Theta')
plt.title('Residual Theta with 3x Variance')
plt.legend()
plt.grid(True)
plt.show()