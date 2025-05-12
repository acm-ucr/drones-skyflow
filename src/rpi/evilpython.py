averaged_acc = [0 , 0 , 0]
averaged_gyro = [0,0,0]

prev_acc = [[3,3,3],[3,3,3]]
prev_gyro = [[3,3,3],[0,0,0]]

accel_data = [0,0,0]
gyro_data = [0,0,0]

for i in range(3):
    averaged_acc[i] = (prev_acc[0][i] + prev_acc[1][i]+accel_data[i])/3
    averaged_gyro[i] = (prev_gyro[0][i] + prev_gyro[1][i]+gyro_data[i])/3
print(averaged_acc)
print(averaged_gyro)
