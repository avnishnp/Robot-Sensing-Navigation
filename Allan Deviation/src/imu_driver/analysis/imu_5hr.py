# -*- coding: utf-8 -*-
"""IMU_5Hr.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1bHSPhZTTj3EyXYz3opINlG9nldPfObrX
"""

import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

b = bagreader('imu_5hr.bag')

# get the list of topics
print(b.topic_table)

data = b.message_by_topic('/imu')
print("File saved: {}".format(data))

pd.set_option('display.float_format', '{:.6f}'.format)

df_imu = pd.read_csv(data)
df_imu

def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z

quat_df=df_imu[['IMU.orientation.x','IMU.orientation.y','IMU.orientation.z','IMU.orientation.w']]

quat_df

import pandas as pd
from scipy.spatial.transform import Rotation

rot = Rotation.from_quat(quat_df)
rot_euler = rot.as_euler('xyz', degrees=True)
euler_df = pd.DataFrame(data=rot_euler, columns=['x', 'y', 'z'])

angvel=df_imu[['IMU.angular_velocity.x','IMU.angular_velocity.y','IMU.angular_velocity.z']]

gx= df_imu['IMU.angular_velocity.x']  # deg/s
gy= df_imu['IMU.angular_velocity.y'] 
gz= df_imu['IMU.angular_velocity.z']

angvel_AV=df_imu[['IMU.angular_velocity.x','IMU.angular_velocity.y','IMU.angular_velocity.z']].abs()
gx_AV= angvel_AV['IMU.angular_velocity.x'].abs()  # rad/s
gy_AV= angvel_AV['IMU.angular_velocity.y'].abs()
gz_AV= angvel_AV['IMU.angular_velocity.z'].abs()

angvel_AV.to_csv('gyro.csv',index=False)

def AllanDeviation(dataArr, FS, maxNumM):
    """Compute the Allan deviation (sigma) of time-series data.

    Algorithm obtained from Mathworks:
    https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html

    Args
    ----
        dataArr: 1D data array
        fs: Data sample frequency in Hz
        maxNumM: Number of output points
    
    Returns
    -------
        (taus, allanDev): Tuple of results
        taus (numpy.ndarray): Array of tau values
        allanDev (numpy.ndarray): Array of computed Allan deviations
    """
    ts = 1.0 / FS
    N = len(dataArr)
    Mmax = 2**np.floor(np.log2(N / 2))
    M = np.logspace(np.log10(1), np.log10(Mmax), num=maxNumM)
    M = np.ceil(M)  # Round up to integer
    M = np.unique(M)  # Remove duplicates
    taus = M * ts  # Compute 'cluster durations' tau

    # Compute Allan variance
    allanVar = np.zeros(len(M))
    for i, mi in enumerate(M):
        twoMi = int(2 * mi)
        mi = int(mi)
        allanVar[i] = np.sum(
            (dataArr[twoMi:N] - (2.0 * dataArr[mi:N-mi]) + dataArr[0:N-twoMi])**2
        )
    
    allanVar /= (2.0 * taus**2) * (N - (2.0 * M))
    return (taus, np.sqrt(allanVar))  # Return deviation (dev = sqrt(var))

DATA_FILE = 'gyro.csv' 
FS=40
dataArr = np.genfromtxt(DATA_FILE, delimiter=',')
ts = 1.0 / FS

# Separate into arrays
gx_arr = dataArr[1:, 0] # [deg/s]
gy_arr = dataArr[1:, 1] 
gz_arr = dataArr[1:, 2] 

# Calculate gyro angles
thetax = np.cumsum(gx_arr) * ts  # [deg]
thetay = np.cumsum(gy_arr) * ts
thetaz = np.cumsum(gz_arr) * ts

# Compute Allan deviations
(taux_g, adx_g) = AllanDeviation(thetax, FS,maxNumM=100)
(tauy_g, ady_g) = AllanDeviation(thetay, FS,maxNumM=100)
(tauz_g, adz_g) = AllanDeviation(thetaz, FS,maxNumM=100)
# abline_values = [-0.5 * i  for i in taux]
# Plot data on log-scale
plt.figure()
plt.title('Gyroscope Allan Deviations')
plt.plot(taux_g, adx_g, label='gx')
plt.plot(tauy_g, ady_g, label='gy')
plt.plot(tauz_g, adz_g, label='gz')


# plt.plot(taux, abline_values, 'r')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Allan Deviation [rad/sec]')
plt.grid(True, which="both", ls="-", color='0.65')
plt.legend()
plt.xscale('log')
plt.yscale('log')
plt.show()

#Angle random walk of GYRO
import math

# % Find the index where the slope of the log-scaled Allan deviation is equal
# % to the slope specified.
slope_gx = -0.5;
logtau_gx = np.log10(taux_g);
logadev_gx = np.log10(adx_g);
dlogadev_gx = np.diff(logadev_gx)/np.diff(logtau_gx)
result_gx= np.min(abs(dlogadev_gx - slope_gx))
i_gx = np.where((dlogadev_gx-slope_gx) == result_gx)
logadev_gxi = logadev_gx[i_gx]
logtau_gxi = logtau_gx[i_gx]
# % Find the y-intercept of the line
b_gx = float(logadev_gxi) - slope_gx*float(logtau_gxi)

# % Determine the angle random walk coefficient from the line.
logN_gx = slope_gx*math.log(1) + b_gx
N_gx = 10**logN_gx
tauN_gx = 1
lineN_gx=[]
for g in taux_g:
  lineN_gx.append(N_gx/g)

  # Calculating rate random walk 
slopek_gx=0.5
resultk_gx= np.min(abs(dlogadev_gx - slopek_gx))
ik_gx = np.where(abs(dlogadev_gx-slopek_gx) == resultk_gx)
logadevk_gxi = logadev_gx[ik_gx]
logtauk_gxi = logtau_gx[ik_gx]
# % Find the y-intercept of the line
bk_gx = float(logadevk_gxi) - slopek_gx*float(logtauk_gxi)

logK_gx = slopek_gx*math.log10(3) + bk_gx
K_gx = 10**logK_gx
lineK_gx=[]
for h in taux_g:
  lineK_gx.append(K_gx * math.sqrt(h/3))
tauK_gx = 3



# % Determine the bias instability coefficient from the line.
slope_gxb = 0;
result_gxb= np.min(abs(dlogadev_gx - slope_gxb))
i_gxb = np.where((dlogadev_gx-slope_gxb) == result_gxb)
logadev_igxb = logadev_gx[i_gxb]
logtau_igxb = logtau_gx[i_gxb]
# % Find the y-intercept of the line
b_gxb = float(logadev_igxb) - slope_gxb*float(logtau_igxb)
tauB_gx = taux_g[i_gxb]
scfB_gx = math.sqrt(2*math.log(2)/np.pi)
logB_gx = b_gxb - math.log10(scfB_gx)
B_gx = 10**logB_gx
lineB_gx=[]
for j in taux_g:
  lineB_gx.append( B_gx * scfB_gx )

# -------------------------End Bias---------------
plt.yscale('log')
plt.xscale('log')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Allan Deviation (rad/s)')
plt.title('Gyroscope Parameters (X)')
plt.plot(taux_g, adx_g,tauN_gx,N_gx,'o',tauK_gx,K_gx,'v',tauB_gx, scfB_gx*B_gx,'p',taux_g,lineN_gx,'--',taux_g,lineK_gx,'--',taux_g,lineB_gx,'--')
plt.gca().legend((' \u03C3','$\u03C3_{N}$ (rad/s) /\u221AHz','$\u03C3_{K}$ (rad/s)','$\u03C3_{B}$ (rad/s)'),bbox_to_anchor=(1, 1),loc='upper left')
plt.show()
# plt.text(N,K,B)
B_value_gx=(B_gx*0.664)
print(" N (angle random walk) is " + str(N_gx) + " rad/s/\u221AHz" )
print(" K (rate random walk) is " + str(K_gx) + " rad/s")
print(" B (Bias instability) is " + str(B_value_gx) + " rad/s" )

#Angle random walk of GYRO
import math

# % Find the indey where the slope of the log-scaled Allan deviation is equal
# % to the slope specified.
slope_gy = -0.5;
logtau_gy = np.log10(tauy_g);
logadev_gy = np.log10(ady_g);
dlogadev_gy = np.diff(logadev_gy)/np.diff(logtau_gy)
result_gy= np.min(abs(dlogadev_gy - slope_gy))
i_gy = np.where(abs(dlogadev_gy-slope_gy) == result_gy)
logadev_gyi = logadev_gy[i_gy]
logtau_gyi = logtau_gy[i_gy]
# % Find the y-intercept of the line
b_gy = float(logadev_gyi) - slope_gy*float(logtau_gyi)

# % Determine the angle random walk coefficient from the line.
logN_gy = slope_gy*math.log(1) + b_gy
N_gy = 10**logN_gy
tauN_gy = 1
lineN_gy=[]
for g in tauy_g:
  lineN_gy.append(N_gy/g)

  # Calculating rate random walk 
slopek_gy=0.5
resultk_gy= np.min(abs(dlogadev_gy - slopek_gy))
ik_gy = np.where(abs(dlogadev_gy-slopek_gy) == resultk_gy)
logadevk_gyi = logadev_gy[ik_gy]
logtauk_gyi = logtau_gy[ik_gy]
# % Find the y-intercept of the line
bk_gy = float(logadevk_gyi) - slopek_gy*float(logtauk_gyi)

logK_gy = slopek_gy*math.log10(3) + bk_gy
K_gy = 10**logK_gy
lineK_gy=[]
for h in tauy_g:
  lineK_gy.append(K_gy * math.sqrt(h/3))
tauK_gy = 3


# % Determine the bias instability coefficient from the line.
slope_gyb = 0;
result_gyb= np.min(abs(dlogadev_gy - slope_gyb))
i_gyb = np.where(abs(dlogadev_gy-slope_gyb) == result_gyb)
logadev_igyb = logadev_gy[i_gyb]
logtau_igyb = logtau_gy[i_gyb]
# % Find the y-intercept of the line
b_gyb = float(logadev_igyb) - slope_gyb*float(logtau_igyb)
tauB_gy = tauy_g[i_gyb]
scfB_gy = math.sqrt(2*math.log(2)/np.pi)
logB_gy = b_gyb - math.log10(scfB_gy)
B_gy = 10**logB_gy
lineB_gy=[]
for j in tauy_g:
  lineB_gy.append( B_gy * scfB_gy )

# -------------------------End Bias---------------
plt.yscale('log')
plt.xscale('log')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Allan Deviation (rad/s)')
plt.title('Gyroscope Parameters (Y)')
plt.plot(tauy_g, ady_g,tauN_gy,N_gy,'o',tauK_gy,K_gy,'v',tauB_gy, scfB_gy*B_gy,'p',tauy_g,lineN_gy,'--',tauy_g,lineK_gy,'--',tauy_g,lineB_gy,'--')
plt.gca().legend((' \u03C3','$\u03C3_{N}$ (rad/s) /\u221AHz','$\u03C3_{K}$ (rad/s)','$\u03C3_{B}$ (rad/s)'),bbox_to_anchor=(1, 1),loc='upper left')
plt.show()
# plt.teyt(N,K,B)
B_value_gy=(B_gy*0.664)
print(" N (angle random walk) is " + str(N_gy) + " rad/s/\u221AHz" )
print(" K (rate random walk) is " + str(K_gy) + " rad/s")
print(" B (Bias instability) is " + str(B_value_gy) + " rad/s" )

#Angle random walk of GYRO
import math

# % Find the indez where the slope of the log-scaled Allan deviation is equal
# % to the slope specified.
slope_gz = -0.5;
logtau_gz = np.log10(tauz_g);
logadev_gz = np.log10(adz_g);
dlogadev_gz = np.diff(logadev_gz)/np.diff(logtau_gz)
result_gz= np.min(abs(dlogadev_gz - slope_gz))
i_gz = np.where(abs(dlogadev_gz-slope_gz) == result_gz)
logadev_gzi = logadev_gz[i_gz]
logtau_gzi = logtau_gz[i_gz]
# % Find the y-intercept of the line
b_gz = float(logadev_gzi) - slope_gz*float(logtau_gzi)

# % Determine the angle random walk coefficient from the line.
logN_gz = slope_gz*math.log(1) + b_gz
N_gz = 10**logN_gz
tauN_gz = 1
lineN_gz=[]
for g in tauz_g:
  lineN_gz.append(N_gz/g)

  # Calculating rate random walk 
slopek_gz=0.5
resultk_gz= np.min(abs(dlogadev_gz - slopek_gz))
ik_gz = np.where(abs(dlogadev_gz-slopek_gz) == resultk_gz)
logadevk_gzi = logadev_gz[ik_gz]
logtauk_gzi = logtau_gz[ik_gz]
# % Find the y-intercept of the line
bk_gz = float(logadevk_gzi) - slopek_gz*float(logtauk_gzi)

logK_gz = slopek_gz*math.log10(3) + bk_gz
K_gz = 10**logK_gz
lineK_gz=[]
for h in tauz_g:
  lineK_gz.append(K_gz * math.sqrt(h/3))
tauK_gz = 3


# % Determine the bias instability coefficient from the line.
slope_gzb = 0;
result_gzb= np.min(abs(dlogadev_gz - slope_gzb))
i_gzb = np.where(abs(dlogadev_gz-slope_gzb) == result_gzb)
logadev_igzb = logadev_gz[i_gzb]
logtau_igzb = logtau_gz[i_gzb]
# % Find the y-intercept of the line
b_gzb = float(logadev_igzb) - slope_gzb*float(logtau_igzb)
tauB_gz = tauz_g[i_gzb]
scfB_gz = math.sqrt(2*math.log(2)/np.pi)
logB_gz = b_gzb - math.log10(scfB_gz)
B_gz = 10**logB_gz
lineB_gz=[]
for j in tauz_g:
  lineB_gz.append( B_gz * scfB_gz )

# -------------------------End Bias---------------
plt.yscale('log')
plt.xscale('log')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Allan Deviation (rad/s)')
plt.title('Gyroscope Parameters (Z)')
plt.plot(tauz_g, adz_g,tauN_gz,N_gz,'o',tauK_gz,K_gz,'v',tauB_gz, scfB_gz*B_gz,'p',tauz_g,lineN_gz,'--',tauz_g,lineK_gz,'--',tauz_g,lineB_gz,'--')
plt.gca().legend((' \u03C3','$\u03C3_{N}$ (rad/s) /\u221AHz','$\u03C3_{K}$ (rad/s)','$\u03C3_{B}$ (rad/s)'),bbox_to_anchor=(1, 1),loc='upper left')
plt.show()
# plt.tezt(N,K,B)
B_value_gz=(B_gz*0.664)
print(" N (angle random walk) is " + str(N_gz) + " rad/s/\u221AHz" )
print(" K (rate random walk) is " + str(K_gz) + " rad/s")
print(" B (Bias instability) is " + str(B_value_gz) + " rad/s" )

#Angle random walk of GYRO
import math

# % Find the indez where the slope of the log-scaled Allan deviation is equal
# % to the slope specified.
slope_gz = -0.5;
logtau_gz = np.log10(tauz_g);
logadev_gz = np.log10(adz_g);
dlogadev_gz = np.diff(logadev_gz)/np.diff(logtau_gz)
result_gz= np.min(abs(dlogadev_gz - slope_gz))
i_gz = np.where(abs(dlogadev_gz-slope_gz) == result_gz)
logadev_gzi = logadev_gz[i_gz]
logtau_gzi = logtau_gz[i_gz]
# % Find the y-intercept of the line
b_gz = float(logadev_gzi) - slope_gz*float(logtau_gzi)

# % Determine the angle random walk coefficient from the line.
logN_gz = slope_gz*math.log(1) + b_gz
N_gz = 10**logN_gz
tauN_gz = 1
lineN_gz=[]
for g in tauz_g:
  lineN_gz.append(N_gz/g)

  # Calculating rate random walk 
slopek_gz=0.5
resultk_gz= np.min(abs(dlogadev_gz - slopek_gz))
ik_gz = np.where(abs(dlogadev_gz-slopek_gz) == resultk_gz)
logadevk_gzi = logadev_gz[ik_gz]
logtauk_gzi = logtau_gz[ik_gz]
# % Find the y-intercept of the line
bk_gz = float(logadevk_gzi) - slopek_gz*float(logtauk_gzi)

logK_gz = slopek_gz*math.log10(3) + bk_gz
K_gz = 10**logK_gz
lineK_gz=[]
for h in tauz_g:
  lineK_gz.append(K_gz * math.sqrt(h/3))
tauK_gz = 3

# % Determine the bias instability coefficient from the line.
slope_gzb = 0;
result_gzb= np.min(abs(dlogadev_gz - slope_gzb))
i_gzb = np.where(abs(dlogadev_gz-slope_gzb) == result_gzb)
logadev_igzb = logadev_gz[i_gzb]
logtau_igzb = logtau_gz[i_gzb]
# % Find the y-intercept of the line
b_gzb = float(logadev_igzb) - slope_gzb*float(logtau_igzb)
tauB_gz = tauz_g[i_gzb]
scfB_gz = math.sqrt(2*math.log(2)/np.pi)
logB_gz = b_gzb - math.log10(scfB_gz)
B_gz = 10**logB_gz
lineB_gz=[]
for l in tauz_g:
  lineB_gz.append( B_gz * scfB_gz )

# -------------------------End Bias---------------


plt.yscale('log')
plt.xscale('log')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Allan Deviation (rad/s)')
plt.title('Gyroscope Parameters (Z)')
plt.plot(tauz_g, adz_g,tauN_gz,N_gz,'o',tauK_gz,K_gz,'v',tauB_gz, scfB_gz*B_gz,'p',tauz_g,lineN_gz,'--',tauz_g,lineK_gz,'--',tauz_g,lineB_gz,'--')
plt.gca().legend((' \u03C3','$\u03C3_{N}$ (rad/s) /\u221AHz','$\u03C3_{K}$ (rad/s)','$\u03C3_{B}$ (rad/s)'),bbox_to_anchor=(1, 1),loc='upper left')
plt.show()
# plt.tezt(N,K,B)
B_value_gz=(B_gz*0.664)
print(" N (angle random walk) is " + str(N_gz) + " rad/s/\u221AHz" )
print(" K (rate random walk) is " + str(K_gz) + " rad/s")
print(" B (Bias instability) is " + str(B_value_gz) + " rad/s" )
print(" Q (Quantization noise) is " + str(Q_gz) + " rad/s" )

N_mean=(N_gx+N_gy+N_gz)/3
N_mean

K_mean=(K_gx+K_gy+K_gz)/3
K_mean

B_value_mean=(B_value_gx+B_value_gy+B_value_gz)/3
B_value_mean

linacc_AV=df_imu[['IMU.linear_acceleration.x','IMU.linear_acceleration.y','IMU.linear_acceleration.z']].abs()
ax_AV= linacc_AV['IMU.linear_acceleration.x'].abs()  # m/s^2
ay_AV= linacc_AV['IMU.linear_acceleration.y'].abs()
az_AV= linacc_AV['IMU.linear_acceleration.z'].abs()

linacc_AV.to_csv('linacc.csv',index=False)

DATA_FILE = 'linacc.csv' 
FS=40
Dataarr = np.genfromtxt(DATA_FILE, delimiter=',')
ts = 1.0 / FS

# Separate into arrays
accx_arr = Dataarr[1:, 0] # [deg/s]
accy_arr = Dataarr[1:, 1] 
accz_arr = Dataarr[1:, 2] 

# Calculate gyro angles
thetax = np.cumsum(accx_arr) * ts  # [deg]
thetay = np.cumsum(accy_arr) * ts
thetaz = np.cumsum(accz_arr) * ts

# Compute Allan deviations
(taux_a, adx_a) = AllanDeviation(thetax, FS,maxNumM=100)
(tauy_a, ady_a) = AllanDeviation(thetay, FS,maxNumM=100)
(tauz_a, adz_a) = AllanDeviation(thetaz, FS,maxNumM=100)
# abline_values = [-0.5 * i  for i in taux]
# Plot data on log-scale
plt.figure()
plt.title('Accelerometer Allan Deviations')
plt.plot(taux_a, adx_a, label='accx')
plt.plot(tauy_a, ady_a, label='accy')
plt.plot(tauz_a, adz_a, label='accz')


# plt.plot(taux, abline_values, 'r')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Allan Deviation [m/s^2]')
plt.grid(True, which="both", ls="-", color='0.65')
plt.legend()
plt.xscale('log')
plt.yscale('log')
plt.show()

#Angle random walk of GYRO
import math

# % Find the index where the slope of the log-scaled Allan deviation is equal
# % to the slope specified.
slope_ax = -0.5;
logtau_ax = np.log10(taux_a);
logadev_ax = np.log10(adx_a);
dlogadev_ax = np.diff(logadev_ax)/np.diff(logtau_ax)
result_ax= np.min(abs(dlogadev_ax - slope_ax))
i_ax = np.where((dlogadev_ax-slope_ax) == result_ax)
logadev_axi = logadev_ax[i_ax]
logtau_axi = logtau_ax[i_ax]
# % Find the y-intercept of the line
b_ax = float(logadev_axi) - slope_ax*float(logtau_axi)

# % Determine the angle random walk coefficient from the line.
logN_ax = slope_ax*math.log(1) + b_ax
N_ax = 10**logN_ax
tauN_ax = 1
lineN_ax=[]
for g in taux_a:
  lineN_ax.append(N_ax/g)

  # Calculating rate random walk 
slopek_ax=0.5
resultk_ax= np.min(abs(dlogadev_ax - slopek_ax))
ik_ax = np.where((dlogadev_ax-slopek_ax) == resultk_ax)
logadevk_axi = logadev_ax[ik_ax]
logtauk_axi = logtau_ax[ik_ax]
# % Find the y-intercept of the line
bk_ax = float(logadevk_axi) - slopek_ax*float(logtauk_axi)

logK_ax = slopek_ax*math.log10(3) + bk_ax
K_ax = 10**logK_ax
lineK_ax=[]
for h in taux_a:
  lineK_ax.append(K_ax * math.sqrt(h/3))
tauK_ax = 3



# % Determine the bias instability coefficient from the line.
slope_axb = 0;
result_axb= np.min(abs(dlogadev_ax - slope_axb))
i_axb = np.where((dlogadev_ax-slope_axb) == result_axb)
logadev_iaxb = logadev_ax[i_axb]
logtau_ixb = logtau_ax[i_axb]
# % Find the y-intercept of the line
b_axb = float(logadev_iaxb) - slope_axb*float(logtau_ixb)
tauB_ax = taux_a[i_axb]
scfB_ax = math.sqrt(2*math.log(2)/np.pi)
logB_ax = b_axb - math.log10(scfB_ax)
B_ax = 10**logB_ax
lineB_ax=[]
for j in taux_a:
  lineB_ax.append( B_ax * scfB_ax )
# lineB_ax = B_ax * scfB_ax * np.ones(len(g));

# -------------------------End Bias---------------
plt.yscale('log')
plt.xscale('log')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Allan Deviation [m/$s^2$]')
plt.title('Accelerometer Parameters (X)')
plt.plot(taux_a, adx_a,tauN_ax,N_ax,'o',tauK_ax,K_ax,'v',tauB_ax, scfB_ax*B_ax,'p',taux_a,lineN_ax,'--',taux_a,lineK_ax,'--',taux_a,lineB_ax,'--')
plt.gca().legend((' \u03C3','$\u03C3_{N}$ (m/$s^2$) /\u221AHz','$\u03C3_{K}$ (m/$s^2$)','$\u03C3_{B}$ (m/$s^2$)'),bbox_to_anchor=(1, 1),loc='upper left')
plt.show()
# plt.text(N,K,B)
B_value_ax=(B_ax*0.664)
print(" N (Velocity random walk) is " + str(N_ax) + " m/s^2/\u221AHz" )
print(" K (rate random walk) is " + str(K_ax) + " m/s^2")
print(" B (Bias instability) is " + str(B_value_ax) + " m/s^2" )

#Angle random walk of GYRO
import math

# % Find the index where the slope of the log-scaled Allan deviation is equal
# % to the slope specified.
slope_ay = -0.5;
logtau_ay = np.log10(tauy_a);
logadev_ay = np.log10(ady_a);
dlogadev_ay = np.diff(logadev_ay)/np.diff(logtau_ay)
result_ay= np.min(abs(dlogadev_ay - slope_ay))
i_ay = np.where((dlogadev_ay-slope_ay) == result_ay)
logadev_ayi = logadev_ay[i_ay]
logtau_ayi = logtau_ay[i_ay]
# % Find the y-intercept of the line
b_ay = float(logadev_ayi) - slope_ay*float(logtau_ayi)

# % Determine the angle random walk coefficient from the line.
logN_ay = slope_ay*math.log(1) + b_ay
N_ay = 10**logN_ay
tauN_ay = 1
lineN_ay=[]
for g in tauy_a:
  lineN_ay.append(N_ay/g)

  # Calculating rate random walk 
slopek_ay=0.5
resultk_ay= np.min(abs(dlogadev_ay - slopek_ay))
ik_ay = np.where(abs(dlogadev_ay-slopek_ay) == resultk_ay)
logadevk_ayi = logadev_ay[ik_ay]
logtauk_ayi = logtau_ay[ik_ay]
# % Find the y-intercept of the line
bk_ay = float(logadevk_ayi) - slopek_ay*float(logtauk_ayi)

logK_ay = slopek_ay*math.log10(3) + bk_ay
K_ay = 10**logK_ay
lineK_ay=[]
for h in tauy_a:
  lineK_ay.append(K_ay * math.sqrt(h/3))
tauK_ay = 3



# % Determine the bias instability coefficient from the line.
slope_ayb = 0;
result_ayb= np.min(abs(dlogadev_ay - slope_ayb))
i_ayb = np.where(abs(dlogadev_ay-slope_ayb) == result_ayb)
logadev_iayb = logadev_ay[i_ayb]
logtau_iyb = logtau_ay[i_ayb]
# % Find the y-intercept of the line
b_ayb = float(logadev_iayb) - slope_ayb*float(logtau_iyb)
tauB_ay = tauy_a[i_ayb]
scfB_ay = math.sqrt(2*math.log(2)/np.pi)
logB_ay = b_ayb - math.log10(scfB_ay)
B_ay = 10**logB_ay
lineB_ay=[]
for j in tauy_a:
  lineB_ay.append( B_ay * scfB_ay )

# -------------------------End Bias---------------
plt.yscale('log')
plt.xscale('log')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Allan Deviation [m/$s^2$]')
plt.title('Accelerometer Parameters (Y)')
plt.plot(tauy_a, ady_a,tauN_ay,N_ay,'o',tauK_ay,K_ay,'v',tauB_ay, scfB_ay*B_ay,'p',tauy_a,lineN_ay,'--',tauy_a,lineK_ay,'--',tauy_a,lineB_ay,'--')
plt.gca().legend((' \u03C3','$\u03C3_{N}$ (m/$s^2$) /\u221AHz','$\u03C3_{K}$ (m/$s^2$)','$\u03C3_{B}$ (m/$s^2$)'),bbox_to_anchor=(1, 1),loc='upper left')
plt.show()
# plt.text(N,K,B)
B_value_ay=(B_ay*0.664)
print(" N (Velocity random walk) is " + str(N_ay) + " m/s^2/\u221AHz" )
print(" K (rate random walk) is " + str(K_ay) + " m/s^2")
print(" B (Bias instability) is " + str(B_value_ay) + " m/s^2" )

#Angle random walk of GYRO
import math

# % Find the index where the slope of the log-scaled Allan deviation is equal
# % to the slope specified.
slope_az = -0.5;
logtau_az = np.log10(tauz_a);
logadev_az = np.log10(adz_a);
dlogadev_az = np.diff(logadev_az)/np.diff(logtau_az)
result_az= np.min(abs(dlogadev_az - slope_az))
i_az = np.where(abs(dlogadev_az-slope_az) == result_az)
logadev_azi = logadev_az[i_az]
logtau_azi = logtau_az[i_az]
# % Find the y-intercept of the line
b_az = float(logadev_azi) - slope_az*float(logtau_azi)

# % Determine the angle random walk coefficient from the line.
logN_az = slope_az*math.log(1) + b_az
N_az = 10**logN_az
tauN_az = 1
lineN_az=[]
for g in tauz_a:
  lineN_az.append(N_az/g)

  # Calculating rate random walk 
slopek_az=0.5
resultk_az= np.min(abs(dlogadev_az - slopek_az))
ik_az = np.where(abs(dlogadev_az-slopek_az) == resultk_az)
logadevk_azi = logadev_az[ik_az]
logtauk_azi = logtau_az[ik_az]
# % Find the y-intercept of the line
bk_az = float(logadevk_azi) - slopek_az*float(logtauk_azi)

logK_az = slopek_az*math.log10(3) + bk_az
K_az = 10**logK_az
lineK_az=[]
for h in tauz_a:
  lineK_az.append(K_az * math.sqrt(h/3))
tauK_az = 3



# % Determine the bias instability coefficient from the line.
slope_azb = 0;
result_azb= np.min(abs(dlogadev_az - slope_azb))
i_azb = np.where((dlogadev_az-slope_azb) == result_azb)
logadev_iazb = logadev_az[i_azb]
logtau_izb = logtau_az[i_azb]
# % Find the y-intercept of the line
b_azb = float(logadev_iazb) - slope_azb*float(logtau_izb)
tauB_az = tauz_a[i_azb]
scfB_az = math.sqrt(2*math.log(2)/np.pi)
logB_az = b_azb - math.log10(scfB_az)
B_az = 10**logB_az
lineB_az=[]
for j in tauz_a:
  lineB_az.append( B_az * scfB_az )


# -------------------------End Bias---------------
plt.yscale('log')
plt.xscale('log')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Allan Deviation [m/$s^2$]')
plt.title('Accelerometer Parameters (Z)')
plt.plot(tauz_a, adz_a,tauN_az,N_az,'o',tauK_az,K_az,'v',tauB_az, scfB_az*B_az,'p',tauz_a,lineN_az,'--',tauz_a,lineK_az,'--',tauz_a,lineB_az,'--')
plt.gca().legend((' \u03C3','$\u03C3_{N}$ (m/$s^2$) /\u221AHz','$\u03C3_{K}$ (m/$s^2$)','$\u03C3_{B}$ (m/$s^2$)'),bbox_to_anchor=(1, 1),loc='upper left')
plt.show()
# plt.text(N,K,B)
B_value_az=(B_az*0.664)
print(" N (Velocity random walk) is " + str(N_az) + " m/s^2/\u221AHz" )
print(" K (Rate random walk) is " + str(K_az) + " m/s^2")
print(" B (Bias instability) is " + str(B_value_az) + " m/s^2" )

# #Wavelet Denoising

# import pywt
# def madev(d, axis=None):
#     """ Mean absolute deviation of a signal """
#     return np.mean(np.absolute(d - np.mean(d, axis)), axis)

# def wavelet_denoising(x, wavelet='db4', level=1):
#     coeff = pywt.wavedec(x, wavelet, mode="per")
#     sigma = (1/0.6745) * madev(coeff[-level])
#     uthresh = sigma * np.sqrt(2 * np.log(len(x)))
#     coeff[1:] = (pywt.threshold(i, value=uthresh, mode='hard') for i in coeff[1:])
#     return pywt.waverec(coeff, wavelet, mode='per')

# filtered = wavelet_denoising(df_imu['IMU.angular_velocity.x'], wavelet='bior3.5', level=1)    
# plt.figure(figsize=(10, 6))
# plt.plot(df_imu['IMU.angular_velocity.x'], label='Raw')
# plt.plot(filtered, label='Filtered')
# plt.legend()
# plt.title(f"DWT Denoising with Wavelet", size=15)
# plt.show()