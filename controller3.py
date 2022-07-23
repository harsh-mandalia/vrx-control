#!/usr/bin/env python
# [-3.2953384616321615e-06, -5.340189979056147e-07, 4.189534846467222e-08, -2.652429134862454e-07, -8.386605260178179e-08]
# [1.1131960826060772e-06, 5.606295814726696e-07, 4.606478135779979e-07, 3.6862991743710135e-07, 3.149088088774174e-07, 2.70247319814929e-07, 2.5362814480739527e-07, 2.2407572974748879e-07, 2.3368480829588233e-07, 2.1390390962385763e-07]
# [5.365724215197658e-06, 5.621772219422619e-06, 4.910860476473387e-06, 5.245029991236806e-06, 4.428555469182062e-06, 4.240771542517596e-06, 6.2795566560001324e-06, 5.2244505083659125e-06, 5.367395261055252e-06, 6.82533927966397e-06]
# [6.389234582319021e-06, 6.187770107828242e-06, 5.559184454593598e-06, 5.405926129222871e-06, 4.7196061026012875e-06, 4.447100698446952e-06, 4.813219904387886e-06, 4.825184474493705e-06, 4.782899387562616e-06, 5.418362913714987e-06, 4.991161108443861e-06, 5.0980040567975065e-06, 5.043835215805174e-06, 5.450864576084144e-06, 5.500635214977604e-06, 5.230542133708556e-06, 5.687689244839919e-06, 5.0567166973922975e-06, 5.674845739325149e-06, 6.071254006731575e-06]
# [5.210624253241465e-06, 5.404897557767225e-06, 4.715225721690307e-06, 4.669100613844491e-06, 4.699731696009551e-06, 4.44700715711088e-06, 4.550648592345003e-06, 5.156495585240827e-06, 4.533665997232219e-06, 3.8003209376188955e-06]

# [5.679457489367829e-06, 5.335335189517189e-06, 5.22824356502312e-06, 5.1019241766840094e-06, 5.122089594639713e-06, 4.674095704875979e-06, 4.905952853677331e-06, 4.98071092509339e-06, 5.010703169345867e-06, 4.785462105609026e-06]



import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Imu
from gazebo_msgs.msg import ModelState
import tf
import numpy as np
import matplotlib.pyplot as plt 
import pickle

# latitude_d = -33.72276383094204
# longitude_d = 150.67398401835345
latitude_0 = -33.7227685
longitude_0 = 150.6739911

latitude_f = -33.722702498
longitude_f = 150.67402097002
euler_d=[0, 0, np.pi/2]

latitude = 0
longitude = 0
euler=[0, 0, 0]
quaternion=[0, 0, 0, 0]

sign=0

xs=[]
ys=[]
error=[]

latitudes=[]
latitude_ds=[]
longitudes=[]
longitude_ds=[]
times=[]

def gps_callback(msg):
    global latitude, longitude
    latitude = msg.latitude
    longitude = msg.longitude

def imu_callback(msg):
    global quaternion
    quaternion[0] = msg.orientation.x
    quaternion[1] = msg.orientation.y
    quaternion[2] = msg.orientation.z
    quaternion[3] = msg.orientation.w

def get_pos(Coeff, slope, tf, t):
    if t>tf:
        t=tf
    latitude_d = Coeff[0] + Coeff[1]*t + Coeff[2]*t**2 + Coeff[3]*t**3 + Coeff[4]*t**4 + Coeff[5]*t**5
    longitude_d = longitude_f - (latitude_f-latitude_d)/slope

    return latitude_d, longitude_d

def get_ilc(middle_thrust_ilc_old, t):
    if len(middle_thrust_ilc_old)==0:
        return 0
    for i in range(len(middle_thrust_ilc_old)):
        if middle_thrust_ilc_old[i][1]>t:
            break
    val1=middle_thrust_ilc_old[i][0]
    val0=middle_thrust_ilc_old[i-1][0]
    t1=middle_thrust_ilc_old[i][1]
    t0=middle_thrust_ilc_old[i-1][1]
    # print(t,t1,t0)
    val = val1 - (t1-t)*(val1-val0)/(t1-t0)
    # print(middle_thrust_ilc_old)
    return val

def main_code():
    global latitude, longitude
    reso = 500
    n = 20
    orientation_thrust_ilc_new = [0]*reso
    orientation_thrust_ilc_old = [0]*reso
    # middle_thrust_ilc_new = [[0,0]]*reso
    middle_thrust_ilc_new=[]
    middle_thrust_ilc_old=[]
    # middle_thrust_ilc_old = [[0,0]]*reso
    error_avg=[0]*n
    rospy.init_node('controller', anonymous=True)

    rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, gps_callback)
    rospy.Subscriber("/wamv/sensors/imu/imu/data", Imu, imu_callback)

    rc_pub = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=1)
    lc_pub = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=1)
    mc_pub = rospy.Publisher('/wamv/thrusters/lateral_thrust_cmd', Float32, queue_size=1)
    ra_pub = rospy.Publisher('/wamv/thrusters/right_thrust_angle', Float32, queue_size=1)
    la_pub = rospy.Publisher('/wamv/thrusters/left_thrust_angle', Float32, queue_size=1)
    ma_pub = rospy.Publisher('/wamv/thrusters/lateral_thrust_angle', Float32, queue_size=1)
    gazebo_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    rate = rospy.Rate(10) # 10hz


    #---------- for path ---------------#
    slope = (latitude_f-latitude_0)/(longitude_f-longitude_0)
    p0 = latitude_0
    pf = latitude_f
    t_0 = 0
    t_f = 60
    v0, vf, a0, af = 0, 0, 0, 0
    
    A = np.array([[1, t_0, t_0**2, t_0**3, t_0**4, t_0**5],
    [0, 1, 2*t_0, 3*t_0**2, 4*t_0**3, 5*t_0**4],
    [0, 0, 2, 6*t_0, 12*t_0**2, 20*t_0**3],
    [1, t_f, t_f**2, t_f**3, t_f**4, t_f**5],
    [0, 1, 2*t_f, 3*t_f**2, 4*t_f**3, 5*t_f**4],
    [0, 0, 2, 6*t_f, 12*t_f**2, 20*t_f**3]])
    
    D = np.array([p0, v0, a0, pf, vf, af])
    A_inv = np.linalg.inv(A)
    Coeff = np.dot(A_inv,D)
    #---------- for path ---------------#

    model_state = ModelState()
    model_state.model_name = "wamv"
    model_state.pose.position.x = -532
    model_state.pose.position.y = 162
    model_state.pose.position.z = 0.1
    model_state.pose.orientation.z = 0.479
    model_state.pose.orientation.w = 0.878

    kp, kd, ki= 30000, 10000, 500

    for k in range(n):
        count = 0
        error_sum=0
        error_dist_old = 0
        error_dist_d = 0
        t1 = rospy.get_time()
        error_dist_i = 0
        rospy.sleep(1)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            euler = [i for i in tf.transformations.euler_from_quaternion(quaternion)]

            if euler_d[2]>0:
                if euler[2]<0:
                    euler[2]+=2*np.pi
            elif euler_d[2]<0:
                if euler[2]>0:
                    euler[2]+=-2*np.pi
            orientation_thrust = 0.1*(euler[2]-euler_d[2])
            rc=-orientation_thrust
            lc=orientation_thrust

            t = rospy.get_time() - t0
            latitude_d, longitude_d = get_pos(Coeff, slope, t_f, t)

            angle_m = np.arctan2(latitude_d-latitude, longitude_d-longitude)-(euler[2]-np.pi/2)
            sign = -1
            if angle_m>np.pi/2:
                ma = angle_m-np.pi
                sign = 1
            elif angle_m<-np.pi/2:
                ma = angle_m+np.pi
                sign = 1
            else:
                ma=angle_m
            
            error_dist = sign*np.sqrt((longitude_d-longitude)**2+(latitude_d-latitude)**2)
            
            # middle_thrust_ilc_new[count][0] = 0.9*get_ilc(middle_thrust_ilc_old,t) + 5000*error_dist
            # middle_thrust_ilc_new[count][1] = t
            middle_thrust_ilc = 0.9*get_ilc(middle_thrust_ilc_old,t) + 5000*error_dist
            middle_thrust_ilc_new.append([middle_thrust_ilc, t])
            # middle_thrust_ilc = middle_thrust_ilc_new[count][0]
            # print(middle_thrust_ilc_new[:count])
            # middle_thrust_ilc_old[count] = middle_thrust_ilc_new[count]
            
            #add middle_thrust_ilc_new, middle_thrust_ilc_old a function of time

            t2 = rospy.get_time()

            # error_dist=np.sqrt((longitude_d-longitude)**2+(latitude_d-latitude)**2)
            if t2!=t1:
                error_dist_d = (error_dist-error_dist_old)/(t2-t1)
            error_dist_old = error_dist
            error_dist_i += error_dist*(t2-t1)

            t1 = rospy.get_time()

            mc = (middle_thrust_ilc + kp*error_dist + kd*error_dist_d + ki*error_dist_i)

            ra=0*np.pi
            la=0*np.pi

            rc_pub.publish(rc)
            lc_pub.publish(lc)
            mc_pub.publish(mc)
            ra_pub.publish(ra)
            la_pub.publish(la)
            ma_pub.publish(ma)

            # xs.append(longitude)
            # ys.append(latitude)
            # error.append(error_dist)
            # plt.plot(error[1:])
            # plt.xlim([150.673, 150.674])
            # plt.ylim([-33.72264, -33.72268])
            latitudes.append(latitude)
            latitude_ds.append(latitude_d)
            longitudes.append(longitude)
            longitude_ds.append(longitude_d)
            times.append(t+k*1.5*t_f)
            plt.plot(t+k*1.5*t_f,latitude_d, ".r")
            plt.plot(t+k*1.5*t_f,latitude, ".b")
            plt.pause(0.001)
            # plt.plot()

            # rospy.loginfo("from main")
            # print(round(latitude,11), round(longitude,11), round(euler[2],4), round(mc,4), count)
            # print(round(middle_thrust_ilc,5), round(kp*error_dist,5), round(kd*error_dist_d,5), round(ki*error_dist_i,5))
            
            count+=1
            # if (latitude!=0 and longitude!=0 and euler[2]!=0):
            error_sum += abs(error_dist)
            if (t>1.5*t_f):
                gazebo_pub.publish(model_state)
                rc_pub.publish(0)
                lc_pub.publish(0)
                mc_pub.publish(0)
                error_avg[k] = error_sum/count
                print("iteration", k+1, "done.", error_avg[k])
                rospy.sleep(5)
                break

            rate.sleep()
        middle_thrust_ilc_old = middle_thrust_ilc_new
        middle_thrust_ilc_new = []
    print(error_avg)
    with open('vrx_data/ilc.pkl', 'wb') as file:
        pickle.dump([error_avg, latitudes,latitude_ds,longitudes, longitude_ds, times, middle_thrust_ilc_new], file)
    plt.show()

if __name__ == '__main__':
    try:
        main_code()
    except rospy.ROSInterruptException:
        pass
