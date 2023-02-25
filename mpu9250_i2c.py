# this is to be saved in the local folder under the name "mpu9250_i2c.py"
# it will be used as the I2C controller and function harbor for the project 
# refer to datasheet and register map for full explanation

import smbus,time
import sys
import numpy as np

# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_CONFIG2 = 0x1D
FIFO_EN      = 0x23
INT_ENABLE   = 0x38
INT_STATUS   = 0x3A
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
USER_CTRL    = 0x6A
FIFO_COUNTH  = 0x72
FIFO_COUNTL = 0x73
FIFO_R_W     = 0x74

gyro_dlpf = np.array([250,184,92,41,20,10,5,3600]) #correponding DLPF cuttoff frequencies for each register value (index)
gyro_config_sel = [0b00000,0b010000,0b10000,0b11000] # byte registers
gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
accel_dlpf = np.array([218.1,218.1,99,44.8,21.2,10.2,5.05,420]) #correponding DLPF cuttoff frequencies for each register value (index)
accel_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
__SENSOR__ = "none"

#AK8963 registers
AK8963_ADDR   = 0x68
AK8963_ST1    = 0x02
HXH          = 0x04
HYH          = 0x06
HZH          = 0x08
AK8963_ST2   = 0x09
AK8963_CNTL  = 0x0A
mag_sens = 4900.0 # magnetometer sensitivity: 4800 uT

def MPU6050_start():
    # reset all sensors
    bus.write_byte_data(MPU6050_ADDR,PWR_MGMT_1,0x00)
    time.sleep(0.1)
    # disable FIFO
    bus.write_byte_data(MPU6050_ADDR,FIFO_EN,0x00)
    bus.write_byte_data(MPU6050_ADDR,USER_CTRL,0x04) #reset fifo
    # power management and crystal settings
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01)
    time.sleep(0.1)
    #Write to Configuration register
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
    time.sleep(0.1)
    # interrupt register (related to overflow of data [FIFO])
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)
    time.sleep(0.1)
    # filter register
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG2,0) #accel f_choice_b and dlpf

def setFilter_gyro(cutoff):
    #set the low pass filter to the closest available cutoff frequency (above requested)
    reg = bus.read_byte_data(MPU6050_ADDR, GYRO_CONFIG)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG,reg & 0b11111100) # gyro fchoice_b (must be zero for DLPF to activate)
    Imin = np.argmin(gyro_dlpf)
    if cutoff < gyro_dlpf[Imin]:
        bus.write_byte_data(MPU6050_ADDR, CONFIG,0x40+int(Imin)) # gyro dlpf write
        print('Requested gyro DLPF cutoff frequency is too low, set to {0:2.2f} Hz'.format(gyro_dlpf[Imin]))
    else:
        gt = gyro_dlpf <= cutoff
        cutoff_set = np.max(gyro_dlpf[gt])
        I = np.argmax(gyro_dlpf==cutoff_set)
        bus.write_byte_data(MPU6050_ADDR, CONFIG,0x40+int(I)) # gyro dlpf write

def getFilter_gyro():
    #Get the low pass filter cutoff frequency
    FCHOICE_B = bus.read_byte_data(MPU6050_ADDR, GYRO_CONFIG) & 0b0011 # gyro fchoice_b (must be one for DLPF to activate)
    dlpf_i = bus.read_byte_data(MPU6050_ADDR, CONFIG) & 0x0F # gyro dlpf setting
    
    if (FCHOICE_B == 0):
        cutoff = gyro_dlpf[dlpf_i]
    else:
        print("The DLPF cutoff frequency is not in effect.")
        cutoff = 3600
    
    return cutoff

def setRange_gyro(rang): 
    #Write to gyro configuration register
    if rang == '250dps':
        bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[0]))
    elif rang == '500dps':
        bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[1]))
    elif rang == '1000dps':
        bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[2]))
    elif rang == '2000dps':
        bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[3]))
    else:
        print("Range options are '250dps', '500dps', '1000dps', or '2000dps'.")

def getRange_gyro():                          
    gyro_indx = (bus.read_byte_data(MPU6050_ADDR, GYRO_CONFIG) & 0b11000) >> 3
    rang = gyro_config_vals[gyro_indx] 
    
    return rang
        
def setFilter_accel(cutoff):
    #set the low pass filter to the closest available cutoff frequency (below requested)
    Imin = np.argmin(accel_dlpf)
    if cutoff < accel_dlpf[Imin]:
        bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG2,int(Imin)) #accel f_choice_b and dlpf
        print('Requested accel DLPF cutoff frequency is too low, set to {0:2.2f} Hz'.format(accel_dlpf[Imin]))
    else:
        gt = accel_dlpf <= cutoff
        cutoff_set = np.max(accel_dlpf[gt])
        I = np.argmax(accel_dlpf==cutoff_set)
        bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG2,int(I)) #accel f_choice_b and dlpf

def getFilter_accel():
    #Get the low pass filter cutoff frequency
    reg = bus.read_byte_data(MPU6050_ADDR, ACCEL_CONFIG2) # gyro fchoice_b (must be one for DLPF to activate)
    FCHOICE_B = (reg & 0b1000) >> 3
    dlpf_i = reg & 0x07 
    
    if (FCHOICE_B == 0):
        cutoff = accel_dlpf[dlpf_i]
    else:
        print("The accel DLPF is not in effect.")
        cutoff = 218.1
    
    return cutoff

def setRange_accel(rang):
    #Write to Accel configuration register
    if rang == '2g':
        bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[0]))
    elif rang == '4g':
        bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[1]))
    elif rang == '8g':
        bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[2]))
    elif rang == '16g':
        bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[3]))
    else:
        print("Range options are '2g', '4g', '8g', or '16g'.")

def getRange_accel():                          
    accel_indx = (bus.read_byte_data(MPU6050_ADDR, ACCEL_CONFIG) & 0b11000) >> 3
    rang = accel_config_vals[accel_indx]
    
    return rang

def setRate(rate):
    # sets sampling rate
    
    # For sample rate divider to take effect:
    # 1. FCHOICE_B must be 00
    FCHOICE_B = bus.read_byte_data(MPU6050_ADDR, GYRO_CONFIG) # gyro fchoice_b (must be one for DLPF to activate)
    # 2. dlpf setting must be 0 < dlpf < 7
    dlpf = bus.read_byte_data(MPU6050_ADDR, CONFIG) & 0x0F # gyro dlpf setting
    
    if (FCHOICE_B == 0 and dlpf < 7 and dlpf > 0):
        base_rate = 1000 #if these setting are in effect then the internal rate is 1000Hz
        rate_div = int(base_rate/rate-1)
        bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, rate_div)
    else:
        print("The DLPF cutoff frequency must be <= to 184 for the sample rate divisor to take effect.")

def getRate():
    # read sample rate 
    
    reg = bus.read_byte_data(MPU6050_ADDR, CONFIG)
    rate_div = bus.read_byte_data(MPU6050_ADDR, SMPLRT_DIV)
    dlpf = reg & 0x0F
    if dlpf < 7 and dlpf > 0:
        sample_rate = 1000/(1+rate_div)
    else:
        sample_rate = 8000
    
    return sample_rate

def rst_FIFO():
    #clear the FIFO, count goes to 0
    bus.write_byte_data(MPU6050_ADDR,USER_CTRL,0x44) #reset fifo AND enable the serial interface
    FIFO_overflow() #call the overflow register to clear it
    
def enable_FIFO(sensor):
    #enable saving sensor data to FIFO buffer (allows faster and more precise sampling)
    #sensor can be 'accel','gyro_x'
    global __SENSOR__
    current = bus.read_byte_data(MPU6050_ADDR,FIFO_EN)
    __SENSOR__ = sensor
    if (sensor == "accel"):
        bus.write_byte_data(MPU6050_ADDR,FIFO_EN,0x08)
        time.sleep(0.1)
    elif (sensor == "gyro"):
        bus.write_byte_data(MPU6050_ADDR,FIFO_EN,0x70)
        time.sleep(0.1)
    elif (sensor == "accel-gyro"):
        bus.write_byte_data(MPU6050_ADDR,FIFO_EN,0x78)
        time.sleep(0.1)
    elif (sensor == "none"):
        bus.write_byte_data(MPU6050_ADDR,FIFO_EN,0x00)
        time.sleep(0.1)
    bus.write_byte_data(MPU6050_ADDR,USER_CTRL,0x40) #enable the serial interface
    #bus.write_byte_data(MPU6050_ADDR,USER_CTRL,0x20)
    #time.sleep(0.1)

def count_FIFO():
    # read high and low FIFO values
    high = bus.read_byte_data(MPU6050_ADDR, FIFO_COUNTH)
    low = bus.read_byte_data(MPU6050_ADDR, FIFO_COUNTL)

    # combine high and low for unsigned bit value
    fifo_count = ((high << 8) | low)
    
    return fifo_count

def FIFO_overflow():
    #returns true if the fifo has overflowed
    
    # read the interrupt status register
    int_stat = bus.read_byte_data(MPU6050_ADDR, INT_STATUS)
    # extract bit 4 (overflow status)
    oflow = int_stat & 0x10 != 0
    
    return oflow

def read_continuous_FIFO(N):
    data = [] #empty data vector
    if __SENSOR__ == "accel-gyro":
        sensor_num = 6
    elif __SENSOR__ == "none":
        return 0
    else:
        sensor_num = 3
    i=0 #number of time samples
    print("Starting Continuous Acquisition...")
    while i < N:
        #This loop continuously reads the FIFO to check for new samples, it will quit if the FIFO overflows,
        #b/c that means it lost data.
        #time.sleep(.01)
        cnti = count_FIFO() #check number of bytes in the FIFO buffer
        oflowi = FIFO_overflow() #check if it overflowed since last read
        #print(cnti)
        #print(cnti)
        #print(oflowi)
        if oflowi == False:
            if not(cnti == 0):
                #if not empty
                block = read_block_FIFO()
                data.extend(block)
                #data_l.extend(block_l)
                i += len(block)/sensor_num/2 #for each sensor FIFO stores two bytes of data per sample
        else:
            i = N+1
            print("FIFO Overflowed: stopping loop. Try reducing sample rate")
    print("Acquisition Complete.")
    return data

def read_block_FIFO():
    #reads all bytes in the fifo (up to max of 32)
    count = count_FIFO() #samples in the FIFO
    read_count = min(32,count) #bytes to read (can only read up to 32)
    block = bus.read_i2c_block_data(MPU6050_ADDR, FIFO_R_W,read_count)
    
    return block

def read_single_FIFO():
    #reads the first byte from the fifo
    byte = bus.read_byte_data(MPU6050_ADDR, FIFO_R_W)
    
    return byte

def conv_FIFO(data,sensor,N=0):
    #adjust sensitivity and number of expected byte blocks based on sensor setting
    if (sensor == "accel-gyro"):
        byte_num = 12
    else:
        byte_num = 6
    
    if (N == 0 or N > len(data)//byte_num):
        N = len(data)//byte_num #read up to the full sample
    
    accel_sens = getRange_accel()
    gyro_sens = getRange_gyro()
    
    i = 0
    
    if (byte_num == 6):
        a_x = []
        a_y = []
        a_z = []
        while i < N*byte_num:        
            a_xh = data[i]
            a_xl = data[i+1]
            a_yh = data[i+2]
            a_yl = data[i+3]
            a_zh = data[i+4]
            a_zl = data[i+5]
            
            # combine higha and low for unsigned bit value
            a_xi = ((a_xh << 8) | a_xl)
            a_yi = ((a_yh << 8) | a_yl)
            a_zi = ((a_zh << 8) | a_zl)
        
            # convert to +- value
            if(a_xi > 32768):
                a_xi -= 65536
            if(a_yi > 32768):
                a_yi -= 65536
            if(a_zi > 32768):
                a_zi -= 65536
            
            #convert to acceleration in g and gyro dps
            if (sensor == "accel"):
                a_xc = (a_xi/(2.0**15.0))*accel_sens
                a_yc = (a_yi/(2.0**15.0))*accel_sens
                a_zc = (a_zi/(2.0**15.0))*accel_sens
            elif (sensor == "gyro"):
                a_xc = (a_xi/(2.0**15.0))*gyro_sens
                a_yc = (a_yi/(2.0**15.0))*gyro_sens
                a_zc = (a_zi/(2.0**15.0))*gyro_sens
        
            a_x.append(a_xc)
            a_y.append(a_yc)
            a_z.append(a_zc)
        
            i += byte_num
        return a_x,a_y,a_z
    elif (byte_num == 12):
        a_x = []
        a_y = []
        a_z = []
        w_x = []
        w_y = []
        w_z = []
        while i < N*byte_num:        
            a_xh = data[i]
            a_xl = data[i+1]
            a_yh = data[i+2]
            a_yl = data[i+3]
            a_zh = data[i+4]
            a_zl = data[i+5]
            w_xh = data[i+6]
            w_xl = data[i+7]
            w_yh = data[i+8]
            w_yl = data[i+9]
            w_zh = data[i+10]
            w_zl = data[i+11]
            
            # combine higha and low for unsigned bit value
            a_xi = ((a_xh << 8) | a_xl)
            a_yi = ((a_yh << 8) | a_yl)
            a_zi = ((a_zh << 8) | a_zl)
            w_xi = ((w_xh << 8) | w_xl)
            w_yi = ((w_yh << 8) | w_yl)
            w_zi = ((w_zh << 8) | w_zl)
        
            # convert to +- value
            if(a_xi > 32768):
                a_xi -= 65536
            if(a_yi > 32768):
                a_yi -= 65536
            if(a_zi > 32768):
                a_zi -= 65536
            if(w_xi > 32768):
                w_xi -= 65536
            if(w_yi > 32768):
                w_yi -= 65536
            if(w_zi > 32768):
                w_zi -= 65536
            
            #convert to acceleration in g and gyro dps
            a_xc = (a_xi/(2.0**15.0))*accel_sens
            a_yc = (a_yi/(2.0**15.0))*accel_sens
            a_zc = (a_zi/(2.0**15.0))*accel_sens
            w_xc = (w_xi/(2.0**15.0))*gyro_sens
            w_yc = (w_yi/(2.0**15.0))*gyro_sens
            w_zc = (w_zi/(2.0**15.0))*gyro_sens
        
            a_x.append(a_xc)
            a_y.append(a_yc)
            a_z.append(a_zc)
            w_x.append(w_xc)
            w_y.append(w_yc)
            w_z.append(w_zc)
        
            i += byte_num
        return a_x,a_y,a_z,w_x,w_y,w_z

def read_raw_bits(register):
    # read accel and gyro values
    high = bus.read_byte_data(MPU6050_ADDR, register)
    low = bus.read_byte_data(MPU6050_ADDR, register+1)

    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    
    # convert to +- value
    if(value > 32768):
        value -= 65536
    return value

def imu_read():
    # raw acceleration bits
    acc_x = read_raw_bits(ACCEL_XOUT_H)
    acc_y = read_raw_bits(ACCEL_YOUT_H)
    acc_z = read_raw_bits(ACCEL_ZOUT_H)

    # raw temp bits
##    t_val = read_raw_bits(TEMP_OUT_H) # uncomment to read temp
    
    # raw gyroscope bits
    gyro_x = read_raw_bits(GYRO_XOUT_H)
    gyro_y = read_raw_bits(GYRO_YOUT_H)
    gyro_z = read_raw_bits(GYRO_ZOUT_H)
    
    accel_sens = getRange_accel()
    gyro_sens = getRange_gyro()
    
    #convert to acceleration in g and gyro dps
    a_x = (acc_x/(2.0**15.0))*accel_sens
    a_y = (acc_y/(2.0**15.0))*accel_sens
    a_z = (acc_z/(2.0**15.0))*accel_sens

    w_x = (gyro_x/(2.0**15.0))*gyro_sens
    w_y = (gyro_y/(2.0**15.0))*gyro_sens
    w_z = (gyro_z/(2.0**15.0))*gyro_sens

##    temp = ((t_val)/333.87)+21.0 # uncomment and add below in return
    return a_x,a_y,a_z,w_x,w_y,w_z

def AK8963_start():
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
    time.sleep(0.1)
    AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
    AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
    AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,AK8963_mode)
    time.sleep(0.1)
    
def AK8963_reader(register):
    # read magnetometer values
    low = bus.read_byte_data(AK8963_ADDR, register-1)
    high = bus.read_byte_data(AK8963_ADDR, register)
    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    # convert to +- value
    if(value > 32768):
        value -= 65536
    return value

def AK8963_conv():
    # raw magnetometer bits

    loop_count = 0
    while 1:
        mag_x = AK8963_reader(HXH)
        mag_y = AK8963_reader(HYH)
        mag_z = AK8963_reader(HZH)

        # the next line is needed for AK8963
        if bin(bus.read_byte_data(AK8963_ADDR,AK8963_ST2))=='0b10000':
            break
        loop_count+=1
        
    #convert to acceleration in g and gyro dps
    m_x = (mag_x/(2.0**15.0))*mag_sens
    m_y = (mag_y/(2.0**15.0))*mag_sens
    m_z = (mag_z/(2.0**15.0))*mag_sens

    return m_x,m_y,m_z

# start I2C driver
bus = smbus.SMBus(1) # start comm with i2c bus
MPU6050_start() # instantiate gyro/accel
AK8963_start() # instantiate magnetometer
