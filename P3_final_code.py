import sys
sys.path.append('../')
from Common.project_library import *

# 1. Interface Configuration
project_identifier = 'P3B' # enter a string corresponding to P0, P2A, P2A, P3A, or P3B
ip_address = '192.168.2.15' # enter your computer's IP address
hardware = False # True when working with hardware. False when working in the simulation

# 2. Servo Table configuration
short_tower_angle = 315 # enter the value in degrees for the identification tower 
tall_tower_angle = 90 # enter the value in degrees for the classification tower
drop_tube_angle = 180#270# enter the value in degrees for the drop tube. clockwise rotation from zero degrees

# 3. Qbot Configuration
bot_camera_angle = 0 # angle in degrees between -21.5 and 0

# 4. Bin Configuration
# Configuration for the colors for the bins and the lines leading to those bins.
# Note: The line leading up to the bin will be the same color as the bin 

bin1_offset = 0.15 # offset in meters
bin1_color = [1,0,0] # e.g. [1,0,0] for red
bin2_offset = 0.19
bin2_color = [0,1,0]
bin3_offset = 0.17
bin3_color = [0,0,1]
bin4_offset = 0.19
bin4_color = [1,1,1]

#--------------- DO NOT modify the information below -----------------------------

if project_identifier == 'P0':
    QLabs = configure_environment(project_identifier, ip_address, hardware).QLabs
    bot = qbot(0.1,ip_address,QLabs,None,hardware)
    
elif project_identifier in ["P2A","P2B"]:
    QLabs = configure_environment(project_identifier, ip_address, hardware).QLabs
    arm = qarm(project_identifier,ip_address,QLabs,hardware)

elif project_identifier == 'P3A':
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    configuration_information = [table_configuration,None, None] # Configuring just the table
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
    
elif project_identifier == 'P3B':
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    qbot_configuration = [bot_camera_angle]
    bin_configuration = [[bin1_offset,bin2_offset,bin3_offset,bin4_offset],[bin1_color,bin2_color,bin3_color,bin4_color]]
    configuration_information = [table_configuration,qbot_configuration, bin_configuration]
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
    bins = bins(bin_configuration)
    bot = qbot(0.1,ip_address,QLabs,bins,hardware)
    

#---------------------------------------------------------------------------------
# STUDENT CODE BEGINS
#---------------------------------------------------------------------------------
def transfer_container(number_of_Bins):
    print('Loading the hopper')
    time.sleep(0.5)
    if number_of_Bins==1:
        x=0.047
    elif number_of_Bins==2:
        x=-0.046
    else:
        x=-0.139

    arm.move_arm(0.668, 0.0, 0.270) 
    time.sleep(1)
    arm.control_gripper(40)
    time.sleep(1)
    arm.move_arm(0.0,-0.007,0.563)
    time.sleep(2)
    arm.move_arm(x, -0.590, 0.523) 
    time.sleep(2)
    arm.control_gripper(-32)
    time.sleep(1.5)
    arm.rotate_shoulder(-40)
    time.sleep(1.5)
    arm.home()
    


def container_to_Qbot(container_exists):
    Bins=[]
    Bins_mass=[]
    number_of_Bins=0
    total_mass=0
    global my_last_bin
    global my_last_mass
    while number_of_Bins<4:
        if container_exists==False:
            information=table.dispense_container(random.randint(1,6),True)
            Bin=information[2]
            mass=float(information[1])
            Bins.append(Bin)
            Bins_mass.append(mass)
            total_mass=sum(Bins_mass)
            number_of_Bins+=1
            if total_mass>90:
                break
            elif number_of_Bins==2 and Bins[0]!=Bins[1]:
                my_last_bin=Bins[1]
                my_last_mass=Bins_mass[1]
                break
            elif number_of_Bins==3 and Bins[1]!=Bins[2]:
                my_last_bin=Bins[2]
                my_last_mass=Bins_mass[2]
                break    
        else:
            number_of_Bins+=1
            Bins.append(my_last_bin)
            Bins_mass.append(my_last_mass)
            total_mass=sum(Bins_mass)
            container_exists=False
            
        transfer_container(number_of_Bins)    
    Bin=Bins[0]
    Qbot_to_TheBin(Bin)
    return Bin




def follow_YellowLine():
    Left_IR,Right_IR=bot.line_following_sensors()
    if Left_IR==1 and Right_IR==1:
        bot.set_wheel_speed([0.08,0.08])
    elif Left_IR==1 and Right_IR==0:
        bot.set_wheel_speed([0,0.03])
    else:
        bot.set_wheel_speed([0.03,0])
def Qbot_to_TheBin(Bin):
    print('Moving twoards',Bin)
    bot.rotate(-104)
    bot.activate_color_sensor()
    time.sleep(0.5)
    Variable=True
    while (Variable):
        follow_YellowLine()
        RGB,colour_data=bot.read_color_sensor()
        if RGB!=[0,0,0]:
            if RGB==[1,0,0] and Bin=="Bin01":
                bot.deactivate_color_sensor()
                Variable=False
                move_towards_Bin()
            elif RGB==[0,1,0] and Bin=="Bin02":
                bot.deactivate_color_sensor()
                Variable=False
                move_towards_Bin()
            elif RGB==[0,0,1] and Bin=="Bin03":
                bot.deactivate_color_sensor()
                Variable=False
                move_towards_Bin()
            elif RGB==[1,1,1] and Bin=="Bin04" :
                bot.deactivate_color_sensor()
                Variable=False
                move_towards_Bin()
            else:
                pass
                
    
def drop_off_container():
    print('Dropping off the container')
    bot.stop()
    time.sleep(1)
    bot.rotate(15)
    bot.activate_stepper_motor()
    bot.dump()
    bot.deactivate_stepper_motor()
    time.sleep(1)
def move_towards_Bin():
    bot.activate_ultrasonic_sensor()
    Variable=True
    while (Variable):
        follow_YellowLine()
        distance=bot.read_ultrasonic_sensor()
        if distance<0.15:
            while (Variable):
                Left_IR,Right_IR=bot.line_following_sensors()
                if Left_IR==1 and Right_IR==1:
                    bot.rotate(-2)
                    bot.forward_time(1.2)
                    bot.deactivate_ultrasonic_sensor()
                    Variable=False
                    time.sleep(0.5)
                    drop_off_container()
                    time.sleep(0.5)
                elif Left_IR==1 and Right_IR==0:
                    bot.set_wheel_speed([0,0.03])
                else:
                    bot.set_wheel_speed([0.03,0])
def return_home():
    while True:
        follow_YellowLine()
        m=bot.position()
        if 1.45<=m[0]<=1.51 and -0.01<=m[1]<=0.01 :
            bot.stop()
            time.sleep(0.3)
            bot.rotate(-105)
            bot.forward_time(0.5)
            bot.stop()
            time.sleep(0.3)
            bot.rotate(110)
            break

   
def main():
    try:
        container_exists=False
        while True:
            bot.rotate(100)
            container_to_Qbot(container_exists)
            return_home()
            container_exists=True
    except:
        bot.stop()
        print('System terminated')

    
    


#---------------------------------------------------------------------------------
# STUDENT CODE ENDS
#---------------------------------------------------------------------------------
