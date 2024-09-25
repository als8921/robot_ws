import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from Automation.BDaq import *
from Automation.BDaq.InstantDiCtrl import InstantDiCtrl
from Automation.BDaq.InstantDoCtrl import InstantDoCtrl
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.BDaqApi import AdxEnumToString, BioFailed

MAIN_CONTROL_PERIOD = 1.0
DATA_ACQUISITION_PERIOD = 0.0005 #seconds, 2kHz
ENCODER_COUNTS_PER_TURN = 15
ENCODER_INTERPOLATION = 2
SIGNAL_HIGH = 1
SIGNAL_LOW = 0

GEAR_RATIO = 93
PID_KP_GAIN = 0.0
PID_KI_GAIN = 0.0
PID_KD_GAIN = 0.0
TORQUE_CONSTANT = 1000.0 / 147.0 # Torque constant: 147mNm/A
MAXONMOTOR_VOLTAGE_TO_CURRENT_CONST = 0.1


class RailController(Node):
    def __init__(self):
        super().__init__('rail_controller')
        timer_period = MAIN_CONTROL_PERIOD
        timer_period2 = DATA_ACQUISITION_PERIOD     
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)  ##Position CMD subscription
        self.subscription ## Prevent unused variable warning
        self.timer = self.create_timer(timer_period, self.timer_callback)       ## Main control loop timer.         
        self.timer2 = self.create_timer(timer_period2, self.timer_callback2)    ## Data acquisition(Encoder 500CPT) loop timer.
       
        ##------------------------------------ initialize DAQ ------------------------------------------  
        ## DI parameter init      
        self.deviceDescription = "USB-4716,BID#0"
        # #self.profilePath = u""
        #self.data = []
        self.di_startPort = 0
        self.di_portCount = 1
        self.instantDiCtrl = InstantDiCtrl(self.deviceDescription)
        self.ret = ErrorCode.Success
        self.encoder_cnt = 0
        self.encoder_calculated_angle = 0.0 #degree
        self.channelA_current_state = 0
        self.channelA_previous_state = 0
        self.channelB_current_state = 0
        
        ## DO parameter init      
        # self.do_startPort = 0
        # self.do_portCount = 1
        # self.dataBuffer = [0] * self.portCount        
        # self.instantDoCtrl = InstantDoCtrl(self.deviceDescription)

        ## AO parameter init       
        self.channelStart = 0
        self.channelCount = 1
        self.dataBuffer = [0.0] * self.channelCount
        self.instantAo = InstantAoCtrl(self.deviceDescription)
        self.aochannels = self.instantAo.channels
        self.first_channel = self.aochannels[0]
        self.first_channel.valueRange = ValueRange.V_0To5       

        ##-------------------------- initialize Position Control Parameters ----------------------------  
        self.ref_pos = 0.0
        self.act_pos = 0.0
        self.err_pos = 0.0
        self.motor_force = 0.0
        self.input_current = 0.0
        self.input_voltage = 0.0

        
    ## Position CMD (Reference position update)
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.ref_pos = float(msg.data)

    ## Data acquisition(Encoder 500CPT) loop timer callback. 
    ## Angle calculated using 2x interpolation encoder signal
    def timer_callback2(self):        
        self.ret, data = self.instantDiCtrl.readAny(self.di_startPort, self.di_portCount)        
        self.channelA_current_state = data[0] & 0x01
        self.channelB_current_state = (data[0] >> 1) & 0x01
        if (self.channelA_previous_state == SIGNAL_LOW) and (self.channelA_current_state == SIGNAL_HIGH):
            if self.channelB_current_state == SIGNAL_HIGH:
                self.encoder_cnt += 1
            else:
                self.encoder_cnt -= 1
        if (self.channelA_previous_state == SIGNAL_HIGH) and (self.channelA_current_state == SIGNAL_LOW):
            if self.channelB_current_state == 0:
                self.encoder_cnt += 1
            else:
                self.encoder_cnt -= 1

        self.encoder_calculated_angle = (self.encoder_cnt/(ENCODER_COUNTS_PER_TURN * ENCODER_INTERPOLATION)) * 360.0
        self.channelA_previous_state = self.channelA_current_state


    ## Main control loop timer callback  
    def timer_callback(self):
        ## 1). Data acquisition and calculation
        self.act_pos = self.encoder_calculated_angle       
                

        ## 2). Ouput calculation       
        self.err_pos = self.ref_pos - self.act_pos      #position error calculation
        self.motor_force = PID_KP_GAIN * self.err_pos   #P control
        

        ## 3). Post-process of control output
        self.input_current = (self.motor_force / GEAR_RATIO) * (TORQUE_CONSTANT)
        self.input_voltage = self.input_current / MAXONMOTOR_VOLTAGE_TO_CURRENT_CONST
        
        if self.input_voltage > 10:
            self.input_voltage = 10
        if self.input_voltage < 0:
            self.input_voltage = 0

        self.dataBuffer[0] = self.input_voltage
        #self.instantAo.writeAny(self.channelStart, self.channelCount, None, self.dataBuffer) 
        #-------------------------------------------AO test---------------------------------------------
        self.dataBuffer[0] = 0.0
        self.instantAo.writeAny(self.channelStart, self.channelCount, None, self.dataBuffer)    
        

    ## Close DAQ
    def close_daq(self):
        self.instantAo.dispose()
        self.instantDiCtrl.dispose()
        #self.instantDoCtrl.dispose()
        



def main(args=None):
    rclpy.init(args=args)
    rail_controller = RailController()
    rclpy.spin(rail_controller)

    rail_controller.destroy_node()    
    rail_controller.close_daq()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
