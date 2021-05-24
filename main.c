/*------------------------------------------------------------------*/
/* --- 8051 car basic demo -----------------------------------------*/
/* --- Chip: STC89C52RD+ -------------------------------------------*/
/* --- Developer: @Autulea -----------------------------------------*/
/* --- Homepage: https://github.com/Autulea ------------------------*/
/*------------------------------------------------------------------*/

#include <STC89C5xRC.H>
#include <intrins.h>
#include <math.h>

//#define IR_test
//#define BCD_test
#define realeased
/**
 * @brief   define all used variables.
 */
#define FOSC 11059200
#define T1time (65536 - FOSC / 12 / 1000)
#define mode_amount 4
#define LED P30
#define IR_pin P33
#define left_motor_IN1 P12
#define left_motor_IN2 P13
#define right_motor_IN1 P14
#define right_motor_IN2 P15
#define left_motor_EN P16
#define right_motor_EN P17

int motor_time = 0, runtime = 0;
code unsigned char BCD[16] = {0x03, 0x9F, 0x25, 0x0D, 0x99, 0x49, 0x41, 0x1F, 0x01, 0x09, 0x11, 0xC1, 0x63, 0x85, 0x61, 0x71};
char globle_left_speed = 0, globle_right_speed = 0, mode = 2, tick = 0, delay_tick = 0, flag_ir_handle = 0;
char flag_sensor_scan = 0, flag_keep_going = 0;
char mid_way = 0, left_way = 0, right_way = 0, left_object_sniffer = 0, right_object_sniffer = 0;
unsigned char way_statu = 0, IRCOM = 0;
char temp = 0;

/**
 * @brief   timer0 init function. By this timer, system ticks every one milisecond.
 * @param   
 * @retval  no return val.
 */
void Timer0Init(void)
{
    TMOD = 0x01;
    TL0 = T1time;
    TH0 = T1time >> 8;
    TR0 = 1;
    ET0 = 1; //enable timer1 interrupt
}

/**
 * @brief   timer0 interruput service function.
 *          it can refreash timer reg, update PWM state, record a tick.
 *					
 * @param   
 * @retval  no retval
 */
void tim0_isr() interrupt 1
{
    TL0 = T1time;
    TH0 = T1time >> 8;
    tick++;
    if (tick == 16)
    {
        tick = 0;
        flag_sensor_scan = 1;
    }
    if (delay_tick)
    {
        delay_tick--;
    }
    if (runtime)
    {
        runtime--;
        left_motor_EN = ((tick <= globle_left_speed) && globle_left_speed);
        right_motor_EN = ((tick <= globle_right_speed) && globle_right_speed);
    }
    else
    {
        left_motor_EN = 0;
        right_motor_EN = 0;
    }
}

/**
 * @brief   motor power & runtime control
 * @param   left_speed: (-16~0~16)negative means backward, 0 means stop.
 * @param   right_speed: (-16~0~16)negative means backward, 0 means stop.
 * @param   time: time to working of motor in ticks.
 *								if time > 60000, the motor will keep running.
 * @retval  no retval
 */
void motor(char left_speed, char right_speed, unsigned int time)
{
    runtime = time;
    if (left_speed == 0 && right_speed == 0)
    {
        globle_left_speed = 0;
        globle_right_speed = 0;
    }
    else
    {
        if (left_speed >= 0)
        {
            left_motor_IN1 = 1;
            left_motor_IN2 = 0;
            globle_left_speed = cabs(left_speed);
        }
        else
        {
            left_motor_IN1 = 0;
            left_motor_IN2 = 1;
            globle_left_speed = cabs(left_speed);
        }
        if (right_speed >= 0)
        {
            right_motor_IN1 = 1;
            right_motor_IN2 = 0;
            globle_right_speed = cabs(right_speed);
        }
        else
        {
            right_motor_IN1 = 0;
            right_motor_IN2 = 1;
            globle_right_speed = cabs(right_speed);
        }
    }
}

/**
 * @brief   software delay while car is running
 * @param   
 * @retval  no retval
 */
void delay_runtime(void)
{
    while (runtime)
        ;
}

/**
 * @brief   software delay based on timer0 tick, while motor run normally
 * @param   
 * @retval  no retval
 */
void delay_ticktime(int time)
{
    delay_tick = time;
    while (delay_tick)
        ;
}

/**
 * @brief   sensor scaner
 * @param   
 * @retval  no retval
 */
void sensor_scan(void)
{
    left_way = P37;
    mid_way = P00;
    right_way = P36;
    left_object_sniffer = ~P34;
    right_object_sniffer = ~P35;
}

/**
 * @brief   direction select function while car running on a line
 * @param   
 * @retval  no retval
 */
void track_tracing(void)
{
    way_statu = (left_way << 2) + (mid_way << 1) + right_way;
    switch (way_statu)
    {
    case (1):
        motor(9, -9, 15); //turn right
        break;
    case (2):
        motor(9, 9, 15); //forward
        break;
    case (3):
        motor(9, -9, 15); //turn right
        break;
    case (4):
        motor(-9, 9, 15); //turn left
        break;
    case (6):
        motor(-9, 9, 15); //turn left
        break;
    case (7):
        motor(-9, -9, 15); //backword
        break;
    default:
        motor(0, 0, 15);
        break;
    }
}

/**
 * @brief   Obstacle avoidance function 
 * @param   
 * @retval  no retval
 */
void avoidence(void)
{
    char avoidence_statu = (left_object_sniffer << 1) + right_object_sniffer;
    switch (avoidence_statu)
    {
    case 0:
        motor(5, 5, 15); //forward
        break;
    case 1:
        motor(-9, 9, 192);//turn left
        delay_runtime();
        break;
    case 2:
        motor(9, -9, 192); //turn right
        delay_runtime();
        break;
    case 3:
        motor(-9, -9, 192); //backword
        delay_runtime();
        motor(-9, 9, 192); //turn left
        delay_runtime();
        break;
    default:
        break;
    }
}

/**
 * @brief   IR reciver init
 * @param   
 * @retval  no retval
 */
void IR_init(void)
{
    IT1 = 1;
    EX1 = 1;
}
/**
 * @brief   IR base tick delay
 * @param   
 * @retval  no retval
 */
void IRdelay(unsigned char x) //x*0.14MS
{
    unsigned char i;
    while (x--)
    {
        for (i = 0; i < 13; i++)
        {
        }
    }
} /**
 * @brief   IR interruput service function
 * @param   
 * @retval  no retval
 */
void IR_isr() interrupt 2
{
    flag_ir_handle = 1;
}
/**
 * @brief   IR demoduation function
 * @param   
 * @retval  no retval
 */
void IR_demod()
{
    unsigned char j, k, N = 0;
    unsigned char IRCOM_buffer[4];
    EX1 = 0;
    IRdelay(15);
    if (IR_pin == 1)
    {
        EX1 = 1;
        flag_ir_handle = 0;
        return;
    }
    while (!IR_pin)
    {
        IRdelay(1);
    }

    for (j = 0; j < 4; j++)
    {
        for (k = 0; k < 8; k++)
        {
            while (IR_pin)
            {
                IRdelay(1);
            }
            while (!IR_pin)
            {
                IRdelay(1);
            }
            while (IR_pin)
            {
                IRdelay(1);
                N++;
                if (N >= 30)
                {
                    EX1 = 1;
                    flag_ir_handle = 0;
                    return;
                }
            }
            IRCOM_buffer[j] = IRCOM_buffer[j] >> 1;
            if (N >= 8)
            {
                IRCOM_buffer[j] = IRCOM_buffer[j] | 0x80;
            }
            N = 0;
        } //end for k
    }     //end for j
    if (IRCOM_buffer[2] != ~IRCOM_buffer[3])
    {
        EX1 = 1;
        flag_ir_handle = 0;
        return;
    }
    IRCOM = IRCOM_buffer[2];
    flag_ir_handle = 0;
    EX1 = 1;
}
/**
 * @brief   IR code decoder, mode selecter
 * @param   
 * @retval  no retval
 */
void IR_decoder(void)
{
    if (flag_ir_handle)
    {
        IR_demod();
#ifdef realeased
        switch (IRCOM)
        {
        case 0x45:
            mode = 0;
            P0 = BCD[mode];
            flag_keep_going = 0;
            IRCOM = 0;
            break;
        case 0x46:
            mode = 1;
            P0 = BCD[mode];
            flag_keep_going = 0;
            IRCOM = 0;
            break;
        case 0x47:
            mode = 2;
            P0 = BCD[mode];
            flag_keep_going = 0;
            IRCOM = 0;
            break;
        default:
            break;
        }
#endif
        flag_ir_handle = 0;
    }
}
/**
 * @brief   mode2 IR command handler
 * @param   
 * @retval  no retval
 */
void mode2_run(void)
{
    switch (IRCOM)
    {
    case 0x43:
    case 0x44:
        motor(9, 9, 256); //forward
        IRCOM = 0;
        break;
    case 0x16:
    case 0x19:
    case 0x0d:
        motor(-9, -9, 256); //backward
        flag_keep_going = 0;
        IRCOM = 0;
        break;
    case 0x07:
        flag_keep_going? motor(3, 12, 192): motor(-9, 9, 96); //left
        IRCOM = 0;
        break;
    case 0x09:
        flag_keep_going? motor(12, 3, 192):motor(9, -9, 96); //right
        IRCOM = 0;
        break;
    case 0x40:
        motor(9, 9, 30000); //keep going
        flag_keep_going = 1;
        IRCOM = 0;
        break;
    case 0x15:
        motor(0, 0, 16); //stop
        flag_keep_going = 0;
        IRCOM = 0;
        break;
    default:
        break;
    }
}
/**
 * @brief   key exti(int0) init
 * @param   
 * @retval  no retval
 */
void exti0_init(void)
{
    IT0 = 1;
    EX0 = 1;
}
/**
 * @brief   key extern interruput service function
 * @param   
 * @retval  no retval
 */
void exti0_isr(void) interrupt 0
{
    unsigned char i, j;
    i = 36;
    j = 217;
    do
    {
        while (--j)
            ;
    } while (--i);
    if (P32 == 0)
    {
        mode = (mode + 1) % mode_amount;
        P0 = BCD[mode];
    }
}
/**
 * @brief   main function
 * @param   
 * @retval  no retval
 */
int main(void)
{
    left_motor_EN = 0;
    right_motor_EN = 0;
    Timer0Init();
    exti0_init();
    IR_init();
    EA = 1; //enable global interrupt
#ifdef IR_test
    while (1)
    {
        IR_decoder();
        if (IRCOM)
        {
            P0 = BCD[IRCOM >> 4];
            delay_ticktime(500);
            P0 = BCD[IRCOM & 0x0F];
            delay_ticktime(500);
            P0 = 0xFF;
            delay_ticktime(500);
            IRCOM = 0;
        }
    }
#endif

#ifdef BCD_test
    while (1)
    {
        temp = (temp + 1) % 16;
        //P0 = ~(0x01 << temp);
        P0 = BCD[temp];
        delay_ticktime(500);
    }
#endif

#ifdef realeased
    P0 = BCD[mode];
    while (1)
    {
        IR_decoder();
        if (flag_sensor_scan)
        {
            sensor_scan();
            switch (mode)
            {
            case (0):
                track_tracing();
                break;
            case (1):
                if (!delay_tick)
                    avoidence();
                break;
            case (2):
                if (flag_keep_going && (!runtime))
                    motor(9, 9, 15);
                if (IRCOM)
                    mode2_run();
                break;
            case (3):
                break;
            default:
                break;
            }
            flag_sensor_scan = 0;
        }
    }; //loop
#endif
}