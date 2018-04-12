#include "msp.h"
#include <stdint.h>
#include <stdio.h>

void systick_init_interrupt(void);
void systick_delay_ms(uint16_t delay);
void systick_delay_us(uint16_t);
void intersection_pin_enable();
void pedestrian_pin_enable();
void timer32_init();
void timer32_delay(uint16_t delay);
void IR_init();
void LCD_pin_enable(void);
void systick_delay_us (uint16_t delay);
void pulse_enable(void);
void push_nibble(uint8_t nibble);
void push_byte(uint8_t byte);
void LCD_enable(void);
void write_command (uint8_t byte);
void light_sequence_display(void);
void write_data (char data[]);
void light_time_display(void);
void second_timer_init(void);


int flag = 0,flash_flag = 1,buggy = 0;
int currentedge, lastedge, period, detect14Hz, detect10Hz, pedestrian,seconds;
enum intersection_states{
    go_busy,
    wait_busy,
    stop_busy,
    go_slow,
    wait_slow,
    stop_slow,
    go_slow_ped,
    wait_slow_ped,
    stop_slow_ped
}state;

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;             // Stop WDT

    systick_init_interrupt();
    intersection_pin_enable();
    pedestrian_pin_enable();
    timer32_init();
    second_timer_init();
    IR_init();
    LCD_pin_enable();
    LCD_enable();
    NVIC_EnableIRQ(T32_INT1_IRQn);          //Enable Timer32_1 interrupt.
    NVIC_EnableIRQ(T32_INT2_IRQn);          //Enable Timer32_2 interrupt.
    NVIC_EnableIRQ(PORT2_IRQn);             //enable port interrupt
    NVIC_EnableIRQ(TA0_N_IRQn);             //enable timer a0 interrupt
    __enable_irq();                         //Enable all interrupts for MSP432
    timer32_delay(12);
    light_sequence_display();   //print status to monitor
while (1)
{
    switch(state)
    {
        case go_busy:
        {
            P5->OUT = 0b0010100;        //light status


            if (detect14Hz)
            {
                while (detect14Hz);     //waits in loop until emergency vehicle leaves
            }

            else if (detect10Hz)
            {
                while (detect10Hz);      //busy stays green while bus is emitting
            }

            if (flag == 1)              //time up in traffic light cycle
            {
                flag = 0;
                timer32_delay(3);
                TIMER_A3->CTL |= TACLR;
                seconds = 0;                //resets seconds counter for LED display
                state = wait_busy;
            }
            break;
        }
        case wait_busy:
        {
            P5->OUT = 0b0010010;        //light status

            if (flag == 1)
            {
                flag = 0;
                timer32_delay(1);
                TIMER_A3->CTL |= TACLR;
                seconds = 0;                //resets seconds counter for LED display
                state = stop_busy;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        case stop_busy:
        {
            P5->OUT = 0b0010001;        //light status

            if (flag == 1)
            {
                flag = 0;
                if (detect14Hz)         //if emergency vehicle
                {
                    state = go_busy;
                    light_sequence_display();   //print status to monitor
                }
                else if (!(pedestrian))      // if no button pressed
                {
                    timer32_delay(8);
                    state = go_slow;
                    light_sequence_display();   //print status to monitor
                }
                else if (pedestrian)    //if button pressed
                {
                    pedestrian = 0;
                    timer32_delay(15);
                    state = go_slow_ped;
                    light_sequence_display();   //print status to monitor
                }

            }
            break;
        }
        case go_slow:
        {
            P5->OUT = 0b1000001;        //light status

            if (detect14Hz)
            {
                state = wait_slow;
                light_sequence_display();   //print status to monitor
                timer32_delay(3);
            }
            if (flag == 1)
            {
                flag = 0;
                timer32_delay(3);
                TIMER_A3->CTL |= TACLR;
                seconds = 0;                //resets seconds counter for LED display
                state = wait_slow;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        case wait_slow:
        {
            P5->OUT = 0b0100001;        //light status

            if (flag == 1)
            {
                flag = 0;
                timer32_delay(1);
                TIMER_A3->CTL |= TACLR;
                seconds = 0;                //resets seconds counter for LED display
                state = stop_slow;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        case stop_slow:
        {
            P5->OUT = 0b0010001;        //light status

            P2->OUT |= BIT4;
            if (flag == 1)
            {
                flag = 0;
                timer32_delay(12);
                TIMER_A3->CTL |= TACLR;
                seconds = 0;                //resets seconds counter for LED display
                state = go_busy;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        case go_slow_ped:
        {
            P5->OUT = 0b1000001;        //light status

            P2->OUT |= BIT7;       //turns white LED on
            P2->OUT &=~ BIT4;       //amber off

            if (detect14Hz)
            {
                state = wait_slow_ped;
                light_sequence_display();   //print status to monitor
            }
            if (flag == 1)
            {
                flag = 0;
                timer32_delay(5);
                TIMER_A3->CTL |= TACLR;
                seconds = 0;                //resets seconds counter for LED display
                TIMER32_2->LOAD = 1500000;
                state = wait_slow_ped;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        case wait_slow_ped:
        {
            P5->OUT = 0b0100001;        //light status
            P2->OUT &=~ BIT7;           //turns white LED off

            if (flash_flag == 1)
            {
                flash_flag = 0;     //clear flag
                P2->OUT ^= BIT4;    //toggle amber LED
                TIMER32_2->LOAD = 1500000; //set another timer for flag toggle
            }
            if (flag == 1)
            {
                flag = 0;
                timer32_delay(1);
                TIMER_A3->CTL |= TACLR;
                seconds = 0;                //resets seconds counter for LED display
                state = stop_slow;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        default:
        {
            state = go_busy;
            light_sequence_display();   //print status to monitor
            timer32_delay(12);
        }
    }
}
}

void systick_init_interrupt(void)
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x05;
}

void systick_delay_ms(uint16_t delay)
{
    SysTick->LOAD = ((delay * 3000) - 1);  //delay for 1 msecond per delay value
    SysTick->VAL = 0;                              // any write to CVR clears it
    while ((SysTick->CTRL & 0x10000) == 0)
        ;               // wait for flag to be SET

}

/*
 * Starts delay of SysTick for a user entered integer of microseconds
 */
void systick_delay_us(uint16_t delay)
{
    SysTick->LOAD = ((delay * 3) - 1); //delay for 1 microsecond per delay value
    SysTick->VAL = 0;                              // any write to CVR clears it
    while ((SysTick->CTRL & 0x10000) == 0)
        ;               // wait for flag to be SET

}

void intersection_pin_enable()
{
    P5->SEL0 &=~ 0xFF;
    P5->SEL1 &=~ 0xFF;
    P5->DIR  |=  0xFF;
    P5->OUT  &=~ 0xFF;      //gpio, output, low
}

void pedestrian_pin_enable()
{
    P2->SEL0 &=~ BIT4 | BIT6 | BIT7;
    P2->SEL1 &=~ BIT4 | BIT6 | BIT7;    //Button and white and amber LED GPIO

    P2->DIR  &=~ BIT6;                  //button is input
    P2->DIR  |=  BIT4 | BIT7;           //amber and white led output

    P2->OUT &=~ BIT7;     //white init off
    P2->OUT |= BIT4;      //AMber init on

    //BUTTON SETUP
    P2->REN  |= BIT6;      //enable resistor
    P2->OUT  |= BIT6;
    P2->IES  |= BIT6;      //trigger on high to low
    P2->IE   |= BIT6;      //enable interrupt
    P2->IFG  = 0;           //clear flags


}

/*
 * Handles the general timing between states of the raffic lights
 * THis part flags when the time is up in a cycle of the intersection
 */
void T32_INT1_IRQHandler()        //Interrupt Handler for Timer 1.
{
    TIMER32_1->INTCLR = 1;          //Clear interrupt flag so it does not interrupt again immediately.
    flag = 1;                       //increment flag
}

/*
 * Handles the flashing of the amber light by rasing a flag every half second
 */
void T32_INT2_IRQHandler()        //Interrupt Handler for Timer 2.
{
    TIMER32_2->INTCLR = 1;          //Clear interrupt flag so it does not interrupt again immediately.
    flash_flag = 1;                       //increment flag
}

/*
 * Initialises timer 32's as one shot interrupts
 */
void timer32_init()
{
    TIMER32_1->CONTROL = 0b11100011;
    TIMER32_2->CONTROL = 0b11100011;
}

/*
 * Short function that when called starts a delay for a user input # of seconds
 */
void timer32_delay(uint16_t delay)
{
    TIMER32_1->LOAD = (delay * 3000000);
}

/*
 * Initializes the IR pins as well as the TIMER_A0 init steps
 */
void IR_init()
{
    P2->SEL0 |=  BIT5;               // TA0.CCI2A input capture pin, second function
    P2->SEL1 &=~ BIT5;                  // TA0.CCI2A input capture pin, second function
    P2->DIR  &=~ BIT5;

    TIMER_A0->CCR[0] = 65535;
    TIMER_A0->CTL |= TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC__UP |        //sets timer a registers to correct timer
                     TIMER_A_CTL_ID_3 | TIMER_A_CTL_CLR;

    TIMER_A0->CCTL[2] =TIMER_A_CCTLN_CM_1 | TIMER_A_CCTLN_CCIS_0 |
                       TIMER_A_CCTLN_CCIE | TIMER_A_CCTLN_CAP | TIMER_A_CCTLN_SCS;
}


/*
*Handles the IR  period sensor, such that flags are raised if a 10 or 14 Hz signal are recieved
*/
void TA0_N_IRQHandler(void)
 {
    currentedge = TIMER_A0->CCR[2];
    period = currentedge-lastedge;              //calculates period
    if (currentedge < lastedge)
        period = lastedge-currentedge;          //conditional for when timer rolls  over, so  that the period value is not negative
    detect14Hz = 0;
    detect10Hz = 0;                             //normailizes flag at 0 so constant refreshes clear flag
    if ((35635<period) && (period<39375))       // within 5% of 10Hz period
        detect10Hz=1;

    if ((25446<period) && (period<28055))       // within 5% of 14Hz period
        detect14Hz = 1;

    TIMER_A0->CCTL[2] &= ~(TIMER_A_CCTLN_CCIFG);    // Clear the interrupt flag

    lastedge = currentedge;                     //assigns last edge
 }

/*
 * Pedestrian walk button interrupt service
 */
void PORT2_IRQHandler(void)
{
    pedestrian = 1;
    P2->IFG = 0;
}

/*
 * Initialization of LCD pins for use in display
 */
void LCD_pin_enable(void)
{
    P4->SEL0 &= 0x00;
    P4->SEL1 &= 0x00;
    P4->DIR |= 0xFF;
    P4->OUT &=~ 0xFF;       //init LCD as gpio, outputs, init as low
}


void pulse_enable(void)
{
    P4->OUT &=~ BIT2;
    systick_delay_us(10);
    P4->OUT |= BIT2;
    systick_delay_us(20);
    P4->OUT &=~ BIT2;
    systick_delay_us(10);
}

void push_nibble(uint8_t nibble)
{
    P4->OUT &=~ 0xF0;
    P4->OUT |= (nibble & 0x0F) << 4;
    pulse_enable();
}

void push_byte(uint8_t byte)
{
    uint8_t nibble;
    P4->OUT |= BIT0;               //RS = 1 set to data
    nibble =(byte & 0xF0)>>4;
    push_nibble(nibble);
    nibble = (byte & 0x0F);
    systick_delay_us(100);
    push_nibble(nibble);
    systick_delay_us(100);
}

void LCD_enable(void)
{
    write_command(0x03);
    systick_delay_ms(10);
    write_command(0x03);
    systick_delay_us(200);
    write_command(0x03);
    systick_delay_ms(10);       //Reset Sequence

    write_command(0x02);
    systick_delay_us(100);
    write_command(0x02);        //Set to 4 bit mode

    write_command(0x08);        //2 lines 5x7 format
    systick_delay_us(100);

    write_command(0x0F);
    systick_delay_us(1000);      //Display ON cursor ON blinking

    write_command(0x01);
    systick_delay_us(1000);      //Clear display, move cursor to home

    write_command(0x06);
    systick_delay_us(100);      //Increment cursor
}

void write_command (uint8_t byte)
{
    uint8_t nibble;
    P4->OUT &=~ BIT0;               //RS = 0 sets to command
    systick_delay_us(100);
    nibble =(byte & 0xF0)>>4;
    push_nibble(nibble);
    nibble = (byte & 0x0F);
    systick_delay_us(100);
    push_nibble(nibble);
}

void light_sequence_display(void)
{
    //write_command(0x00);
    //systick_delay_us(1000);
    //write_command(0x01);
    //systick_delay_us(1000);
    char line1[] = " Green on Heibel ";
    char line2[] = " Yellow on Heibel";
    char line3[] = "    Red Light    ";     //init state display readouts
    char line4[] = "  Green on Babb  ";
    char line5[] = "  Yellow on Babb ";

    if      (state == go_busy)
    {
        write_data(line1);
        //systick_delay_us(100);                                //delay between data and comm.
        //write_command(0xC0);                                  //newline
        //systick_delay_us(100);
    }
    else if (state == wait_busy)     write_data(line2);
    else if (state == stop_busy)     write_data(line3);
    else if (state == go_slow)       write_data(line4);
    else if (state == wait_slow)     write_data(line5);       //different displays for states
    else if (state == stop_slow)     write_data(line3);
    else if (state == go_slow_ped)   write_data(line4);
    else if (state == wait_slow_ped) write_data(line5);
    systick_delay_us(100);                                //delay between data and comm.
    //write_command(0xC0);                                  //newline
}

void write_data (char data[])
{
    int i;
        for(i=0; i<=17 ; i++)
        {
            push_byte(data[i]);
        }
}

void second_timer_init(void)
{
    TIMER_A3->CCR[0] = 32768;
    TIMER_A3->CTL |= TASSEL_1 | MC_2 | TIMER_A_CTL_CLR | TIMER_A_CTL_IE;
}
void light_time_display(void)
{
    int time1, time2;
    time1 = seconds;

}

void TA3_0_IRQHandler(void)
{
    seconds++;
}
