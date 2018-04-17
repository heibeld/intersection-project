#include "msp.h"
#include <stdint.h>
#include <stdio.h>

void systick_init_interrupt(void);
void systick_delay_ms(uint16_t delay);
void systick_delay_us(uint16_t);
void intersection_pin_enable();
void pedestrian_pin_enable();
void timer32_init();
void IR_init();
void LCD_pin_enable(void);
void systick_delay_us (uint16_t delay);
void pulse_enable(void);
void push_nibble(uint8_t nibble);
void push_byte(uint8_t byte);
void LCD_enable(void);
void write_command (uint8_t byte);
void light_sequence_display(void);
void write_data (char data[],int n);
void light_time_display(void);
void second_timer_init(void);
void status_update(void);
void buzzer_init(void);
void buzzer_play(int tone);



int intersection_time = 0,flash_flag = 1,buggy = 0;
int currentedge = 0, lastedge=0, period=0, detect14Hz=0, detect10Hz=0, pedestrian=0,seconds = 0,deaf_time= 0;
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
    buzzer_init();

    NVIC_EnableIRQ(T32_INT1_IRQn);          //Enable Timer32_1 interrupt.
    NVIC_EnableIRQ(T32_INT2_IRQn);          //Enable Timer32_2 interrupt.
    NVIC_EnableIRQ(PORT2_IRQn);             //enable port interrupt
    NVIC_EnableIRQ(TA0_N_IRQn);             //enable timer a0 interrupt
    NVIC_EnableIRQ(TA3_0_IRQn);             //enable timer a3 interrupt for LCD seconds timer
    __enable_irq();                         //Enable all interrupts for MSP432


    light_sequence_display();   //print status to monitor

while (1)
{
    switch(state)
    {
        case go_busy:
        {
            status_update();
            light_time_display();
            if (detect14Hz)
            {
                while (detect14Hz);     //waits in loop until emergency vehicle leaves
            }
            else if (detect10Hz)
            {
                while (detect10Hz);      //busy stays green while bus is emitting
            }
            if (intersection_time >= 12)              //time up in traffic light cycle
            {
                intersection_time= 0;
                state = wait_busy;
                light_sequence_display();
            }
            break;
        }
        case wait_busy:
        {
            status_update();
            light_time_display();

            if (intersection_time >= 3)
            {
                intersection_time= 0;
                state = stop_busy;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        case stop_busy:
        {
            status_update();

            if (intersection_time >= 1)
            {
                intersection_time = 0;

                if (detect14Hz)         //if emergency vehicle
                {
                    state = go_busy;
                    light_sequence_display();   //print new status to monitor
                }
                else if (!(pedestrian))      // if no button pressed
                {
                    state = go_slow;
                    light_sequence_display();   //print new status to monitor
                }
                else if (pedestrian)    //if button pressed
                {
                    pedestrian = 0;
                    state = go_slow_ped;
                    light_sequence_display();   //print new status to monitor
                }

            }
            break;
        }
        case go_slow:
        {
            status_update();
            light_time_display();

            if (detect14Hz)
            {
                state = wait_slow;
                light_sequence_display();   //print status to monitor
            }
            if (intersection_time >= 8)
            {
                intersection_time= 0;
                TIMER_A3->CTL |= TACLR;
                state = wait_slow;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        case wait_slow:
        {
            status_update();
            light_time_display();

            if (intersection_time >= 3)
            {
                intersection_time= 0;
                state = stop_slow;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        case stop_slow:
        {
            status_update();

            if (intersection_time >= 1)
            {
                intersection_time= 0;
                state = go_busy;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        case go_slow_ped:
        {
            status_update();
            light_time_display();

            if (detect14Hz)
            {
                state = wait_slow_ped;
                light_sequence_display();   //print status to monitor
            }
            if (deaf_time & intersection_time < 3)
            {
                buzzer_play(4688);      //send 10Hz tone
            }
            if (deaf_time & intersection_time > 3)
            {
                buzzer_play(46874);     //send 1Hz tone
            }
            if (intersection_time >= 15)
            {
                intersection_time= 0;
                TIMER32_2->LOAD = 1500000;
                state = wait_slow_ped;
                light_sequence_display();   //print status to monitor
            }
            break;
        }
        case wait_slow_ped:
        {
            status_update();
            light_time_display();

            if (flash_flag == 1)
            {
                flash_flag = 0;     //clear flag
                P2->OUT ^= BIT4;    //toggle amber LED
                TIMER32_2->LOAD = 1500000; //set another timer for flag toggle
            }
            if (deaf_time)
            {
                buzzer_play(15624);
            }
            if (intersection_time  >= 5)
            {
                intersection_time= 0;
                state = stop_slow;
                buzzer_play(0);
                light_sequence_display();   //print new light status to monitor
                deaf_time = 0;
            }
            break;
        }
        default:
        {
            state = go_busy;
            light_sequence_display();   //print new light status to monitor
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
    intersection_time++;                       //increment flag
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
    TIMER32_1->CONTROL = 0b11100010;
    TIMER32_1->LOAD = 3000000;              //load for 1 second interrupts for timer
    TIMER32_2->CONTROL = 0b11100011;
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
    seconds = 0;
    TIMER_A3->CTL |= TIMER_A_CTL_CLR;
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

/*
 * Pulses the enable pin on the LCD in order to send nibble
 * properly timed for sending one nibble, not double send
 */
void pulse_enable(void)
{
    P4->OUT &=~ BIT2;       //set e low
    systick_delay_us(10);
    P4->OUT |= BIT2;        //Set E high
    systick_delay_us(20);
    P4->OUT &=~ BIT2;
    systick_delay_us(10);
}
/*
 * Sends nibble to LCD
 */
void push_nibble(uint8_t nibble)
{
    P4->OUT &=~ 0xF0;
    P4->OUT |= (nibble & 0x0F) << 4;
    pulse_enable();
    systick_delay_ms(3);
}
/*
 * Controls the two nibbles that need to be sent to the LCd to build a character
 */
void push_byte(uint8_t byte)
{
    uint8_t nibble;
    P4->OUT |= BIT0;                //RS = 1 set to data
    nibble =(byte & 0xF0)>>4;       //assign MSB's
    push_nibble(nibble);            //send to LCD
    nibble = (byte & 0x0F);         //assign LSB's
    systick_delay_us(100);          //wait for LCD to be ready
    push_nibble(nibble);            //send LSB's to LCD
    systick_delay_us(100);          //preemptive delay
}
/*
 * This program enables the LCD by writing commands to the system, clearing and prepping the screen for use
 */
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
/*
 * Writes a single command to the LCD
 */
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
/*
 * Controls the character display to the LCD
 */
void light_sequence_display(void)
{
    write_command(0x01);
    write_command(0x00);                    //go to home pos
    char line1[] = "Green on Heibel ";
    char line2[] = "Yellow on Heibel";
    char line3[] = "   Red Light    ";     //init state display readouts
    char line4[] = "Green on Babb   ";
    char line5[] = "Yellow on Babb  ";

    if      (state == go_busy)
    {
        write_data(line1,17);           //write state to lcd
        systick_delay_us(100);          //delay between data and comm.
        write_command(0xC0);            //go to second line of lcd
        systick_delay_us(100);          //delay between data and comm.
    }
    else if (state == wait_busy)
    {
        write_data(line2,17);
        systick_delay_us(100);          //delay between data and comm.
        write_command(0xC0);            //go to second line of lcd
        systick_delay_us(100);          //delay between data and comm.
    }
    else if (state == stop_busy)
    {
        write_data(line3,17);
        systick_delay_us(100);          //delay between data and comm.
        write_command(0xC0);            //go to second line of lcd
        systick_delay_us(100);          //delay between data and comm.
    }
    else if (state == go_slow)
    {
        write_data(line4,17);
        systick_delay_us(100);          //delay between data and comm.
        write_command(0xC0);            //go to second line of lcd
        systick_delay_us(100);          //delay between data and comm.
    }
    else if (state == wait_slow)
    {
        write_data(line5,17);       //different displays for states
        systick_delay_us(100);          //delay between data and comm.
        write_command(0xC0);            //go to second line of lcd
        systick_delay_us(100);          //delay between data and comm.
    }
    else if (state == stop_slow)
    {
        write_data(line3,17);
        systick_delay_us(100);          //delay between data and comm.
        write_command(0xC0);            //go to second line of lcd
        systick_delay_us(100);          //delay between data and comm.
    }
    else if (state == go_slow_ped)
    {
        write_data(line4,17);
        systick_delay_us(100);          //delay between data and comm.
        write_command(0xC0);            //go to second line of lcd
        systick_delay_us(100);          //delay between data and comm.
    }
    else if (state == wait_slow_ped)
    {
        write_data(line5,17);
        systick_delay_us(100);          //delay between data and comm.
        write_command(0xC0);            //go to second line of lcd
        systick_delay_us(100);          //delay between data and comm.
    }
}
/*
 * command that pushes bytes to the LCD for each character in a passed string 
 */
void write_data (char data[],int n)
{
    P4->OUT |= BIT0;               //RS = 1 sets to d
    systick_delay_us(100);
    int i;
        for(i=0; i<=n ; i++)
        {
            push_byte(data[i]);
            //systick_delay_ms(100);

        }
}
/*
 * Timer that counts for determining if a pedestrian needs the sounds on cross
 */
void second_timer_init(void)
{
    TIMER_A3->CCR[0] = 32768;
    TIMER_A3->CTL |= TASSEL_1 | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;
    TIMER_A3->CCTL[0] = TIMER_A_CCTLN_CCIE;
}
/*
 * Displays the second line of the LCD, re- printing so that each second updates the display
 */
void light_time_display(void)
{
    write_command(0xC0);                //newline
    char time[10] = "Time: ";
    char sec[10]  = "n/a";
    char add[10]  = "sec   ";
    sprintf(sec,"%d   ",intersection_time);       //turns seconds integer into char array

    write_data(time,5);         //prints "TIME: "
    write_data(sec,3);          //prints time
    systick_delay_us(100);      //prints "sec"
    write_data(add,6);

    write_command(0x0C);                //remove cursor
}

void TA3_0_IRQHandler(void)
{
    if ((P2->IN & BIT6) == 0)   //if button is pressed
    {
        seconds++;
        if (seconds == 6)       //if 3 interrupts pass while button is held
        {
            deaf_time = 1;      //flag that  piezo  is needed for pedestrian
        }
    }

    else
    {
        seconds = 0;
    }
    TIMER_A3->CCTL[0] &=~ TIMER_A_CCTLN_CCIFG;
}
/*
 * This function is used to update the LEDs to their new state
 */

void status_update(void)
{
    if      (state == go_busy)
    {
        P5->OUT = 0b0010100;        //traffic light status
    }
    else if (state == wait_busy)
    {
        P5->OUT = 0b0010010;        //traffic light status
    }
    else if (state == stop_busy)
    {
        P5->OUT = 0b0010001;        //traffic light status
    }
    else if (state == go_slow)
    {
        P5->OUT = 0b1000001;        //traffic light status
    }
    else if (state == wait_slow)
    {
        P5->OUT = 0b0100001;        //traffic light status
    }
    else if (state == stop_slow)
    {
        P5->OUT = 0b0010001;        //traffic light status
        P2->OUT |= BIT4;
    }
    else if (state == go_slow_ped)
    {
        P5->OUT = 0b1000001;        //light status
        P2->OUT |= BIT7;       //turns white LED on
        P2->OUT &=~ BIT4;       //amber off
    }
    else if (state == wait_slow_ped)
    {
        P5->OUT = 0b0100001;        //light status
        P2->OUT &=~ BIT7;           //turns white LED off
    }
    else
    {
        ;
    }
}
void buzzer_init(void)
{
    P5->SEL0 |= BIT7;
    P5->SEL1 &=~ BIT7;
    TIMER_A2->CCR[0] = 6000; //init period
    TIMER_A2->CCR[2] = 0;       //no sound initially
    //TIMER_A2->EX0 = 0b0111;     //divide by 8 again
    TIMER_A2->CTL |= TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_1 | TIMER_A_CTL_CLR |
                     TIMER_A_CTL_ID_0;              //SMCLK, UP, CLEAR, DIV by 8
    TIMER_A2->CCTL[2] |= TIMER_A_CCTLN_OUTMOD_7;      //reset/set
}
void buzzer_play(int tone)
{
    TIMER_A2->CCR[0] = tone; //1 SEC. / 10
    TIMER_A2->CCR[2] = (tone/2); //50% DUTY CYCLE
}
