// 360 degrees ultrasonic sampler
// By Pedro Iniguez Huerta
//
// This program uses all pins of port 1 for MSP430 to read up to 4 ultrasonic
// sensors and send the samples to Raspberry PI through I2C protocol.
// The MSP430 is the slave while the Raspberry PI 3 is the master.
// MSP430 supports the following commands from I2C bus:
//   - GET_SENSORS (50) ==> Return the number of seconds currently supported.
//                          this is configurable at compilation time (SENSORS=4)
//   - READ_SENSORS (50) ==> Return the samples of each ultrasonic seonsor.
//                           Notes:
//                            - To avoid hangs MSP430 starts a timer to abort
//                              waiting for ultrasonic response if MAX_TIMEOUT 
//                              expired
//                            - MSP430 added header '<' and tail '>' to the 
//                              sampling data prior sending ultrasonic samples 
//                              back to Raspberry PI 3.
//
//  This code can be compiled in two modes:
//  - Parallel    ==> Simultaneously enable all ultrasonic sensors to start sampling
//  - Round Robin ==> Read one ultrasonic sensor at a time.
//
//  If decided to use fewer sensors, at compilation time the code for non used
//  sensors is removed.
//
// The IDE used in this solution is IAR.
//
//   Below is an pinout diagram about how connect MSP430 with Reaspberry PI 3
//
//                                /|\  /|\
//               MSP430G2xx3     100k  100k     Raspberry PI 3
//                   slave         |    |        master
//             -----------------   |    |  -----------------
//HR/SRF05-1 -|P1.1 P1.7/UCB0SDA|<-|---+->|P3 SDA           |-
//HR/SRF05-2 -|P1.2             |  |      |                 |
//HR/SRF05-3 -|P1.3             |  |      |                 |-
//HR/SRF05-4 -|P1.4 P1.6/UCB0SCL|<-+----->|P5 SCL           |
//START SAMP -|P1.5             |         |                 |
// LED       -|P1.0             |         |                 |

#include <msp430g2253.h>

// PARALLEL is a precompiler directive so at compilation time unnecesary code
// is removed to reduce code size of the solution.
#define PARALLEL 1  // Parallel = 1  ==> simultaneous sensors sampling
                    // Parallel = 0  ==> round robin sensors sampling

#define SENSORS 4                               // Read 4 ultrasonic sensors
#define MAX_TIMEOUT 50                          // Sampling timeout
#define READ_START (BIT5)                       // P1.5 as  start sampling signal
#define ECHO_PINS  (BIT1 | BIT2 | BIT3 | BIT4)  // P1.1, P1.2, P1.3, P1.4
#define OUTPUT_PINS (BIT0)                      // P1.0 as sampling indicator

#define GET_SENSORS         50     // Get sensors command( from Raspberry PI)
#define READ_SENSORS        51     // read sensors command( from Raspberry PI)

#define SLAVE_ADDRESS      0x48    // MSP430 I2C slave address
#define COMMAND_LENGTH     1       // minimum command length
#define RESPONSE_LENGTH    4       // 2 +  SENSORS ==> HEADER S1 S2 S3 S4 TAIL
#define RX                 0       // Receive mode 
#define TX                 1       // Transmite mode

volatile unsigned char txData[RESPONSE_LENGTH];
volatile unsigned char sensorsData[SENSORS];
volatile unsigned char CMode = RX;
volatile unsigned char TXByteCtr = 0, RXByteCtr = 0;
volatile unsigned char rxCommand = 0;

void Setup_RX(void);
void Receive(void);
void readSensorsRoundRobin(void);
void readSensorsParallel(void);

volatile int miliseconds;
volatile int s1, s2 ,s3, s4;
volatile int timeout = 0;

#if PARALLEL == 1
    volatile int s1ms, s2ms ,s3ms, s4ms;
#endif

void main(void)
{
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;               // submainclock 1mhz
    WDTCTL = WDTPW + WDTHOLD;           // Stop WDT
  
    CCR0 = 1000;			// 1ms at 1mhz
    TACTL = TASSEL_2 + MC_1;            // SMCLK, upmode

    P1IFG  = 0x00;			// clear all interrupt flags
    P1DIR |= READ_START;		// trigger pin as output
    P1DIR |= OUTPUT_PINS;               // P1.0 as output for LED
    P1DIR &= ~ECHO_PINS;		// make pin P1.2 input (ECHO)
    
    P1OUT &= ~OUTPUT_PINS;              // turn LED off
 
    P1SEL |= BIT6 + BIT7;               // Assign I2C pins to USCI_B0
    P1SEL2|= BIT6 + BIT7;               // Assign I2C pins to USCI_B0
    
    _BIS_SR(GIE);                 	// global interrupt enable
 
    while(1)
    {
        /////////////////////////////////////////////////////////////
        // Wait for Raspberry for asking for reading data
        /////////////////////////////////////////////////////////////
        P1OUT |= OUTPUT_PINS;           // OUTPUT LED ON, waiting cmd form raspberry pi3
	P1IE &= ~ECHO_PINS;		// disable interupt for sensors
        P1IFG = 0x00;                   // clear flag just in case anything happened before
        CCTL0 = 0;                      // CCR0 interrupt DISABLED
                
        P1SEL |= BIT6 + BIT7;           // Assign I2C pins to USCI_B0
        P1SEL2|= BIT6 + BIT7;           // Assign I2C pins to USCI_B0
        Setup_RX();        
        Receive();

        P1OUT &= ~OUTPUT_PINS;          // OUTPUT LED off, sampling sensors

        // Read the sensors for GET_SENSORS command only
        if (rxCommand == GET_SENSORS)
        {
            #if PARALLEL == 1
                readSensorsParallel();
            #else
                readSensorsRoundRobin();
            #endif
            P1OUT &= ~OUTPUT_PINS;              // turn LED off
        } // End if GET_SENSORS
    } 
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    /////////////////////////////////////////////////////////////
    // If Echo interrupt received for sensor 1, turn on flag and wake CPU
    /////////////////////////////////////////////////////////////
    if(P1IFG&BIT1)         // is there interrupt pending?
    {
	P1IFG &= ~BIT1;	   // clear flag
        if(!(P1IES&BIT1))  // is this the rising edge?
            P1IES |= BIT1; // falling edge
        else
        {
            ///////////////////////////////////////////////////////
            // Parallel code only
            ///////////////////////////////////////////////////////
            #if PARALLEL == 1
                s1ms = miliseconds;
                s1 = TAR;
            #else
            ///////////////////////////////////////////////////////
            // Round Robin code only
            ///////////////////////////////////////////////////////
                s1 = 1;
                __bic_SR_register_on_exit(CPUOFF); // Return to active mode
            #endif
        }
    }

    #if SENSORS > 1
        /////////////////////////////////////////////////////////////
        // If Echo interrupt received for sensor 2, turn on flag and wake CPU
        /////////////////////////////////////////////////////////////
        if(P1IFG&BIT2)         // is there interrupt pending?
        {
            P1IFG &= ~BIT2;    // clear flag
            if(!(P1IES&BIT2))  // is this the rising edge?
                P1IES |= BIT2; // falling edge
            else
            {
                ///////////////////////////////////////////////////////
                // Parallel code only
                ///////////////////////////////////////////////////////
                #if PARALLEL == 1
                    s2ms = miliseconds;
                    s2 = TAR;
                #else
                ///////////////////////////////////////////////////////
                // Round Robin code only
                ///////////////////////////////////////////////////////
                    s2 = 2;
                    __bic_SR_register_on_exit(CPUOFF); // Return to active mode
                #endif
            }
        }
    #endif

    #if SENSORS > 2
        /////////////////////////////////////////////////////////////
        // If Echo interrupt received for sensor 3, turn on flag and wake CPU
        /////////////////////////////////////////////////////////////
        if(P1IFG&BIT3)         // is there interrupt pending?
        {
            P1IFG &= ~BIT3;    // clear flag
            if(!(P1IES&BIT3))  // is this the rising edge?
                P1IES |= BIT3; // falling edge
            else
            {
                ///////////////////////////////////////////////////////
                // Parallel code only
                ///////////////////////////////////////////////////////
                #if PARALLEL == 1
                    s3ms = miliseconds;
                    s3 = TAR;
                #else
                ///////////////////////////////////////////////////////
                // Round Robin code only
                ///////////////////////////////////////////////////////
                    s3 = 3;
                    __bic_SR_register_on_exit(CPUOFF); // Return to active mode
                #endif
            }
        }
    #endif

    #if SENSORS > 3
        /////////////////////////////////////////////////////////////
        // If Echo interrupt received for sensor 4, turn on flag and wake CPU
        /////////////////////////////////////////////////////////////
        if(P1IFG&BIT4)         // is there interrupt pending?
        {
            P1IFG &= ~BIT4;    // clear flag
            if(!(P1IES&BIT4))  // is this the rising edge?
                P1IES |= BIT4; // falling edge
            else
            {
                ///////////////////////////////////////////////////////
                // Parallel code only
                ///////////////////////////////////////////////////////
                #if PARALLEL == 1
                    s4ms = miliseconds;
                    s4 = TAR;
                #else
                ///////////////////////////////////////////////////////
                // Round Robin code only
                ///////////////////////////////////////////////////////
                    s4 = 4;
                    __bic_SR_register_on_exit(CPUOFF); // Return to active mode
                #endif
            }
        }
    #endif    
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
  miliseconds++;

  /////////////////////////////////////////////////////////////
  // If at least one of the sensors didn't respond, wake CPU to handle the error
  /////////////////////////////////////////////////////////////
  if(miliseconds > MAX_TIMEOUT)
  {
      timeout = 1;
      __bic_SR_register_on_exit(CPUOFF); // Return to active mode
  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
  /////////////////////////////////////////////////////////////
  // If Transmitting, move next byte to UCB0TXBUF to be transmitted
  /////////////////////////////////////////////////////////////
  if(CMode == TX)
  {
    UCB0TXBUF = txData[TXByteCtr];  // Transmit response
    TXByteCtr++;                    // Increment TX byte counter
  }
  
  /////////////////////////////////////////////////////////////
  // If Receiving, read Received command and prepare the response
  /////////////////////////////////////////////////////////////
  if(CMode == RX)
  {
      rxCommand = UCB0RXBUF;            // Save received command
      RXByteCtr++;                      // Increment RX byte count
      if(RXByteCtr >= COMMAND_LENGTH)
      {
          CMode = TX;
          RXByteCtr = 0;
          
          IE2 &= ~UCB0RXIE;             // Stop Receiving
          
          ///////////////////////////////////////////////////////////////////
          // GET_SENSORS:
          //      return the number of active sensors and start reading sensors
          //      to have the data when READ_SENSORS command is received. ex:
          //      for 2 sensors this will respond with following command:
          //      <2>    ==>  Which means 2 sensors available
          // READ_SENSORS:
          //      return the data of the sensors. Example for 2 sensors:
          //      <20,40>  ==> which means 20 cm distance sensor 1 and 40 cm for sensor 2
          //
          // OUTPUT FORMAT:
          //  <DATA>
          ///////////////////////////////////////////////////////////////////
          if (rxCommand == GET_SENSORS)
          {
              P1OUT |= OUTPUT_PINS;            // OUTPUT LED ON, cmd received
              
              txData[0] = '<';
              txData[1] = SENSORS;
              txData[2] = '>';
          }
          else if (rxCommand == READ_SENSORS)
          {
              for (int i = 0; i < SENSORS; i++)
                  txData[i] = sensorsData[i];
          }
            
          IE2 |= UCB0TXIE;                     // Start Transmitting
      }
  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCIAB0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{ 
    UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);          // Clear interrupt flags

    /////////////////////////////////////////////////////////////
    // If done transmitting the response, wake CPU to check for
    // pending processing to do
    /////////////////////////////////////////////////////////////
    if(CMode == TX)
    {
        if (TXByteCtr >= RESPONSE_LENGTH)        // Check TX byte counter
        {
            __bic_SR_register_on_exit(CPUOFF);   // Exit LPM0 if data was txed
        }
    }
}

void Setup_RX(void)
{
    __disable_interrupt();
    CMode = RX;
    IE2 &= ~UCB0TXIE;                         // Disable TX interrupt
    UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode
    UCB0I2COA = SLAVE_ADDRESS;                // Own Address is 048h
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
    UCB0I2CIE |= UCSTPIE + UCSTTIE;           // Enable STT and STP interrupt
    IE2 |= UCB0RXIE;                          // Enable RX interrupt  
}

void Receive(void)
{
    RXByteCtr = 0;
    TXByteCtr = 0;
    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

#if PARALLEL == 1
    void readSensorsParallel(void)
    {
        /////////////////////////////////////////////////////////////
        // Initialize control variables
        /////////////////////////////////////////////////////////////
        s1 = s2 = s3 = s4 = 0;
        s1ms = s2ms = s3ms = s4ms = 0;
        timeout = 0;
    
        // initialize sensors reading
        for (int i = 0; i < SENSORS; i++)
            sensorsData[i] = 255;  // No object detected
        
        /////////////////////////////////////////////////////////////////////
        // Configure ECHO pins to generate interrupts on data conversion done
        /////////////////////////////////////////////////////////////////////
        P1IFG  = 0x00;			// clear all interrupt flags
        P1IE = 0x00;		        // disable all interupts
        P1IES |= ECHO_PINS;             // trigger interrupt on rising edge for ECHO
        P1IE  |= ECHO_PINS;	        // enable interrupt on ECHO pins only
        P1IFG = 0x00;                   // clear flag just in case anything happened before
    
        /////////////////////////////////////////////////////////////////////
        // Start timer
        /////////////////////////////////////////////////////////////////////
        CCTL0 = CCIE;                   // CCR0 interrupt enabled
        CCR0 = 1000;			// 1ms at 1mhz
        TACTL = TASSEL_2 + MC_1;        // SMCLK, upmode
    
        /////////////////////////////////////////////////////////////////////
        // assert Trigger signal to
        /////////////////////////////////////////////////////////////////////
        P1OUT &= ~READ_START;           // stop pulse
        __delay_cycles(100);            // for 10us
        P1OUT |= READ_START;		// generate pulse
        __delay_cycles(100);            // for 10us
        miliseconds = 0;
        timeout = 0;
        TACTL |= TACLR;                 // clears timer A
        P1OUT &= ~READ_START;           // stop pulse
        
        /////////////////////////////////////////////////////////////
        // Wait until timeout or all sensors return the sample data
        /////////////////////////////////////////////////////////////
        __bis_SR_register(CPUOFF + GIE);

        /////////////////////////////////////////////////////////////
        // store the distance of each sensor into its corresponding position into an array
        /////////////////////////////////////////////////////////////
        if (s1 > 0)
        {
            unsigned int sample = s1ms*1000;
            sample += s1;
            sensorsData[0] = (unsigned char)(sample/116);  //58*2=116
        }

        #if SENSORS > 1
            if (s2 > 0)
            {
                unsigned int sample = s2ms*1000;
                sample += s2;
                sensorsData[1] = (unsigned char)(sample/116);  //58*2=116
            }
        #endif

        #if SENSORS > 2
            if (s3 > 0)
            {
                unsigned int sample = s3ms*1000;
                sample += s3;
                sensorsData[2] = (unsigned char)(sample/116);  //58*2=116
            }
        #endif

        #if SENSORS > 3
            if (s4 > 0)
            {
                unsigned int sample = s4ms*1000;
                sample += s4;
                sensorsData[3] = (unsigned char)(sample/116); //58*2=116
            }
        #endif
    
        /////////////////////////////////////////////////////////////////////
        // Stop timer
        /////////////////////////////////////////////////////////////////////
        CCTL0 = 0;                           // CCR0 interrupt DISABLED
    }

#else   // No parallel enabled

    void readSensorsRoundRobin(void)
    {  
        /////////////////////////////////////////////////////////////
        // Initialize control variables
        /////////////////////////////////////////////////////////////  
        int echobit = BIT1;
        int triggerbit = READ_START;
        int counter = 0;
        long sensor = 0;
        
        while (counter < SENSORS)
        {
            s1 = s2 = s3 = s4 = 0;
            /////////////////////////////////////////////////////////////////////
            // Configure ECHO pin to generate interrupt on data conversion done
            /////////////////////////////////////////////////////////////////////      
            P1IE = 0x00;		    // disable all interupts
            P1IE  |= echobit;               // enable interrupt on ECHO pins only
            P1IES |= echobit;               // trigger interrupt on rising edge for ECHO
            P1IFG = 0x00;                   // clear flag just in case anything happened before
    
            /////////////////////////////////////////////////////////////////////
            // Start timer
            /////////////////////////////////////////////////////////////////////
            CCTL0 = CCIE;                   // CCR0 interrupt enabled
            CCR0 = 1000;                    // 1ms at 1mhz
            TACTL = TASSEL_2 + MC_1;        // SMCLK, upmode
    
            /////////////////////////////////////////////////////////////////////
            // assert Trigger signal to
            /////////////////////////////////////////////////////////////////////        
            P1OUT &= ~triggerbit;           // set trigger bit to LOW
            __delay_cycles(100);            // for  1ms //10us
            P1OUT |= triggerbit;	    // set trigger bit to HIGH
            miliseconds = 0;
            timeout = 0;
            TACTL |= TACLR;                 // clears timer A
            __delay_cycles(100);            // for  1ms //10us
            P1OUT &= ~triggerbit;           // set trigger bit back to LOW
    
            /////////////////////////////////////////////////////////////
            // Wait until ECHO pin changes or timeout occurred
            /////////////////////////////////////////////////////////////        
            __bis_SR_register(CPUOFF + GIE); // LPM0 with interrupts enabled// Low Powe
    
            /////////////////////////////////////////////////////////////////////
            // Disable interrupts on ECHO pin
            /////////////////////////////////////////////////////////////////////      
            P1IE  &= ~echobit;               // disable interrupt on ECHO pins only
            
            /////////////////////////////////////////////////////////////
            // timeout occured, skip reading sensors which didn't respond
            /////////////////////////////////////////////////////////////
            if(timeout > 0)
            {
                timeout = 0;
                sensorsData[counter] = 255;  // Indicates timeout occurred detecting objects
                          
                counter++;
                echobit = echobit * 2;
                // ignore this sensor if not responding
                continue;
            }
    
            
            sensor = (long)miliseconds;
            sensor = sensor*1000 + (long)TAR;
            sensor /= 58;
    
            sensorsData[counter] = sensor/2;
    
            /////////////////////////////////////////////////////////////////////
            // sleep until timeout, to ensure anything of previous transmission
            // is finished
            /////////////////////////////////////////////////////////////////////
            __bis_SR_register(CPUOFF + GIE); // LPM0 with interrupts enabled// Low Powe
            
            /////////////////////////////////////////////////////////////////////
            // Stop timer and increase counters
            /////////////////////////////////////////////////////////////////////
            CCTL0 = 0;                           // CCR0 interrupt DISABLED 
            counter++;
            echobit = echobit * 2;
        }
    }
#endif