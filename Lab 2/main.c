/*
 * main.c
 * Created: 23/10/2024 8:55:42 AM
 * Author : Erron Francis
 * ID: 816023346
 */ 

// Relevant libraries
#include "HEADER.h"
#include "sam.h"
#include <stdio.h>
#include <stdlib.h>

// Clock initialization function
void init_clock(void)
{
	
	uint32_t tempDFLL48CalibrationCoarse;	// used to retrieve DFLL48 coarse calibration value from NVM

	// Set Flash wait states for 48 MHz
	NVMCTRL->CTRLB.bit.RWS = 1;	
	
	// Enable XOSC32K clock (External on-board 32.768kHz oscillator), will be used as DFLL48M reference.
	// Configure SYSCTRL->XOSC32K settings
	SYSCTRL_XOSC32K_Type sysctrl_xosc32k =
	 {
		.bit.WRTLOCK = 0,		/* XOSC32K configuration is not locked */
		.bit.STARTUP = 0x2,		/* 3 cycle start-up time */
		.bit.ONDEMAND = 0,		/* Osc. is always running when enabled */
		.bit.RUNSTDBY = 0,		/* Osc. is disabled in standby sleep mode */
		.bit.AAMPEN = 0,		/* Disable automatic amplitude control */
		.bit.EN32K = 1,			/* 32kHz output is disabled */
		.bit.XTALEN = 1			/* Crystal connected to XIN32/XOUT32 */
	};
	
	// Write settings
	SYSCTRL->XOSC32K.reg = sysctrl_xosc32k.reg;
	
	// Enable the Oscillator - Separate step per data sheet recommendation (sec 17.6.3)
	SYSCTRL->XOSC32K.bit.ENABLE = 1;
	
	// Wait for XOSC32K to stabilize
	while(!SYSCTRL->PCLKSR.bit.XOSC32KRDY);
	
	// Put XOSC32K as source of Generic Clock Generator 1
	// Set the Generic Clock Generator 1 output divider to 1
	// Configure GCLK->GENDIV settings
	GCLK_GENDIV_Type gclk1_gendiv = 
	{
		.bit.DIV = 1,								/* Set output division factor = 1 */
		.bit.ID = GENERIC_CLOCK_GENERATOR_XOSC32K	/* Apply division factor to Generator 1 */
	};
	
	// Write settings
	GCLK->GENDIV.reg = gclk1_gendiv.reg;
	
	// Configure Generic Clock Generator 1 with XOSC32K as source
	GCLK_GENCTRL_Type gclk1_genctrl = 
	{
		.bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
		.bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
		.bit.OE = 0,			/* Disable generator output to GCLK_IO[1] */
		.bit.OOV = 0,			/* GCLK_IO[1] output value when generator is off */
		.bit.IDC = 1,			/* Generator duty cycle is 50/50 */
		.bit.GENEN = 1,			/* Enable the generator */
		.bit.SRC = 0x05,		/* Generator source: XOSC32K output */
		.bit.ID = GENERIC_CLOCK_GENERATOR_XOSC32K			/* Generator ID: 1 */
	};
	
	// Write settings
	GCLK->GENCTRL.reg = gclk1_genctrl.reg;
	
	// GENCTRL is Write-Synchronized...so wait for write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	// Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
	GCLK_CLKCTRL_Type gclk_clkctrl = 
	{
		.bit.WRTLOCK = 0,		/* Generic Clock is not locked from subsequent writes */
		.bit.CLKEN = 1,			/* Enable the Generic Clock */
		.bit.GEN = GENERIC_CLOCK_GENERATOR_XOSC32K, 	/* Generic Clock Generator 1 is the source */
		.bit.ID = 0x00			/* Generic Clock Multiplexer 0 (DFLL48M Reference) */
	};
	
	// Write settings
	GCLK->CLKCTRL.reg = gclk_clkctrl.reg;
	
	// Enable the DFLL48M in open loop mode.
	// PCLKSR.DFLLRDY must be one before writing to the DFLL Control register
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	SYSCTRL->DFLLCTRL.reg = (uint16_t)(SYSCTRL_DFLLCTRL_ENABLE);
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	
	// Set up the Multiplier, Coarse and Fine steps
	SYSCTRL_DFLLMUL_Type sysctrl_dfllmul = 
	{
		.bit.CSTEP = 31,		/* Coarse step - use half of the max value (63) */
		.bit.FSTEP = 511,		/* Fine step - use half of the max value (1023) */
		.bit.MUL = 1465			/* Multiplier = MAIN_CLK_FREQ (48MHz) / EXT_32K_CLK_FREQ (32768 Hz) */
	};
	
	// Write settings
	SYSCTRL->DFLLMUL.reg = sysctrl_dfllmul.reg;
	
	// Wait for synchronization
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	
	// Get factory calibrated value for "DFLL48M COARSE CAL" from NVM Software Calibration Area
	tempDFLL48CalibrationCoarse = *(uint32_t*)FUSES_DFLL48M_COARSE_CAL_ADDR;
	tempDFLL48CalibrationCoarse &= FUSES_DFLL48M_COARSE_CAL_Msk;
	tempDFLL48CalibrationCoarse = tempDFLL48CalibrationCoarse>>FUSES_DFLL48M_COARSE_CAL_Pos;
	
	// Write the coarse calibration value
	SYSCTRL->DFLLVAL.bit.COARSE = tempDFLL48CalibrationCoarse;
	
	// Switch DFLL48M to Closed Loop mode and enable WAITLOCK
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	SYSCTRL->DFLLCTRL.reg |= (uint16_t) (SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK);
	
	// Now that DFLL48M is running, switch CLKGEN0 source to it to run the core at 48 MHz.
	// Enable output of Generic Clock Generator 0 (GCLK_MAIN) to the GCLK_IO[0] GPIO Pin
	GCLK_GENCTRL_Type gclk_genctrl0 = 
	{
		.bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
		.bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
		.bit.OE = 1,			/* Enable generator output to GCLK_IO[0] */
		.bit.OOV = 0,			/* GCLK_IO[0] output value when generator is off */
		.bit.IDC = 1,			/* Generator duty cycle is 50/50 */
		.bit.GENEN = 1,			/* Enable the generator */
		.bit.SRC = 0x07,		/* Generator source: DFLL48M output */
		.bit.ID = GENERIC_CLOCK_GENERATOR_MAIN			/* Generator ID: 0 */
	};
	GCLK->GENCTRL.reg = gclk_genctrl0.reg;
	
	// GENCTRL is Write-Synchronized...so wait for write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	// Direct the GCLK_IO[0] output to PA28
	PORT_WRCONFIG_Type port0_wrconfig = 
	{
		.bit.HWSEL = 1,			/* Pin# (28) - falls in the upper half of the 32-pin PORT group */
		.bit.WRPINCFG = 1,		/* Update PINCFGy registers for all pins selected */
		.bit.WRPMUX = 1,		/* Update PMUXn registers for all pins selected */
		.bit.PMUX = 7,			/* Peripheral Function H selected (GCLK_IO[0]) */
		.bit.PMUXEN = 1,		/* Enable peripheral Multiplexer */
		.bit.PINMASK = (uint16_t)(1 << (28-16)) /* Select the pin(s) to be configured */
	};
	
	// Write these settings
	PORT->Group[0].WRCONFIG.reg = port0_wrconfig.reg;

	//Modify prescaler value of OSC8M to produce 8MHz output
	SYSCTRL->OSC8M.bit.PRESC = 0;		/* Prescale by 1 */
	SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;	/* Oscillator is always on if enabled */
	
	// Put OSC8M as source for Generic Clock Generator 3
	// Set the Generic Clock Generator 3 output divider to 1
	// Configure GCLK->GENDIV settings
	GCLK_GENDIV_Type gclk3_gendiv = 
	{
		.bit.DIV = 1,								/* Set output division factor = 1 */
		.bit.ID = GENERIC_CLOCK_GENERATOR_OSC8M		/* Apply division factor to Generator 3 */
	};
	
	// Write these settings
	GCLK->GENDIV.reg = gclk3_gendiv.reg;
	
	// Configure Generic Clock Generator 3 with OSC8M as source
	GCLK_GENCTRL_Type gclk3_genctrl = 
	{
		.bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
		.bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
		.bit.OE = 0,			/* Disable generator output to GCLK_IO[1] */
		.bit.OOV = 0,			/* GCLK_IO[2] output value when generator is off */
		.bit.IDC = 1,			/* Generator duty cycle is 50/50 */
		.bit.GENEN = 1,			/* Enable the generator */
		.bit.SRC = 0x06,		/* Generator source: OSC8M output */
		.bit.ID = GENERIC_CLOCK_GENERATOR_OSC8M			/* Generator ID: 3 */
	};
	
	// Write these settings
	GCLK->GENCTRL.reg = gclk3_genctrl.reg;
	// GENCTRL is Write-Synchronized...so wait for write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);

	//Set CPU and APBx BUS Clocks to 48MHz
	PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1 ;
	PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val ;
	PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val ;
	PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val ;
	
} // ClockSysInit48M()

void delay_n_cycles(unsigned long n)
{
	__asm (
	"loop: DMB	\n"
	"SUB r0, r0, #1 \n"
	"CMP r0, #0  \n"
	"BNE loop         "
	);
} // delay_n_cycles()

  // Function to initialize UART operation
void init_UART(uint32_t baud)
{
	// Enable bus clock to APBC mask
	REG_PM_APBCMASK |=  PM_APBCMASK_SERCOM3;

	// select UART clock
	GCLK->CLKCTRL.reg =  
	GCLK_CLKCTRL_ID(SERCOM3_GCLK_ID_CORE) |  // select the UART clock
	GCLK_CLKCTRL_CLKEN |                     // enable the clock
    GCLK_CLKCTRL_GEN(0);                     // set GCLK GEN(0)

	// Configure PA22 as TX Pin
	// Set pin direction to output
	PORT->Group[0].DIRSET.reg = (1 << 22);     
	
    // Enable PMUX
	PORT->Group[0].PINCFG[22].reg |= PORT_PINCFG_PMUXEN;   
	
	// Enable PMUX and set the PMUX bit, since pin is even we use PMUXE
	PORT->Group[0].PMUX[22>>1].bit.PMUXE = PORT_PMUX_PMUXE_C_Val;
	
	// Configure PA23 as RX Pin
	// Set pin direction to input
	PORT->Group[0].DIRCLR.reg = (1 << 23);       

	// Enable pull down resistor
	PORT->Group[0].PINCFG[23].reg &= ~PORT_PINCFG_PULLEN;   
	
	// Enable PMUX and set the PMUX bit, since pin is odd we use PMUX0
	PORT->Group[0].PINCFG[23].reg |= PORT_PINCFG_PMUXEN; 
	PORT->Group[0].PMUX[23>>1].bit.PMUXO = PORT_PMUX_PMUXO_C_Val; 
	
	// Configure USART via Control A and Control B
	SERCOM3->USART.CTRLA.reg =                  // USART is ASYNCHRONOUS
	   SERCOM_USART_CTRLA_DORD |                // Transmit LSB First
	   SERCOM_USART_CTRLA_MODE_USART_INT_CLK |  // Set Internal Clock 
	   SERCOM_USART_CTRLA_RXPO(1) |             // Use SERCOM pad 1 for data reception
	   SERCOM_USART_CTRLA_TXPO(0/*PAD0*/);      // Set SERCOM pad 0 for data transmission
	
	SERCOM3->USART.CTRLB.reg =        // We don't use PARITY
	    SERCOM_USART_CTRLB_RXEN |     // Enable receive when USART in enabled
		SERCOM_USART_CTRLB_TXEN |     // Enable transmit when USART is enabled
		SERCOM_USART_CTRLB_CHSIZE(0); // Set character size to 8 bits
	
	//Set USART Baud Rate
	// Baud rate is (65536) * (CPU_CLock - 16 * wanted baud) / CPU_Clock
	uint64_t baudRate = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;
	
	// Set Baud Rate
	SERCOM3->USART.BAUD.reg = (uint32_t)baudRate;
	
	// Enable the USART
	// SERCOM3 peripheral enabled
	SERCOM3->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
}  // UART3_Init()

// Function to write data using UART
void singlewrite_UART(char data)
{
	// Wait on interrupt flag and Write some data
	while(!(REG_SERCOM3_USART_INTFLAG) & 1)
	{
		
	}
	
	REG_SERCOM3_USART_DATA = data;
} //UART3_Write()


// Function to output data to serial monitor
void burstwrite_UART_Text(char *text)
{
	// Write text until EOL
	for(int i=0;text[i]!='\0';i++)
	{
		singlewrite_UART(text[i]);
	}
	
} // UART3_Write_Text()


// Function to check if data is present in UART
bool is_done_UARTwrite()
{
	// if data is present
	if ((SERCOM3->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) >> SERCOM_USART_INTFLAG_RXC_Pos)
    {
		// return true
		return true;
	}
	
	// else return false
	return false;
}  // UART3_Has_Data()

// Function to read UART data
char singleread_UART()
{
	// return data in the USART data register
	return SERCOM3->USART.DATA.reg;
}  // UART3_Read()

// Function to initialize ADC operation
void init_ADC(void)
{
	// Enable APBC clock for ADC
	REG_PM_APBCMASK |= PM_APBCMASK_ADC;
	while (!(REG_PM_APBCMASK & PM_APBCMASK_ADC));

	// Assign clock source to ADC
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_ADC | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY);

	// Reset ADC
	ADC->CTRLA.reg = ADC_CTRLA_SWRST;
	while (ADC->CTRLA.bit.SWRST || ADC->STATUS.bit.SYNCBUSY); // Wait for reset and sync to complete

	// Load calibration data
	ADC->CALIB.reg = ADC_CALIB_BIAS_CAL((*(uint32_t *)ADC_FUSES_BIASCAL_ADDR >> ADC_FUSES_BIASCAL_Pos)) |
	ADC_CALIB_LINEARITY_CAL((*(uint64_t *)ADC_FUSES_LINEARITY_0_ADDR >> ADC_FUSES_LINEARITY_0_Pos));

	// Set reference to VCC (3.3V)
	ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0;

	// Set sample length and prescaler
	ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(10);
	ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV32_Val;

	// Enable ADC
	ADC->CTRLA.reg = ADC_CTRLA_ENABLE;
	while (ADC->STATUS.bit.SYNCBUSY); // Wait for synchronization to complete
}


// Function to read a channel number for ADC conversion
int32_t adc_readchannel(uint8_t channel)
{
	// set positive MUX input selection
	ADC->INPUTCTRL.bit.MUXPOS = channel;

	// start conversion then flush
	// ADC will pick up where it left off
	ADC->SWTRIG.reg = ADC_SWTRIG_START | ADC_SWTRIG_FLUSH;

	// wait for analog conversion to complete
	while (ADC->INTFLAG.bit.RESRDY == 0)
	{

	}

	// return the result of the ADC
	return ADC->RESULT.reg;
}


int main(void)
{
	// Clock initialization
	init_clock();
	
    // Application code
		// Initialize the UART at 9600 baud
		init_UART(9600);
		delay_ms(500);

		// Debug message to indicate initialization is complete
		burstwrite_UART_Text("UART Initialized successfully at 9600 baud rate.\r\n");

		// Initialize the ADC
		init_ADC();
		delay_ms(100);
		burstwrite_UART_Text("ADC Initialized successfully.\r\n");

		// Variable to store the result of ADC conversion
		int result;
		
		// Main application
		while(1)
		{
			// Read the ADC channel connected to A0
			result = adc_readchannel(19); // A0 on the MKR Zero is connected to ADC channel 19 (PA11)

			// Check if the result is greater than zero before printing
			if (result >= 0) {
				// Convert the ADC result to a string
				char buffer[10];
				itoa(result, buffer, 10);

				// Send ADC reading over UART
				burstwrite_UART_Text("ADC Reading: ");
				burstwrite_UART_Text(buffer);
				burstwrite_UART_Text("\r\n");
			}
			// Small dely between readings
			delay_ms(100);
		}
    
}

