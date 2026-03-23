// Lab 8 - Analog Heartbeat Measurement 
//
//Written by Jeffrey McCormick
// With assistance from Dr. Larry Aamodt, Dr. Natalie Smith-Gray,
// Peter Kyle, and Google Gemini 2.5


#include <MKL25Z4.h>
#include <string.h>
#include <stdio.h>

#define CLOCK_SETUP 1		// 48MHz DEFAULT_SYSTEM_CLOCK
#define MASK(x) (1UL << (x))

#define ADC_POS		(20)	// PTE20
#define DAC_POS		(31)	// PTE31 and 30 on ENGR355 Dev board

#define R_LED (8)
#define Y_LED (9)
#define G_LED (10)		//Port-C Pin 10

// Button Pin Definitions
#define SW1 (5)   // PTA5
#define SW2 (2)   // PTA2
#define SW3 (13)  // PTA13
#define SW4 (12)  // PTA12
#define SW5 (1)   // PTA1

// Global Variables
int DAC_RES= 1<<12; 									// 12-bit DAC 2^12 values
// Mode 0: Simulation (DAC generates pulse for loopback logic testing)
// Mode 1: Scope (DAC reconstructs ADC input for oscilloscope viewing and hardware testing)
volatile int dac_mode = 1;
volatile uint32_t n_counts = 0;				// Note: Freq_pit = 1000 and counts >59999 --> BPM = 0 (beat period is greater than 1 min.)
float Freq_pit = 1000;								//Hz
																			// T_pit = 1/Freq_pit -> Multiplication reduces computation time.
float T_pit = 0.001;									//sec	Effectively the sampling resolution
volatile uint32_t pit_count = 0; 			// Incremented by ISR
volatile uint32_t system_ms = 0; 			// Always increments, never resets
volatile int beat_trig = 0;						// 0=False, 1=True Link to button/sim for Debug
volatile int waiting_for_first_beat = 1;		// Tracks state in HB_program's get_heartbeat()
volatile int program_start = 0;				// 0=False, 1=True Linked to button
char line0[16]; 
char line1[16];
uint32_t last_n = 0;
int last_start_state = -1; 						// Track if the user just toggled the button
// Selectable program flags (Only set one at a time)
int HB_prog = 1;
int program2 = 0;
int program3 = 0;


	
// Prototypes
void Init_DAC(void);
void DAC_Scope(uint16_t adc_value);
void Init_SysTick(void);
void Init_ADC(void);
void Init_Buttons(void);
void Init_LEDs(void);
void LCD_init(void);
void LCD_command(uint32_t command);       // use to send LCD commands
void LCD_send_data(uint32_t data);        // use to send one char to screen
void delayMs(uint32_t n);
void get_heartbeat(void);

void ENABLE_CLKS(void) {
	
	// Enable clock to Ports A, B, C, and E
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	}
void Init_SysTick(void) {			// For Heartbeat Simulator
    // SystemCoreClock is 48 MHz when CLOCK_SETUP == 1
    // We want a 10 kHz sample rate (10,000 interrupts per second)
    // 48,000,000 / 10,000 = 4,800 ticks between interrupts
    SysTick_Config(SystemCoreClock / 1000);
		NVIC_SetPriority(SysTick_IRQn, 3); // Lower priority than PIT (2)
}
void Init_Buttons(void) {
    // Enable clock to Port A if no otherwise enabled
    //SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    // 1. Configure SW1 for: GPIO + Pull-up + Falling Edge Interrupt (0xA)
    PORTA->PCR[SW1] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0x0A);
    
    // 2. Configure other buttons as normal GPIO + Pull-up (no interrupts for now)
    uint32_t other_btn_cfg = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTA->PCR[SW2] = other_btn_cfg;
    PORTA->PCR[SW3] = other_btn_cfg;
    PORTA->PCR[SW4] = other_btn_cfg;
    PORTA->PCR[SW5] = other_btn_cfg;

    // 3. Set Port A pins to input
    PTA->PDDR &= ~(MASK(SW1) | MASK(SW2) | MASK(SW3) | MASK(SW4) | MASK(SW5));

    // 4. Enable Port A Interrupts in NVIC
    NVIC_SetPriority(PORTA_IRQn, 0); // Higher priority than PIT
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
}
void PORTA_IRQHandler(void) {
    // 1. CLEAR FLAG IMMEDIATELY to prevent "ghost" pending interrupts
    uint32_t flags = PORTA->ISFR;
    PORTA->ISFR = flags; 

    if (flags & MASK(SW1)) {
        static uint32_t last_press_time = 0;
        uint32_t current_time = system_ms;
				/* Only act if 20ms has passed since the last valid press
         *(Since your PIT_Init is 0.001s, 20 counts = 20ms)
				 * 150ms is usually safe for human presses/releases
				*/
        if ((current_time - last_press_time) > 300) {
            program_start = !program_start;
            last_press_time = current_time;
        }
    }
}


void Init_LEDs(void){
	// Enable clock for poit C if not otherwise enabled
	//SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	// Set LED pins to GPIO
	PORTC->PCR[R_LED] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[R_LED] |= PORT_PCR_MUX(1);
	PORTC->PCR[Y_LED] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[Y_LED] |= PORT_PCR_MUX(1);
	PORTC->PCR[G_LED] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[G_LED] |= PORT_PCR_MUX(1);
	// Set LED pins to outputs (1)
	PTC->PDDR |= MASK(R_LED)| MASK(Y_LED) | MASK(G_LED);
	// Initialize LEDs in off state by setting the port bits to zeros
	PTC->PCOR |= MASK(R_LED)| MASK(Y_LED) | MASK(G_LED);
	
}
void PIT_Init(float desired_period) {
		/*
		 *@param desired_period must be < 89.4 seconds
		*/
    // Enable clock to PIT module
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    
    // Turn on PIT (MCR[MDIS] = 0)
    PIT->MCR &= ~PIT_MCR_MDIS_MASK;
    //Freeze timers when in debug mode
		PIT->MCR |= PIT_MCR_FRZ_MASK;
		// No chaining
		PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_CHN_MASK;
	
    // Set load value for 1ms interrupt
    // Formula: (Bus Clock Frequency / Desired Frequency) - 1
    // (24,000,000 / 1000) - 1 = 47,999
		// Alt: (Sys Clock Frequency/2 * Desired Period) - 1
		uint32_t clk_cycles =  (uint32_t)(DEFAULT_SYSTEM_CLOCK * 0.5 * desired_period) - 1;
		// Initialize PIT0 to count down from argument 
		//PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(countsInPeriod);
    PIT->CHANNEL[0].LDVAL = clk_cycles;
    
    // Enable Interrupt and Start Timer (TIE and TEN)
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
    
    // Enable PIT IRQ in NVIC
    NVIC_ClearPendingIRQ(PIT_IRQn);
    NVIC_SetPriority(PIT_IRQn, 2);
    NVIC_EnableIRQ(PIT_IRQn);
}
void PIT_IRQHandler(void) {
    // Check if interrupt flag is set for Channel 0
    if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
        // Clear the flag by writing 1 to it
        PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
        
        // Increment our counters
        system_ms++; // This is our "Wall Clock" for debouncing
        pit_count++; // This is our "Stopwatch" for the selected program
			
			// Visual TEST ONLY: Toggle the Green LED every interrupt
      // Use PTOR (Port Toggle Output Register) for a simple flip
      PTC->PTOR = MASK(G_LED);
    }
}

int get_bpm(uint32_t counts) {
	//Calculates the time period based on PIT cycles
    int bpm = 0;
    // 60,000 ms per minute. 
    // Assuming Freq_pit = 1000 Hz (1 ms per count)
    if (counts > 0) {
        bpm = 60000 / counts; 
    }
    if (bpm > 999) {
        bpm = 999; // Cap overflow
    }
    return bpm;
}
void displayString(char* str, int line) {
	/**
 * Displays a string on the specified LCD line.
 * @param str:  The string to display (max 8 chars)
 * @param line: 0 for the top line, 1 for the bottom line
 */
    // 1. Move cursor to the start of the requested line
    if (line == 0) {
        LCD_command(0x80); // Line 0 (t0p) address
    } else {
        LCD_command(0xC0); // Line 1 (bottom) address
    }

    // 2. Print up to 8 characters, or until the end of string
    int i = 0;
    while (str[i] != '\0' && i < 8) {
        LCD_send_data(str[i]);
        i++;
    }

    // 3. Optional: Fill remaining spaces with blanks to clear old data
    while (i < 8) {
        LCD_send_data(' '); 
        i++;
    }
}
void SysTick_Handler(void) {	// Heartbeat Simulator
    static uint32_t tick_count = 0;
    tick_count++;

    // 1000ms period (60 BPM)
    if (tick_count >= 1000) {
        tick_count = 0;
        beat_trig = 1;      // Signal to get_heartbeat() logic
        PTC->PSOR = MASK(Y_LED);
        
        // Output 3.3V (4095) to DAC
        DAC0->DAT[0].DATL = DAC_DATL_DATA0(0xFF);
        DAC0->DAT[0].DATH = DAC_DATH_DATA1(0x0F);
    } 
    // Pulse width: Keep high for 50ms
    else if (tick_count == 50) {
        beat_trig = 0;
        PTC->PCOR = MASK(Y_LED);
        
        // Output 0V to DAC
        DAC0->DAT[0].DATL = DAC_DATL_DATA0(0);
        DAC0->DAT[0].DATH = DAC_DATH_DATA1(0);
    }
}
void get_heartbeat(void) {
    static int last_trig_state = 0;

    // Only operate if both master start and heartbeat program are active
    if (program_start && HB_prog) {
        
        // Edge Detection: Look for the moment beat_trig goes from 0 to 1
        if (beat_trig == 1 && last_trig_state == 0) {
						// ATOMIC ENTRY: Snap the current count instantly
            uint32_t current_capture = pit_count;
					
            if (waiting_for_first_beat) {
                // First beat: Just sync the timer and start counting
                pit_count = 0;
                waiting_for_first_beat = 0;
            } else {
                // Subsequent beats: Capture the interval and reset immediately
                n_counts = current_capture; 
                pit_count = 0;
            }
        }
        last_trig_state = beat_trig;
    } else {
        // Reset state when program is stopped or switched
        waiting_for_first_beat = 1;
        last_trig_state = 0;
    }
}



void Init_ADC(void) {
    // Enable clocks to ADC0 and Port E [cite: 892, 893]
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Configure Port E Pin 20 for ADC input [cite: 897, 899]
    PORTE->PCR[ADC_POS] &= ~PORT_PCR_MUX_MASK; 
    PORTE->PCR[ADC_POS] |= PORT_PCR_MUX(0);

    // 12-bit conversion, clock divided by 4 [cite: 904, 906]
    ADC0->CFG1 = ADC_CFG1_ADIV(2) | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0);
    
    // Select VREFH/VREFL as reference [cite: 912]
    ADC0->SC2 = ADC_SC2_REFSEL(0);

    // Enable conversion complete interrupts [cite: 914, 916]
    ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(0); // Start first conversion;

    // Enable ADC IRQ in NVIC [cite: 891]
    NVIC_EnableIRQ(ADC0_IRQn);
		NVIC_SetPriority(ADC0_IRQn, 1);
}

void ADC0_IRQHandler(void) {
    uint16_t adc_val = ADC0->R[0];
		// For Lab 8
    // const uint16_t threshold = 140; // (0.75/3.3)*4095
		// const uint16_t hysteresis_low = 62; // ~680mV
		// For Heart Monitor
	  const uint16_t threshold = 1923; // (1.55/3.3)*4095
		const uint16_t hysteresis_low = 1675; // (1.35/3.3)*4095
	
    // Detection Logic
		// Only trigger if (x)ms has passed since last beat (refractory period)
    if (adc_val >= threshold && (pit_count > 2 || waiting_for_first_beat)) {
			if (beat_trig == 0) {
				beat_trig = 1;
        PTC->PSOR = MASK(R_LED);
			}
    } 
		// Clear trigger only when signal drops below hysteresis level
		else if (adc_val < hysteresis_low && beat_trig == 1) { // Hysteresis to clear trigger
        beat_trig = 0;
        PTC->PCOR = MASK(R_LED);
    }

    // DAC Reconstruction (Scope Mode)
    if (dac_mode == 1) {
        DAC0->DAT[0].DATL = DAC_DATL_DATA0(adc_val & 0xFF);
        DAC0->DAT[0].DATH = DAC_DATH_DATA1((adc_val >> 8) & 0x0F);
    }

    // Trigger next conversion
    ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(0);
}
void Init_DAC(void) {
    // Enable clock to DAC0 [cite: 740]
    SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK;

    // Set pin to DAC output mode [cite: 746]
    PORTE->PCR[DAC_POS] &= ~PORT_PCR_MUX_MASK;

    // Enable DAC and select reference [cite: 756]
    DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
}




// Example usage for your heartbeat monitor:
int main(void) {
    // Standard Initializations...
		// Use Init_Systick() for Simulation
    ENABLE_CLKS();
		LCD_init();
		Init_LEDs();
		Init_Buttons();
		PIT_Init(T_pit);
		Init_ADC();
		Init_DAC();

		if (dac_mode == 0) Init_SysTick(); // Only simulate if in mode 0

    while(1) {
				get_heartbeat();
			
        // 1. ATOMIC TRANSITION HANDLER
        if (program_start != last_start_state) {
            if (program_start) {
                // RESTART: Clear everything to prevent "stale" data math
                last_n = 0;
                waiting_for_first_beat = 1;
                displayString("WAITING", 0);
								displayString("       ", 1);
            } else {
                displayString("MANUALLY", 0);
                displayString("STOPPED", 1);
            }
            last_start_state = program_start;
        }

        // 2. PROTECTED RUNNING LOGIC
        if (program_start && HB_prog) {
            // CRITICAL: Do not perform math until the first beat is actually captured
            // This prevents Divide-by-Zero and illegal memory access on restart
						if (!waiting_for_first_beat && n_counts != last_n) {
                __disable_irq(); 
								uint32_t safe_counts = n_counts; 
								__enable_irq();  // Unlock immediately after snapping the data
        
								int bpm = get_bpm(safe_counts);
							
                // Create a string limited to the memory assigned to the lines
                __disable_irq();
								snprintf(line0, sizeof(line0), "n:%-6lu", (unsigned long)n_counts);
                snprintf(line1, sizeof(line1), "BPM:%-4d", bpm);
								__enable_irq();
							
                displayString(line0, 0);
                displayString(line1, 1);
							
                //__enable_irq();  // Unlock the door
                last_n = n_counts;
            }
        }
    }
}

