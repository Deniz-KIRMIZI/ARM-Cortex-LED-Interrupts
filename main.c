#include "MKL25Z4.h"
#include "utils.h"
#define LedForRed   18  // Red LED pin is corresponding to 18th pin of the port to be selected (PTB18)
#define LedForGreen 19  // Green LED pin is corresponding to 18th pin of the port to be selected (PTB19)
#define LedForBlue   1   // Blue LED pin is corresponding to first pin of the port to be determined (PTD1)
#define ExSwitch 1 // PTA1
#define ExSwitch2 2 // pTA2
#define ExSwitch3 4 // PTD4

#define CounterLed1 7 // PTC7 to be used
#define CounterLed2 0 // PTC0 to be used
#define CounterLed3 3 // PTC3 to be used
#define CounterLed4 4 // PTC4 to be used

volatile uint8_t delay_block_action = 0;
		int BUTTON1, BUTTON2, BUTTON3;
void LedInitialization() {
    // Clock has been enabled for port b and d
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

    // Pins 18, 19 of portB and 4th of portd has been configured to be GPIO
    PORTB->PCR[LedForRed] |= PORT_PCR_MUX(1);
    PORTB->PCR[LedForGreen] |= PORT_PCR_MUX(1);
    PORTD->PCR[LedForBlue] |= PORT_PCR_MUX(1);

    // Pins 18, 19 of portB and 4th of portd has been configured to be output
    PTB->PDDR |= (1 << LedForRed) | (1 << LedForGreen);
    PTD->PDDR |= (1 << LedForBlue);
}

void ColorSelectRGB(uint8_t red, uint8_t green, uint8_t blue) {
    // Color selecter getting three inputs from user to determine/select a color
    if (red) PTB->PCOR = (1 << LedForRed);
    else PTB->PSOR = (1 << LedForRed);

    if (green) PTB->PCOR = (1 << LedForGreen);
    else PTB->PSOR = (1 << LedForGreen);

    if (blue) PTD->PCOR = (1 << LedForBlue);
    else PTD->PSOR = (1 << LedForBlue);
}

void SysTick_Handler(void) {
    delay_block_action = 1;
}

void milisecondsDelay(uint32_t milliseconds) {
    delay_block_action = 0;

    // systick configuration
    SysTick->LOAD = (SystemCoreClock / 1000) * milliseconds - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;

    // while the delay has been happening nothing else may take the process
    while (!delay_block_action) {
        // lock the cpu
    }

    SysTick->CTRL = 0; // systick has been disabled
}


void SwitchOnboardInterruptsA() {
    // Port A has acces to clock
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    // Pta0 has been configured to be GPIO
    PORTA->PCR[ExSwitch] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		PORTA->PCR[ExSwitch2] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    // falling edge interrrupt
    PORTA->PCR[ExSwitch] |= PORT_PCR_IRQC(10); // 1010 in irqc
		PORTA->PCR[ExSwitch2] |= PORT_PCR_IRQC(10);
    // interrupt activated
		NVIC_SetPriority(PORTA_IRQn, 128); // priority decided

    NVIC_EnableIRQ(PORTA_IRQn);
}

void BinaryCounterInterrupt() {
    // clock connected to port D
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    // Ptd0 has been configured to be GPIO
    PORTD->PCR[ExSwitch3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    // Configure falling edge interrupt on PTA0
    PORTD->PCR[ExSwitch3] |= PORT_PCR_IRQC(10); // Falling edge interrupt

    // interrupt enabled with secon priority
		NVIC_SetPriority(PORTD_IRQn, 64);

    NVIC_EnableIRQ(PORTD_IRQn);
}

void PORTA_IRQHandler(void) {
    // interrupt isr code wsill be running here
    // Falling edge will trigger
		if (PORTA->ISFR & (1 << ExSwitch)) {
			BUTTON1 = 1;
			PORTA->ISFR |= (1 << ExSwitch);
    // Clear the interrupt flag
		}
    if (PORTA->ISFR & (1 << ExSwitch2)) {
			BUTTON2 = 1;
		PORTA->ISFR |= (1 << ExSwitch2);
		}
}

int CounterControl(void){
			if ( BUTTON3 % 8 == 0){
			PTC->PTOR = (1 << CounterLed1);
		}
			if ( BUTTON3 % 4 == 0){
			PTC->PTOR = (1 << CounterLed2);
		}
			if ( BUTTON3 % 2 == 0){
			PTC->PTOR = (1 << CounterLed3);
			}
				if ( BUTTON3 % 1 == 0){
			PTC->PTOR = (1 << CounterLed4);
				}
}

		
void PORTD_IRQHandler(void) {
    
		BUTTON3++;
		CounterControl();

		Delay(300000);
    // flag has been triggered
    PORTD->ISFR |= (1 << ExSwitch3);
}

void InitializeCountByLED() {
    //clock has been anticipated
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // Configure Port C 's desired pins as GPIO output
    PORTC->PCR[CounterLed1] = PORT_PCR_MUX(1); // GPIO mode
		PORTC->PCR[CounterLed2] = PORT_PCR_MUX(1);
		PORTC->PCR[CounterLed3] = PORT_PCR_MUX(1);
		PORTC->PCR[CounterLed4] = PORT_PCR_MUX(1);
	
    // Configure Port C 's desired pins as output
    PTC->PDDR |= (1 << CounterLed1);
		PTC->PDDR |= (1 << CounterLed2);
		PTC->PDDR |= (1 << CounterLed3);
		PTC->PDDR |= (1 << CounterLed4);
	

    // Configure Port C 's desired pins as closed when started
    PTC->PCOR = (1 << CounterLed1);
		PTC->PCOR = (1 << CounterLed2);
		PTC->PCOR = (1 << CounterLed3);
		PTC->PCOR = (1 << CounterLed4);	
}

int main(void) {
    SwitchOnboardInterruptsA();
		LedInitialization();
		InitializeCountByLED();
		BinaryCounterInterrupt();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000); 
		while (1){
    while (BUTTON2 == 0) {
				if (BUTTON1 == 1){
					for (int i = 0; i < 5; i++){
						if (BUTTON2 == 1){
							ColorSelectRGB(0, 0, 0);  // Black
							milisecondsDelay(650);
							milisecondsDelay(650);
							milisecondsDelay(650);
							BUTTON2 = 0;
						}
				ColorSelectRGB(0, 0, 0);  // Black
        milisecondsDelay(650);
				milisecondsDelay(650);
							if (BUTTON2 == 1){
							ColorSelectRGB(0, 0, 0);  // Black
							milisecondsDelay(650);
							milisecondsDelay(650);
							milisecondsDelay(650);
							BUTTON2 = 0;
						}
        ColorSelectRGB(0, 1, 0);  // Green
        milisecondsDelay(650);
				milisecondsDelay(650);
		}
					BUTTON1 = 0;
	}
        ColorSelectRGB(0, 0, 0);  // Black
        milisecondsDelay(650);
				milisecondsDelay(650);
				if (BUTTON2 == 1){
					break;
				}
        ColorSelectRGB(1, 0, 0);  // Red
        milisecondsDelay(650);
				milisecondsDelay(650);
				
    }    
				ColorSelectRGB(0, 0, 0);
				milisecondsDelay(670);
				milisecondsDelay(660);
				milisecondsDelay(670);
				BUTTON2 = 0;
	}

    return 0;
}
