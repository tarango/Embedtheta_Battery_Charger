#include <LiquidCrystal.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

LiquidCrystal lcd (2,3,4,7,8,10);


#define Full_charge			495
#define Max_auto_current	1000
#define Max_duty_cycle		254
#define Min_duty_cycle		0
#define DivideValue			2
#define FloatingVoltage		515
#define Nominal_voltage		460

#define lcd_delay			delay(100)

volatile unsigned char count=0;
volatile unsigned int Pot=0;
struct timer
{
	volatile unsigned char tick;
	volatile unsigned char check_flag;
	volatile unsigned int lcd_clear;
	volatile unsigned int reset_tick;
	}Timer;
struct battery
{
	volatile unsigned int B_volt[DivideValue];
	volatile unsigned int Temp_B_volt;
	volatile unsigned int Final_B_volt;
	}Battery;

struct current
{
	volatile unsigned Charge_Current[DivideValue];
	volatile unsigned int Temp_Charge_Current;
	volatile unsigned int Final_Charge_Current;
	}Current;
	
struct charge
{
	volatile signed int duty_cycle;
	}battery_charge;
	
struct ac_mains
{
	volatile unsigned int final_voltage;
	}AC_MAINS;
struct flag
{
	bool adv_flag=0;
	bool full_charge_flag=0;
	}Flag;
	


void timer_enable()
{
	//Timer interrupt for 1 sec
	//disable all interrupts
	noInterrupts();
	TCCR1A=0;
	TCCR1B=0;
	TCNT1=0;
	// set compare match register to desired timer count:
	OCR1A = 39;//7812 would be for 8Mhz//15500 for 16 Mhz// for *5msec*
	// turn on CTC mode:
	TCCR1B |= (1 << WGM12);
	// Set CS10 and CS12 bits for 1024 prescaler:
	TCCR1B |= (1 << CS10);
	TCCR1B |= (1 << CS12);
	//TimerA output Compare A match interrupt enable
	TIMSK |= (1<<OCIE1A); // For atmega8 it would be TIMSK |= (1<<OCIE1A)
	interrupts();
}
void setup()
{

  /* add setup code here */
  lcd.begin(16,2);//git test
  pinMode(9,OUTPUT);//git another test 
  pinMode(11,OUTPUT);	
  //analogWrite(9,128);
  lcd.setCursor(3,0);
  lcd_delay;
  lcd.print("EMBEDTHETA");
  lcd.setCursor(0,1);
  lcd_delay;
  lcd.print(" Battery Charger");
  delay(3000);
  //OCR2=191;
  TCCR2 |= (1 << WGM21) |(1 << WGM20) | (1 << COM21) | (1 << CS20); //Fast PWM, non inverting, no prescaling

	lcd.clear();
 timer_enable();
 for (count=0;count<DivideValue;count++)
 {
	Battery.B_volt[count]=0;	
	Current.Charge_Current[count]=0;
 }
 battery_charge.duty_cycle=100;
 Flag.full_charge_flag=0;
 sei();
}

void loop()
{

  /* add main program code here */

	  bool sw=digitalRead(6);//git comment
	  /*lcd.setCursor(10,1);
	  lcd.print(sw);
	  lcd.setCursor(0,0);
	  lcd.print(Pot);*/
	  switch (Timer.check_flag)
	  {
		case 1:
			/************************************************************************************************************/
			/********************************************************************************************************/
			/*Battery.Temp_B_volt/=DivideValue;
			for(count=0;count<DivideValue;count++)
			{
				Battery.B_volt[count]=Battery.B_volt[count+1];
			}
			Battery.B_volt[count]=Battery.Temp_B_volt;
			for (count=0,Battery.Final_B_volt=0,Battery.Temp_B_volt=0;count<DivideValue;count++)
			{
				Battery.Final_B_volt+=Battery.B_volt[count];
			}
			Battery.Final_B_volt/=DivideValue;*/
			
			Battery.Temp_B_volt=0;Battery.Final_B_volt=0;
			for(count=0;count<DivideValue;count++)
			{
				Battery.Temp_B_volt+=analogRead(A0);
			}
			Battery.Final_B_volt=Battery.Temp_B_volt;
			Battery.Final_B_volt/=DivideValue;
			
			/********************************************************************************************************/
			/************************************************************************************************************/
			//lcd.clear();
	
		
			/************************************************************************************************************/
			/**********************************************************************************************************/
			/*Current.Temp_Charge_Current/=DivideValue;
			for (count=0;count<DivideValue;count++)
			{
				Current.Charge_Current[count]=Current.Charge_Current[count+1];
			}
			Current.Charge_Current[count]=Current.Temp_Charge_Current;
			for (count=0,Current.Final_Charge_Current=0,Current.Temp_Charge_Current=0;count<DivideValue;count++)
			{
				Current.Final_Charge_Current+=Current.Charge_Current[count];
			}
			Current.Final_Charge_Current/=DivideValue;*/
			
			Current.Temp_Charge_Current=0;Current.Final_Charge_Current=0;
			for(count=0;count<DivideValue;count++)
			{
				Current.Temp_Charge_Current+=analogRead(A1);
			}
			Current.Final_Charge_Current=Current.Temp_Charge_Current;
			Current.Final_Charge_Current/=DivideValue;
			/************************************************************************************************************/
			/************************************************************************************************************/
			for (count=0,AC_MAINS.final_voltage=0;count<DivideValue;count++)
			{
				AC_MAINS.final_voltage+=analogRead(A4);
			}
			AC_MAINS.final_voltage/=DivideValue;
			
			/************************************************************************************************************/
			/************************************************************************************************************/
			switch (sw)
			{
				case 0:
					/*(Current.Final_Charge_Current>Pot)?(battery_charge.duty_cycle--):(battery_charge.duty_cycle++);
					(battery_charge.duty_cycle>=Max_duty_cycle)? (battery_charge.duty_cycle=Max_duty_cycle):(battery_charge.duty_cycle);
					(battery_charge.duty_cycle<=Min_duty_cycle)? (battery_charge.duty_cycle=Min_duty_cycle):(battery_charge.duty_cycle);
					OCR2=battery_charge.duty_cycle;*/
					battery_charge.duty_cycle=map(Pot,0,1023,0,255);
					OCR2=battery_charge.duty_cycle;
					/*lcd.clear();
					lcd.setCursor(6,0);
					lcd.print(battery_charge.duty_cycle);
					lcd.setCursor(10,1);
					lcd.print("Case0");*/
					lcd.setCursor(0,0);
					lcd_delay;
					lcd.print("B.V:");
					lcd_delay;
					lcd.print(Battery.Final_B_volt/40.43535);
					lcd_delay;
					lcd.print("V");
					//lcd.print (map(Battery.Final_B_volt,100,890,0,98));
					//lcd.print("%");
					
					lcd.setCursor(10,1);
					lcd.print((AC_MAINS.final_voltage)/2.03);
					break;
				case 1:	
					if (Battery.Final_B_volt <= Full_charge)
					{
						//(Current.Final_Charge_Current>Max_auto_current)?(battery_charge.duty_cycle--):(battery_charge.duty_cycle++);
						TCCR2 |= (1 << WGM21) |(1 << WGM20) | (1 << COM21) | (1 << CS20); //Fast PWM, non inverting, no prescaling
						(Current.Final_Charge_Current<Max_auto_current)?(battery_charge.duty_cycle++):(battery_charge.duty_cycle--);
						(battery_charge.duty_cycle>=Max_duty_cycle)? (battery_charge.duty_cycle=Max_duty_cycle):(battery_charge.duty_cycle);
						(battery_charge.duty_cycle<=Min_duty_cycle)? (battery_charge.duty_cycle=Min_duty_cycle):(battery_charge.duty_cycle);
						OCR2=battery_charge.duty_cycle;
						/*lcd.setCursor(0,1);
						lcd.print("Case1");
						lcd.setCursor(12,0);
						lcd.print(battery_charge.duty_cycle);*/
						
						/*lcd.setCursor(0,1);
						lcd.print(Flag.full_charge_flag);*/
						
						if (Flag.adv_flag==1  && Flag.full_charge_flag==0)
						{
							lcd.setCursor(0,0);
							lcd_delay;
							lcd.print("B.V:");
							lcd_delay;
							lcd.print((Battery.Final_B_volt)/33.8);
							lcd_delay;
							lcd.print("V, ");
							lcd_delay;
							lcd.print (map(Battery.Final_B_volt,350,450,0,98));
							lcd_delay;
							lcd.print("%");
							lcd_delay;
							lcd.setCursor(0,1);
							lcd.print("               ");
						}
						else if(Flag.adv_flag==0 && Flag.full_charge_flag==0)
						{
							lcd.setCursor(0,0);
							lcd.print("               ");
							lcd.setCursor(0,1);
							lcd_delay;
							lcd.print(battery_charge.duty_cycle);
							lcd.setCursor(8,1);
							lcd.print("AC:");
							lcd.setCursor(11,1);
							lcd.print((AC_MAINS.final_voltage)/2.03);
						}
						
						if (Battery.Final_B_volt < Nominal_voltage)
						{
							Flag.full_charge_flag=0;
						}
						
						
						/*lcd.setCursor(0,1);
						lcd.print("M.V:");
						lcd.print((AC_MAINS.final_voltage/3.72));
						lcd.print("V");*/
						
						
						/*lcd.print("C.C=");
						lcd_delay;
						lcd.print(0.0488*(Current.Final_Charge_Current-512));
						//lcd.print(Current.Final_Charge_Current);
						lcd_delay;
						lcd.print("A ");*/
						
					}
					else
					{
						//OCR2=128;
						(Battery.Final_B_volt>FloatingVoltage)?(battery_charge.duty_cycle--):(battery_charge.duty_cycle++);
						(battery_charge.duty_cycle>=Max_duty_cycle)? (battery_charge.duty_cycle=Max_duty_cycle):(battery_charge.duty_cycle);
						(battery_charge.duty_cycle<=Min_duty_cycle)? (battery_charge.duty_cycle=Min_duty_cycle):(battery_charge.duty_cycle);
						//OCR2=battery_charge.duty_cycle;
						digitalWrite(11,LOW);
						/*lcd.setCursor(0,1);
						lcd.print("Case2");
						lcd.setCursor(12,0);
						lcd.print(battery_charge.duty_cycle);*/
						
						/*lcd.setCursor(0,0);
						lcd_delay;
						lcd.print("B.V:");
						lcd_delay;
						lcd.print(Battery.Final_B_volt);
						lcd_delay;
						lcd.print("V, ");*/
						
						Flag.full_charge_flag=1;
						lcd.setCursor(0,0);
						lcd_delay;
						lcd.print("Full Charged    ");
						lcd.setCursor(0,1);
						lcd_delay;
						lcd.print("                ");
						
						if (Battery.Final_B_volt < Nominal_voltage)
						{
							Flag.full_charge_flag=0;
						}
						
						/*lcd.setCursor(0,1);
						lcd_delay;
						lcd.print("C.C=");
						lcd_delay;
						lcd.print(0.0488*(Current.Final_Charge_Current-512));
						//lcd.print(Current.Final_Charge_Current);
						lcd_delay;
						lcd.print("A ");
						
						lcd.setCursor(11,1);
						lcd.print((AC_MAINS.final_voltage)/2.03);*/
						
					}
					break;
				default:
					break;
			}
			/************************************************************************************************************/
			/************************************************************************************************************/
			Timer.check_flag=0;
			break;
			
		default:
			break;
	  }
	  
	
	
}

void(* resetFunc) (void) = 0;//declare reset function at address 0

ISR(TIMER1_COMPA_vect)
{
	Timer.tick++;
	Timer.lcd_clear++;
	Timer.reset_tick++;
	//Battery.Temp_B_volt+=analogRead(A0);
	//Current.Temp_Charge_Current+=analogRead(A1);
	Pot=analogRead(A2);
	switch (Timer.tick)
	{
		case DivideValue:
			 Timer.tick=0;
			 Timer.check_flag=1; 
			 break;

		default:
			break;
	}
	switch (Timer.lcd_clear)
	{
		case 5000:// 5 
			lcd_delay;
			lcd.clear();
			Timer.lcd_clear=0;
			LCD_CLEARDISPLAY;
			//resetFunc();
			Flag.adv_flag=!(Flag.adv_flag);
			break;
		
		default:
			break;
	}
	switch (Timer.reset_tick)
	{
		case 20000:
			Timer.reset_tick=0;
			//resetFunc();
			break;
		default:
			break;
	}
	
}

int analogNoiseReducedRead(int pinNumber)
{
	int reading;
	
	ADCSRA |= _BV( ADIE );             //Set ADC interrupt
	set_sleep_mode(SLEEP_MODE_ADC);    //Set sleep mode
	reading = analogRead(pinNumber);   //Start reading
	sleep_enable();                    //Enable sleep
	do
	{                                  //Loop until reading is completed
		sei();                           //Enable interrupts
		sleep_mode();                    //Go to sleep
		cli();                           //Disable interrupts
	} while(((ADCSRA&(1<<ADSC))!= 0)); //Loop if the interrupt that woke the cpu was something other than the ADC finishing the reading
	sleep_disable();                   //Disable sleep
	ADCSRA &= ~ _BV( ADIE );           //Clear ADC interupt
	sei();                             //Enable interrupts
	
	return(reading);
}