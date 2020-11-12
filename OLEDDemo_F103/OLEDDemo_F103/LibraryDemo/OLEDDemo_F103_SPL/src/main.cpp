//
//#include <stdio.h>
//#include "Core.h"
//
//#include "HardwareSerial.h"
//
//#include "GOFi2cOLED.h"
//GOFi2cOLED GOFoled;
//
///**********************************************************************/
//#define LED_PIN         2
//#define LED_PORT        GPIOB
//#define LED_PIN_SPEED   GPIO_Speed_2MHz
//
//#define LED_LOW()       BITMASK_SET(LED_PORT->BRR,  _BV(LED_PIN))
//#define LED_HIGH()      BITMASK_SET(LED_PORT->BSRR, _BV(LED_PIN))
//#define LED_TOGGLE()    BITMASK_TGL(LED_PORT->ODR,  _BV(LED_PIN))
//
///**********************************************************************/
//void GPIO_begin() {
//  BITMASK_SET(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);             // enable AFIO clock
//  BITMASK_SET(AFIO->MAPR,   AFIO_MAPR_SWJ_CFG_JTAGDISABLE);  // disable JTAG: free A15
//
//  BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_GPIOB);            // enable GPIOB clock
//  F103_GPIO_pinMode_output(LED_PORT, LED_PIN, GPIO_Mode_Out_PP | LED_PIN_SPEED);
//}
//
//
///**********************************************************************/
//void drawtriangleTest(void) {
//  for (int i=0; i<min(GOFoled.width(),GOFoled.height())/2; i+=5) {
//    GOFoled.drawTriangle(GOFoled.width()/2, GOFoled.height()/2-i,
//                     GOFoled.width()/2-i, GOFoled.height()/2+i,
//                     GOFoled.width()/2+i, GOFoled.height()/2+i, WHITE);
//    GOFoled.display();
//    delay(500);
//  }
//}
//
//void filltriangleTest(void) {
//  boolean color = WHITE;
//  for (int i=min(GOFoled.width(),GOFoled.height())/2; i>0; i-=5) {
//    GOFoled.fillTriangle(GOFoled.width()/2, GOFoled.height()/2-i,
//                     GOFoled.width()/2-i, GOFoled.height()/2+i,
//                     GOFoled.width()/2+i, GOFoled.height()/2+i, color);
//    color =~color;
//    GOFoled.display();
//    delay(500);
//  }
//}
//
//
//
///**********************************************************************/
//int main(void) {
//  /// Setup:
//  Core_begin();
//  delay(100);
//
//  Serial.begin(BAUD_1M);
//  Core_PrintInfo();
//
//  GPIO_begin();
//
//
//  /*------------------*/
//  // default slave address is 0x3C
//  GOFoled.init(0x3C);  //initialze  OLED display
//  GOFoled.display(); // show splashscreen
//  delay(2000);
////
////  GOFoled.setTextSize(2);
////  GOFoled.setTextColor(WHITE);
////  GOFoled.setCursor(0,0);
//////  GOFoled.println("Hello, world!");
//////  GOFoled.println(-1234);
//////  GOFoled.println(3.14159);
//////  GOFoled.setTextColor(BLACK, WHITE); // 'inverted' text
//////  GOFoled.println(3.14159,5);
//////  GOFoled.setTextSize(2);
//////  GOFoled.setTextColor(WHITE);
//////  GOFoled.print("0x"); GOFoled.println(0xDEADBEEF, HEX);
////  GOFoled.write('A');
////  GOFoled.write('B');
////  GOFoled.write('C');
////  GOFoled.display();
//  /*------------------*/
//
//  /// Loop:
//  int x = 0;
//  double y = -123.4567;
//
//  while (1) {
//    GOFoled.clearDisplay();
//    drawtriangleTest();
//    filltriangleTest();
//  }
//
//
//  while (1) {
//    GOFoled.clearDisplay();
//    GOFoled.setTextSize(2);
//    GOFoled.setTextColor(WHITE);
//    GOFoled.setCursor(0,0);
//    GOFoled.println(x);
//    GOFoled.display();
//
//
//    if (Serial.available() > 0) {
//      while (Serial.available() > 0) {
//        Serial.print( (char)Serial.read() );
//        delay_us(500);
//      }
//
//      Serial.println();
//      LED_TOGGLE();
//    }
//
//    Serial.print("millis() = ");
//    Serial.print(millis());
//    Serial.print("\t");
//
//    Serial.print("x = ");
//    Serial.print(x, HEX);
//    Serial.print("\t");
//
//    Serial.print("y = ");
//    Serial.print(y, 3);  // print float & digit
//    Serial.println();
//
//    x += 1;
//    y += 0.01;
//
//    LED_TOGGLE();
//    delay(250);
//  }
//}


/* Includes -----------------------------------------------------------*/
#include "Core.h"
#include "GOFi2cOLED.h"
#include <stdio.h>
/* Private typedef ----------------------------------------------------*/

/* Private define -----------------------------------------------------*/
#define dataDHT11 GPIO_Pin_7
GOFi2cOLED GOFoled;
/* Private macro ------------------------------------------------------*/
/* Private variables --------------------------------------------------*/

/* Private functions --------------------------------------------------*/
void config_GPIO(void);
void config_TIMER2(void);
void setup(void);
int getData(int byteData[4]);

void config_GPIO(void) {
    GPIO_InitTypeDef GPIO_Struct;

    //config gpio port B
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_Struct.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Struct.GPIO_Pin  = dataDHT11;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_Struct);

    GPIO_SetBits(GPIOA, dataDHT11);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Struct.GPIO_Pin  = GPIO_Pin_7;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_Struct);

}

void config_TIMER2(void) {
    //config timer2
    TIM_TimeBaseInitTypeDef TimerInit;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TimerInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerInit.TIM_Period = 0xFFFF;
	TimerInit.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseInit(TIM2, &TimerInit);
	TIM_Cmd(TIM2, ENABLE);
}

void setup(void) {
    config_GPIO();
    config_TIMER2();
}

int getData(int byteData[4]) {

    uint8_t numberOfTimer;
	uint8_t data[5];

    // MCU sends out start signal
	GPIO_ResetBits(GPIOA, dataDHT11);
	delay_ms(20);
	GPIO_SetBits(GPIOA, dataDHT11);

	// wait pin dataDHT11 pulled up high level
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 10) {
		if (GPIO_ReadInputDataBit(GPIOA, dataDHT11)) {
			break;
		}
	}
	numberOfTimer = TIM_GetCounter(TIM2);
	if (numberOfTimer >= 10) {
		return 0;
	}

	// wait pin dataDHT11 pulled up low level
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 45) {
		if (!GPIO_ReadInputDataBit(GPIOA, dataDHT11)) {
			break;
		}
	}
	numberOfTimer = TIM_GetCounter(TIM2);
	if ((numberOfTimer >= 45) || (numberOfTimer <= 5)) {
		return 0;
	}

	// wait pin dataDHT11 pulled up high level
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 90) {
		if (GPIO_ReadInputDataBit(GPIOA, dataDHT11)) {
			break;
		}
	}
	numberOfTimer = TIM_GetCounter(TIM2);
	if ((numberOfTimer >= 90) || (numberOfTimer <= 70)) {
		return 0;
	}

	// wait pin dataDHT11 pulled up low level
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 95) {
		if (!GPIO_ReadInputDataBit(GPIOA, dataDHT11)) {
			break;
		}
	}
	numberOfTimer = TIM_GetCounter(TIM2);
	if ((numberOfTimer >= 95) || (numberOfTimer <= 75)) {
		return 0;
	}

	// receive data byte from DHT11
    for (int numberOfByte = 0; numberOfByte < 5; numberOfByte++) {

        for (int i = 0; i < 8; ++i) {
            // wait pin dataDHT11 pulled up high level
            TIM_SetCounter(TIM2, 0);
            while (TIM_GetCounter(TIM2) < 65) {
                if (GPIO_ReadInputDataBit(GPIOA, dataDHT11)) {
                    break;
                }
            }
            numberOfTimer = TIM_GetCounter(TIM2);
            if ((numberOfTimer >= 65) || (numberOfTimer <= 45)) {
                return 0;
            }

            // wait pin dataDHT11 pulled up low level
            TIM_SetCounter(TIM2, 0);
            while (TIM_GetCounter(TIM2) < 80) {
                if (!GPIO_ReadInputDataBit(GPIOA, dataDHT11)) {
                    break;
                }
            }
            numberOfTimer = TIM_GetCounter(TIM2);
            if ((numberOfTimer >= 80) || (numberOfTimer <= 10)) {
                return 0;
            }

            // check bit received is '1' or '0'
            data[numberOfByte] <<= 1;
            if (numberOfTimer > 45) {   // received bit '1'
                data[numberOfByte] |= 1;
            }
            else {                      // received bit '0'
                data[numberOfByte] &= ~1;
            }
        }
    }

	int checkSum = data[0] + data[1] + data[2] + data[3];
	if (checkSum != data[4]) {
		return 0;
	}

	for (int numberOfByte = 0; numberOfByte < 4; ++numberOfByte) {
		byteData[numberOfByte] = data[numberOfByte];
	}

	return 1;
}

int main(void) {
    Core_begin();

    setup();

    GOFoled.init(0x3C);  //initialze  OLED display
    GOFoled.clearDisplay();
    int arrData[4];

    /* Infinite loop */
    while (1) {
        if (getData(arrData)) {
            GOFoled.clearDisplay();
            GOFoled.setTextSize(2);
            GOFoled.setTextColor(WHITE);
            GOFoled.setCursor(0, 20);
            GOFoled.print("Humi:");
            GOFoled.print(arrData[0], DEC);
            GOFoled.println("%");
            GOFoled.print("Temp:");
            GOFoled.print(arrData[2], DEC);
            GOFoled.setTextSize(1);
            GOFoled.print("o");
            GOFoled.setTextSize(2);
            GOFoled.println("C");
            GOFoled.display();
        }
        else {
            GOFoled.setTextSize(2);
            GOFoled.setTextColor(WHITE);
            GOFoled.setCursor(0, 0);
            GOFoled.println("ERROR!!!");
            GOFoled.display();
        }
		delay_ms(500);
        GOFoled.clearDisplay();
    }
}
