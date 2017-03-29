#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usbdrv.h"
#include "oddebug.h"


/* ------------------------------------------------------------------------- */

static uchar reportBuffer[8];
static uchar idleRate;

/* ------------------------------------------------------------------------- */

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] =
{
  0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
  0x15, 0x00,                    // LOGICAL_MINIMUM (0)
  0x26, 0xff, 0x00,              // LOGICAL_MAXIMUM (255)
  0x75, 0x08,                    // REPORT_SIZE (8)
  0x09, 0x04,                    // USAGE (Joystick)
  0xa1, 0x01,                    // COLLECTION (Application)
  0x09, 0x01,                    //   USAGE (Pointer)
  0xa1, 0x00,                    //   COLLECTION (Physical)
  0x09, 0x30,                    //     USAGE (X)
  0x09, 0x31,                    //     USAGE (Y)
  0x95, 0x02,                    //     REPORT_COUNT (2)
  0x81, 0x82,                    //     INPUT (Data,Var,Abs,Vol)
  0xc0,                          //   END_COLLECTION
  0xa1, 0x00,                    //   COLLECTION (Physical)
  0x09, 0x32,                    //     USAGE (Z)
  0x09, 0x33,                    //     USAGE (Rx)
  0x95, 0x02,                    //     REPORT_COUNT (2)
  0x81, 0x82,                    //     INPUT (Data,Var,Abs,Vol)
  0xc0,                          //   END_COLLECTION
  0x09, 0x34,                    //   USAGE (Ry)
  0x09, 0x35,                    //   USAGE (Rz)
  0x09, 0x36,                    //   USAGE (Slider)
  0x09, 0x37,                    //   USAGE (Dial)
  0x95, 0x04,                    //   REPORT_COUNT (4)
  0x81, 0x82,                    //   INPUT (Data,Var,Abs,Vol)
  0xc0                           // END_COLLECTION
};



/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar  usbFunctionSetup(uchar data[8]) {
  usbRequest_t    *rq = (void *)data;

  usbMsgPtr = reportBuffer;
  if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
    if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
      /* we only have one report type, so don't look at wValue */
      return sizeof(reportBuffer);
    }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
      usbMsgPtr = &idleRate;
      return 1;
    }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
      idleRate = rq->wValue.bytes[1];
    }
  }else{
    /* no vendor specific requests implemented */
  }
  return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void) {
  uchar       step = 128;
  uchar       trialValue = 0, optimumValue;
  int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

  /* do a binary search: */
  do{
    OSCCAL = trialValue + step;
    x = usbMeasureFrameLength();    /* proportional to current real frequency */
    if(x < targetValue)             /* frequency still too low */
      trialValue += step;
    step >>= 1;
  }while(step > 0);
  /* We have a precision of +/- 1 for optimum OSCCAL here */
  /* now do a neighborhood search for optimum value */
  optimumValue = trialValue;
  optimumDev = x; /* this is certainly far away from optimum */
  for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
    x = usbMeasureFrameLength() - targetValue;
    if(x < 0)
      x = -x;
    if(x < optimumDev){
      optimumDev = x;
      optimumValue = OSCCAL;
    }
  }
  OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void usbEventResetReady(void)
{
  /* Disable interrupts during oscillator calibration since
   * usbMeasureFrameLength() counts CPU cycles.
   */
  cli();
  calibrateOscillator();
  sei();
  eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

uchar readADC(){
  ADCSRA = (1<<ADEN|1<<ADSC|1<<ADIF|1<<ADPS2|1<<ADPS1);
  while (!(ADCSRA & (1<<ADIF)));
  return ADCH;
}

int main(void) {
  uchar i;

  usbDeviceDisconnect();
  for(i=0;i<20;i++){  /* 300 ms disconnect */
    _delay_ms(15);
  }
  usbDeviceConnect();
  
  DDRA =0;
  DDRB &=~(1<<4);

  if ((PINB & (1<<4)) ==0){
    PRR = 0x0F;
    MCUCR = (1<<SE) | (1<<SM1);
    asm("sleep");
  }
  
  wdt_enable(WDTO_1S);
  
  usbInit();
  sei();
  for(;;){
    wdt_reset();
    usbPoll();
    
    if(usbInterruptIsReady()){
      
      ADMUX = (1<<ADLAR | 3 ); //PA4
      reportBuffer[0] = readADC();

      ADMUX = (1<<ADLAR | 4 ); //PA5
      reportBuffer[1] = readADC();

      ADMUX = (1<<ADLAR | 5 ); //PA6
      reportBuffer[2] = readADC();

      ADMUX = (1<<ADLAR | 6 ); //PA7
      reportBuffer[3] = readADC();

      reportBuffer[4] = (PINA & 1<<0) ? 255:0;
      reportBuffer[5] = (PINA & 1<<1) ? 255:0;
      reportBuffer[6] = (PINA & 1<<2) ? 255:0;
      reportBuffer[7] = (PINA & 1<<3) ? 255:0;

      usbSetInterrupt(reportBuffer, sizeof(reportBuffer));

    }

  }
  return 0;
}