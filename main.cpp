#include "Arduino.h"
#include "FXUtil.h"		// read_ascii_line(), hex file support
extern "C" {
  #include "FlashTxx.h"		// TLC/T3x/T4x/TMM flash primitives
}

const int led = LED_BUILTIN;	// LED pin
Stream *serial = &Serial;	// Serial (USB) or Serial1, Serial2, etc. (UART)

#define HEX_FILE_NAME "FlasherX.ino.hex"
#define FIRMWARE_VERSION "v0.2"
void blink_short() {
  digitalWrite( led, HIGH );
  delay(200);
  digitalWrite( led, LOW );
  delay(200);
}
void blink_long() {
  digitalWrite( led, HIGH );
  delay(600);
  digitalWrite( led, LOW );
  delay(200);
}

void setup ()
{

  ((HardwareSerial*)serial)->begin( 115200 );


  pinMode( led, OUTPUT );
  blink_short();
  for ( int i = 0; i < 4; i++ ){
    blink_long();
  }


  serial->printf( "%s - %s %s\n", FIRMWARE_VERSION, __DATE__, __TIME__ );
  delay(100);
  serial->printf( "WARNING: this can ruin your device!\n" );
  delay(100);
  serial->printf( "target = %s (%dK flash in %dK sectors)\n",
			FLASH_ID, FLASH_SIZE/1024, FLASH_SECTOR_SIZE/1024);

}



void loop ()
{
  uint32_t buffer_addr, buffer_size;
  // create flash buffer to hold new firmware
  if (firmware_buffer_init( &buffer_addr, &buffer_size ) == 0) {
    serial->printf( "unable to create buffer\n" );
    serial->flush();
    for (;;) {}
  }

  serial->printf( "created buffer = %1luK %s (%08lX - %08lX)\n",
		buffer_size/1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
		buffer_addr, buffer_addr + buffer_size );

  int user_input = -1;
  char line[32];
  serial->printf( "enter opcodes via serial" );
  while (true) {
    blink_short();
    read_ascii_line( serial, line, sizeof(line) );
    sscanf( line, "%d", &user_input );
    if (user_input == 1) { // serial
      // read hex file, write new firmware to flash, clean up, reboot
      if (update_firmware(serial, serial, buffer_addr, buffer_size)) {
        serial->printf("Firmware update failed!\n");
        firmware_buffer_free(buffer_addr, buffer_size);
        serial->flush();
        REBOOT;
      }
    }
    if ( user_input == 2) {
      firmware_buffer_free( buffer_addr, buffer_size );
      serial->printf("rebooting...\n");
      serial->flush();
      REBOOT;
    }

    if ( user_input == 3) {
      commit_update( serial, serial,buffer_addr);
    }

    if ( user_input == 4) {
      serial->printf( "%s - %s %s\n", FIRMWARE_VERSION, __DATE__, __TIME__ );
      for ( int i = 0; i < 4; i++ ){
        blink_short();
      }
    }
  }


  /*
  // return from update_firmware() means error or user abort, so clean up and
  // reboot to ensure that static vars get boot-up initialized before retry
  serial->printf( "erase FLASH buffer / free RAM buffer...\n" );
  firmware_buffer_free( buffer_addr, buffer_size );
  serial->flush();
  REBOOT;
  */
}