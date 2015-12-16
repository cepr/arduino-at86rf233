#include <mraa.h>
#include "at86rf2xx.h"

typedef unsigned char byte;
void setup();
void loop();
void at86rf2xx_eventHandler();
void at86rf2xx_receive_data();

int received = 0;

int IRQ = 15;
int RESET = 49;
int SLP_TR = 14;
int SEL = 110;
int SPI_BUS = 0;

int main() {
    setup();
    for(;;) {
        loop();
    }
}

void setup() {
  mraa_init();
  at86rf2xx.init(SEL, IRQ, SLP_TR, RESET, SPI_BUS);
  at86rf2xx.set_chan(26); // set channel to 26
}

void loop() {
  if (at86rf2xx.events)
    at86rf2xx_eventHandler();
  return;
}

void at86rf2xx_eventHandler() {
  /* One less event to handle! */
  at86rf2xx.events--;

  /* If transceiver is sleeping register access is impossible and frames are
   * lost anyway, so return immediately.
   */
  byte state = at86rf2xx.get_status();
  if(state == AT86RF2XX_STATE_SLEEP)
    return;

  /* read (consume) device status */
  byte irq_mask = at86rf2xx.reg_read(AT86RF2XX_REG__IRQ_STATUS);

  /*  Incoming radio frame! */
  if (irq_mask & AT86RF2XX_IRQ_STATUS_MASK__RX_START)
    printf("[at86rf2xx] EVT - RX_START\n");

  /*  Done receiving radio frame; call our receive_data function.
   */
  if (irq_mask & AT86RF2XX_IRQ_STATUS_MASK__TRX_END)
  {
    if(state == AT86RF2XX_STATE_RX_AACK_ON || state == AT86RF2XX_STATE_BUSY_RX_AACK) {
      printf("[at86rf2xx] EVT - RX_END\n");
      at86rf2xx_receive_data();
    }
  }
}

void at86rf2xx_receive_data() {
  /*  print the length of the frame
   *  (including the header)
   */
  size_t pkt_len = at86rf2xx.rx_len();
  printf("Frame length: %d bytes", (int)pkt_len);

  /*  Print the frame, byte for byte  */
  printf("Frame dump (ASCII):\n");
  uint8_t data[pkt_len];
  at86rf2xx.rx_read(data, pkt_len, 0);
  for (int d=0; d<pkt_len; d++)
    printf("%02x ", (int)data[d]);
  printf("\n");

  /* How many frames is this so far?  */
  printf("[[Total frames received: %d]]\n\n", ++received);
}
