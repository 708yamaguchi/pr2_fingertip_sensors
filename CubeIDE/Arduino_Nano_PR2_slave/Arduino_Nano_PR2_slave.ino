#include <avr/io.h>
#include <avr/interrupt.h>
int cnt = 0;

int main(void)
{
  DDRB = (1<<PORTB4);     //MISO出力
  SPCR = (1<<SPIE)|(1<<SPE);  //割り込み有効
  sei();            //全体割り込み許可
    while (1)
    {
    }
}

SIGNAL(SPI_STC_vect){
  if (SPDR == 18)
  {
    cnt = 0;
  }
  if ((cnt % 2) == 0)
  {
    SPDR = 0; // upper 8 bits
  }
  else
  {
    SPDR = 100; // lower 8 bits
  }
  cnt++;
}
