/*H27 8/14~
 *エフェクター
 *Atmega328p使用
 *動作周波数20MHz(外部セラミック発振子,分周なし)
 *エフェクトは"ディレイ","クリッピング","エコー"を予定
 *ヒューズ設定
 fL0xa6 fH0xd9 fX0x07
 外部セラミック振動子外部クロック出力
*/

#include <avr/io.h>
#include <math.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/*AD変換用定義*/
#define is_SET(x,y) ((x) & (1<<(y)))

/*シリアル通信用定義*/
#define FOSC   20000000//動作周波数
#define BAUD   9600//ボーレート
#define MYUBRR (FOSC/16/BAUD)-1

/* シリアル通信ｺﾏﾝﾄﾞ（ｱｽｷｰｺｰﾄﾞ)*/
#define LF        usart_tx(0x0a)        //改行
#define CR        usart_tx(0x0d)        //復帰
#define HT        usart_tx(0x09)        //水平ﾀﾌﾞ

#define smp_f 44000//サンプリング周波数(Hz) ICRの設定値より

/*変数宣言*/
volatile float  ICR1_buf = 0;
volatile float  ad_buf = 0;
volatile uint8_t spi_buf = 0;
volatile uint8_t serial_buf = 0;
//volatile uint8_t adc_delay_width = 0;
volatile uint8_t add_top_w=0;
volatile uint8_t add_middle_w=0x22;
volatile uint8_t add_low_w=0;
volatile uint8_t add_top_r=0;
volatile uint8_t add_middle_r=0;
volatile uint8_t add_low_r=0;
volatile uint16_t add_check_w=0;
volatile uint16_t add_check_r=0;

//uint8_t add_low=0;
/*外付けSRAM下位8bitは毎回8bitのデータを入れるので常に0x00*/

/*関数宣言*/
void spi_ini(){//spi通信設定
  //CSはPD2ピン
  SPCR|=_BV(SPE)|_BV(MSTR);
  /*  SPIE    : SPI割り込み許可
	 SPE     : SPI許可(SPI操作を許可するために必須)
	 DORD    : データ順選択,1:LSBから 0:MSBから
	 MSTR    : 1:主装置動作 0:従装置動作
	 CPOL    : SCK極性選択
	 CPHA    :　SCK位相選択
	 SPR1,SPR0 : 00:SCK周波数=fosc/4
  */
  /*SPI状態レジスタ SPSR
    SPIF    : SPI割り込み要求フラグ 転送完了時1
    WCOL    :上書き発生フラグ
  */
  /*SPIデータレジスタ　SPDR
    8bit
    7 6 5 4 3 2 1 0
    (MSB)       (LSB)
  */
}
void spi_send(uint8_t spi_data){
  //puts_tx("1");
  volatile  uint8_t dummy = 0;
  dummy = SPDR;
  SPDR = spi_data;
  while(!(SPSR&(1<<SPIF)));//転送完了まで待機
  dummy = SPDR;
}

unsigned int spi_get(void){
  //puts_tx("2");
  volatile uint8_t dummy = 0;
  SPDR = dummy;
  while(!(SPSR&(1<<SPIF)));//転送完了まで待機
  return SPDR;
    
}
/*高速PWMではTOP値がOCRA、比較値がOCR0Bとなる*/
void adc_ini(){
  ADMUX |=_BV(ADLAR);
  /*REFS1 REFS0 : 00 Arefピンの外部基準電圧
    ADPS2 ADPS1 ADPS0 :000 CK/2 ~= 100000Hz(変換クロック)
    MUX3 MUX2 MUX1 MUX0: 0000ADC0ピン
  */
    
  ADCSRA|= _BV(ADEN)|_BV(ADPS1);
  /*ADEN :1 A/D許可
    ADPS2 ADPS1 ADPS0 :010 CK/4 ~= 50000Hz(変換クロック),それ以上だとうまくできない？？
    _BV(ADSC)によって起動 単独変換動
  */
    
  /*ADCSRA|=_BV(ADEN)|_BV(ADATE);
    ADCSRB|=_BV(ADTS2)|_BV(ADTS1)|_BV(ADTS0);
    自動変換タイマ/カウンタ1捕獲*/
  DIDR0 |=_BV(ADC0D);
  /*デジタル入力禁止 ADC0: PC0*/
}

void timer_ini(){//タイマー設定
  /*PWM*/
  TCCR1A |=_BV(COM1A1)|_BV(WGM11);
  /*位相基準PWM OC1A TOP値ICR1*/
  TCCR1B|=_BV(WGM13)|_BV(CS10);
  /*WGM13 WGM12 WGM11 WGM10: 1000 位相基準PWM動作 ICR1
   *CS12 CS11 CS10 : 001 分周なし*/
    
  ICR1 = 226;//割り込み周波数 20000Hz時 499
  //ICR1 = 10000;
  ICR1_buf = ICR1;
  TIMSK1|=_BV(ICIE1);/*タイマ/カウンタ1捕獲割り込み許可*/
  OCR1B = 0;
}

void usart_tx(unsigned char data){//送信用関数
  while( !(UCSR0A & (1<<UDRE0)) );        //送信ﾊﾞｯﾌｧ空き待機
  UDR0 = data;
}

unsigned char usart_rx(void){//受信用関数
  while( !(UCSR0A & (1<<RXC0)) );                //受信完了待機
  return UDR0;                                //受信ﾃﾞｰﾀ返す
}

void puts_tx(char *str){//文字列送信用関数
  while( *str != 0x00 ){ //nullになるまで実行
    usart_tx(*str);
    str++;                                    //ｱﾄﾞﾚｽ+1
  }
}

void serial_ini(){// シリアル通信設定
  UBRR0 = MYUBRR;
  UCSR0A=0b00000000;//受信すると10000000 送信有効になると00100000
  UCSR0B=0b00011000;//送受信有効
  UCSR0C=0b00000110;//データ8bit、非同期、バリティなし、stop1bit
}

void pwm_tx(unsigned int pwm_val){    //PWMのﾃﾞｰﾀ＆電圧値換算し送信
  unsigned char a4,a3,a2,a1;        //PWMのﾃﾞｰﾀ各桁
  a4 = (pwm_val/1000)+0x30;        //1000の位
  a3 = (pwm_val/100)%10+0x30;        //100の位
  a2 = (pwm_val/10)%10+0x30;        //10の位
  a1 = (pwm_val%10)+0x30;            //1の位

  usart_tx(a4);        //ﾃﾞｰﾀ各桁の送信
  usart_tx(a3);
  usart_tx(a2);
  usart_tx(a1);
  return;
}

void pin_ini(){//ピン設定
  DDRB    =   0b00101110;
  /*SCK:1 MISO:0 MOSI:1PB1:OCR1A PB2:CS CSをLOWで選択*/
  PORTB   =   0b00000100;
}

uint8_t clip_ef(uint8_t ad_data){
  if(ad_data>180)//上限
    ad_data=180;
  if (ad_data<30)//下限
    ad_data=30;
  return ad_data;
}

void add_check(uint8_t top, uint8_t middle, uint8_t distingish){//distingish 1ならwrite,0ならread
  uint8_t delay_buf=0;
  if(distingish==1){//wrie
    delay_buf = 0x22;//0x55*(adc_delay_width/256.0);
  }else {//read
    delay_buf = 0;
  }

  uint16_t add_check=(top<<2)+middle+delay_buf;
    if(add_check >= 0xEFF){
	 if(distingish ==1){//writeのaddress
	   add_top_w=0;
	   add_middle_w=0;
	   add_low_w=0;
	 }else{//readのaddress
	   add_top_r=0;
	   add_middle_r=0;
	   add_low_r=0;
	 }
    }
  }

  /*タイマ1 捕獲割り込み*/
ISR(TIMER1_CAPT_vect){

  /* ADMUX|=_BV(ADLAR)|_BV(MUX0);
	 ADCSRA|=_BV(ADSC);
	 while(is_SET(ADCSRA,ADIF)==0);  //変換終了まで待機
	 adc_delay_width = ADCH;
	 add_middle_w = 0x55*(adc_delay_width/256.0);*/
  
    //OCR1A = bufICR1 * (delay_data[i+1]/256.0);
    // OCR1A = bufICR1 * (buf_ad/256.0);//クリッピング用
  // ADMUX |= _BV(ADLAR);
  // ADMUX&=~_BV(MUX0);
    ADCSRA|= _BV(ADSC);//ADC開始
    while(is_SET(ADCSRA,ADIF)==0);  //変換終了まで待機*/
    ad_buf = ADCH;

    add_check(add_top_w,add_middle_w,1);
    add_check(add_top_r,add_middle_r,0);

    PORTB=0b00000000;
    spi_send(0x02);//write
    spi_send(add_top_w);
    spi_send(add_middle_w);
    spi_send(add_low_w);
    spi_send(ad_buf);
    PORTB=0b00000100;
      
    PORTB=0b00000000;
    spi_send(0x03);//read
    spi_send(add_top_r);
    spi_send(add_middle_r);
    spi_send(add_low_r);
    serial_buf = spi_get();
    PORTB=0b00000100;
    
    OCR1A = serial_buf;
    //delay_sound(buf_ad);
    //buf_ad = clip_ef(buf_ad); //クリッピング用

    add_low_w++;
    if(add_low_w>=0xFF){
	 add_low_w=0;
	 add_middle_w++;
	 if(add_middle_w>=0xFF){
	   add_middle_w=0;
	   add_top_w++;
	 }
    }
    add_low_r++;
    if(add_low_r>=0xFF){
	 add_low_r=0;
	 add_middle_r++;
	 if(add_middle_r>=0xFF){
	   add_middle_r=0;
	   add_top_r++;
	 }
    }
    //  pwm_tx(ad_buf);
    // pwm_tx(serial_buf);
    // LF;
    /*シリアル通信すると正常に波形が出力できないのであくまでデバッグ用に*/
  }

  /*メイン関数*/
int main(void){
    /*初期化設定*/
    spi_ini();
    serial_ini();
    adc_ini();
    timer_ini();
    pin_ini();
    
    sei();//全体割り込み許可
    
    PORTB = 0b00000000;
    spi_send(0x01);
    spi_send(0x00);
    PORTB = 0b00000100;
    // ADCSRA |=_BV(ADSC);//AD変換初期化兼開始
    
    while(1){
    
    }
    return 0;
  }
