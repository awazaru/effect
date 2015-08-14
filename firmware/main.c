/*H27 8/14~
 *エフェクター
 *Atmega328p使用
 *動作周波数20MHz(外部セラミック発振子,分周なし)
 *エフェクト不明
 */
#include <avr/io.h>
#include <math.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define is_SET(x,y) ((x) & (1<<(y)))


#define FOSC   20000000//動作周波数
#define BAUD   9600//ボーレート
#define MYUBRR (FOSC/16/BAUD)-1

// ｼﾘｱﾙ通信ｺﾏﾝﾄﾞ（ｱｽｷｰｺｰﾄﾞ)
#define LF        usart_tx(0x0a)        //改行
#define CR        usart_tx(0x0d)        //復帰
#define HT        usart_tx(0x09)        //水平ﾀﾌﾞ

#define smp_f 30000//割り込み周波数(Hz) ICRの設定値

void pwm_tx(unsigned int);
void usart_tx(unsigned char);
unsigned char usart_rx(void);


/*変数宣言*/

uint8_t  bufICR1 = 0;
uint8_t  buf_ad = 0;

/*関数宣言*/
/*高速PWMではTOP値がOCRA、比較値がOCR0Bとなる*/

void adc_ini(){
    ADMUX |=_BV(ADLAR);
    /*REFS1 REFS0 : 00 Arefピンの外部基準電圧
     ADPS2 ADPS1 ADPS0 :000 CK/2 ~= 100000Hz(変換クロック)
     MUX3 MUX2 MUX1 MUX0: 0000　ADC0ピン
     */
    ADCSRA|= _BV(ADEN)|_BV(ADPS1);
    /*ADEN :1 A/D許可
     ADPS2 ADPS1 ADPS0 :000 CK/4 ~= 50000Hz(変換クロック)
     
     _BV(ADSC)によって起動 単独変換動作
     */
    DIDR0 |=_BV(ADC0D);
}

void timer_ini(){//タイマー設定
    /*PWM*/
    TCCR1A |=_BV(COM1B1)|_BV(WGM11);
    /*位相基準PWM TOP値ICR1*/
    TCCR1B|=_BV(WGM13)|_BV(CS10);
    /*WGM13 WGM12 WGM11 WGM10: 1000 位相基準PWM動作 ICR1
     *CS12 CS11 CS10 : 001 分周なし*/
    
    ICR1 = 332;//割り込み周波数 20000Hz時 499
    bufICR1 = ICR1;
    TIMSK1|=_BV(ICIE1);/*タイマ/カウンタ1捕獲割り込み許可*/
    OCR1B = 0;
}

/*タイマ1 捕獲割り込み*/
ISR(TIMER1_CAPT_vect){
    
    ADCSRA|= _BV(ADSC);//ADC開始
    while(is_SET(ADCSRA,ADIF)==0);  //変換終了まで待機
   
    buf_ad = ADCH;
    OCR1B = bufICR1 * buf_ad/256.0;
    //pwm_tx(OCR1B);
    //LF;
}

void usart_tx(unsigned char data)//送信用関数
{
    while( !(UCSR0A & (1<<UDRE0)) );        //送信ﾊﾞｯﾌｧ空き待機
    UDR0 = data;
}

unsigned char usart_rx(void)//受信用関数
{
    while( !(UCSR0A & (1<<RXC0)) );                //受信完了待機
    return UDR0;                                //受信ﾃﾞｰﾀ返す
}


void puts_tx(char *str)//文字列送信用関数
{
    while( *str != 0x00 ) //nullになるまで実行
    {
        usart_tx(*str);
        str++;                                    //ｱﾄﾞﾚｽ+1
    }
}

void serial_ini(){
    // シリアル通信設定
    UBRR0 = MYUBRR;
    UCSR0A=0b00000000;//受信すると10000000 送信有効になると00100000
    UCSR0B=0b00011000;//送受信有効
    UCSR0C=0b00000110;//データ8bit、非同期、バリティなし、stop1bit
}

void pwm_tx(unsigned int pwm_val)    //PWMのﾃﾞｰﾀ＆電圧値換算し送信
{
    unsigned char a4,a3,a2,a1;        //PWMのﾃﾞｰﾀ各桁
    
    a4 = (pwm_val/1000)+0x30;        //1000の位
    a3 = (pwm_val/100)%10+0x30;        //100の位
    a2 = (pwm_val/10)%10+0x30;        //10の位
    a1 = (pwm_val%10)+0x30;            //1の位
    
    usart_tx(a4);        //ﾃﾞｰﾀ各桁の送信
    usart_tx(a3);
    usart_tx(a2);
    usart_tx(a1);
    // LF;
    
    return;
}


void pin_ini(){//ピン設定
    DDRB = 0b00000000;
    PORTB= 0b00000000;
    DDRB = 0b00000100;//OCR1B
    PORTB = 0b00000000;
    
}


/*メイン関数*/
int main(void){
    
    serial_ini();
    adc_ini();
    timer_ini();
    pin_ini();
    
    sei();//割り込み許可
    ADCSRA |=_BV(ADSC);//AD変換開始
    
    while(1){

    }
    return 0;
}
