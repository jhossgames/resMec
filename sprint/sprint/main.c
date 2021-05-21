 /*
 * Sprint8.c
 *
 * Created: 06/05/2021 01:18:26
 * Author : Francisco Olimpio Ferreira da Silva
 */ 

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define TRUE 1
#define FALSE 0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>
#include <math.h>
#include "nokia5110.h"

typedef unsigned char bool;

int  FreqRespiracao = 5;
long int tempo_ms = 0;
long int tempo_anterior_bvm = 0;
int bvm_time = 0;
int leitura_ADC = 0;
float temperatura = 0;
int temperatura_num_int = 0;
int temperatura_num_dec = 0;
int porcentagem_oximetro = 0;
int batimentos = 0;
int bpm = 0;
int contador = 1;
int contador_pressao = 0;
int contador_bvm = 0;
int contador_aux1 = 0;
int saturacao = 0;
int tipotela = -1; //vai de 0 a 3 e cada inteiro significa uma exibição diferente no display
int volume = 1;
bool sentido = TRUE; // caso seja 1 o servo BVM desce e caso seja 0 desce.
bool checkwave1 = TRUE;
bool bip_fim_bvm = TRUE;
bool estado_bip = TRUE;
bool estado_eqaux = FALSE;
bool estado_leds = TRUE;
char freq_lcd[3];
char bpm_lcd[4];
char porcentagem_oximetro_lcd[4];
char temperatura_int_lcd[4];
char temperatura_dec_lcd[3];
char pressao_recebida[9] =";HHHxLLL:";
char pressao_pronta[8] ="HHHxLLL";
char saturacao_lcd[4];
char volume_lcd[2];
char estado_bip_lcd[4];
char estado_eqaux_lcd[4]; 
char estado_leds_lcd[4];


void lcd_ctrl();
void calculo_bpm();
void sensores_temp_oxig();
void escrever_erro_pressao();
void formatar_pressao();
void bvm_o2_ctrl();

ISR(INT0_vect){

	if(tipotela >= 1){
	
		if(tipotela == 1){
			if(FreqRespiracao<30)
				FreqRespiracao++;
		}
		else
			if(tipotela == 2){
				if(saturacao<10)
					saturacao++;	
			}
			else
				if(tipotela == 3){
					if(volume < 8)
						volume++;
				}
				else
					if(tipotela == 4)
						estado_bip = TRUE;
					else
						if(tipotela == 5)
							estado_eqaux = TRUE;
						else
							if(tipotela == 6)
								estado_leds = TRUE;
	}
}

ISR(INT1_vect){

	if(tipotela >= 1){
		
		if(tipotela == 1){
			if(FreqRespiracao>5)
			FreqRespiracao--;
		}
		else
			if(tipotela == 2){
				if(saturacao > 0)
					saturacao--;
			}
			else
				if(tipotela == 3){
					if(volume > 1)
						volume--;
				}
				else
					if(tipotela == 4)
						estado_bip = FALSE;
					else
						if(tipotela == 5)
							estado_eqaux = FALSE;
						else
							if(tipotela == 6)
								estado_leds = FALSE;
	}	
}

ISR(PCINT2_vect){
	
	batimentos++;
}

//seleção display
ISR(PCINT0_vect){
	
	checkwave1 = !checkwave1; //gambiarra pra funcionar somente no clock de subida ¯\_('-')_/¯
	
	if(checkwave1){
	if(tipotela == 6)
		tipotela = -1;
	else
		tipotela++;
	}
}

//Chamada a cada 1ms
ISR(TIMER0_COMPA_vect){
	
	tempo_ms++;	
}

ISR(ADC_vect){
	
	leitura_ADC = ADC;
}

ISR(USART_RX_vect){
	
	pressao_recebida[contador_pressao] = UDR0;
	contador_pressao++;
	
	if(contador_pressao == 9){
		
		contador_pressao=0;
		if(pressao_recebida[0] != ';' || pressao_recebida[8] != ':' || pressao_recebida[4] != 'x' || pressao_recebida[1] < '0' || pressao_recebida[1] > '9' || pressao_recebida[2] < '0' || pressao_recebida[2] > '9' || pressao_recebida[3] < '0' || pressao_recebida[3] > '9' || pressao_recebida[5] < '0' || pressao_recebida[5] > '9' || pressao_recebida[6] < '0' || pressao_recebida[6] > '9' || pressao_recebida[7] < '0' || pressao_recebida[7] > '9')
			escrever_erro_pressao();
		else
			formatar_pressao();
	}
}

void USART_Init(unsigned int ubrr){
	
	UBRR0H = (unsigned char)(ubrr>>8); //Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita o transmissor e o receptor
	UCSR0C = (0<<USBS0)|(3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 1 de parada
}

void USART_Transmit(unsigned char data){
	
	while(!( UCSR0A & (1<<UDRE0)));//Espera a limpeza do registr. de transmissão
	UDR0 = data; //Coloca o dado no registrador e o envia
}

unsigned char USART_Receive(void){
	
	while(!(UCSR0A & (1<<RXC0))); //Espera o dado ser recebido
	return UDR0; //Lê o dado recebido e retorna
}

int main(void){
	
	long int tempo_anterior_bpm = 0;
	long int tempo_anterior_potenciometros = 0;
	long int tempo_atualizacao_lcd = 0;
	
	DDRD = 0b10100011; //definindo D 0 p/entrada e 1 p/ saida
	PORTD = 0b00001100; //resistores de pull up ativados na porta d
	DDRB = 0b00010110; //definindo B como saida para controle dos servos
	DDRC = 0b11111100; //definindo B como saida para display Nokia & entrada para temp e Sp02
	PORTC = 0b00000000; //resistores de pull up desativados na porta c
	PORTB = 0b01000000;
	
	//interrupções
	EICRA = 0b00001010; //borda de descida
	EIMSK = 0b00000011; //interrupção int0 e int 1
	PCICR = 0b00000101; // porta d como interrupção
	PCMSK2 = 0b00010000; //pino 1 da porta d na interrupção
	PCMSK0 = 0b01000000;
	
	//interrupções do timer  (led bar)
	TCCR0A = 0b00000010;//habilita modo CTC do TC0
	TCCR0B = 0b00000011;//liga TC0 com prescaler = 64
	OCR0A =  249; //formula do tempo é (prescaler * (OCR0A +1) / 16MHZ, neste caso é (64 * (249 +1) / 16MHZ = 1ms
	TIMSK0 = 0b00000010;//habilita a interrupção na igualdade de comparação com OCR0A.
	
	//configurações ADC
	ADMUX = 0b01000000; //AVCC tensão de referencia
	ADCSRA = 0b11101111; //habilita o AD, habilita interrupção
	ADCSRB = 0b00000000; //modo de conversão contínua
	DIDR0 = 0b00111100;  //pino PC0 como entrada ADC0
	
	//configura~ções PWM
	ICR1 = 39999; //configura periodo do pwm (20ms)
	TCCR1A = 0b10100010; //pwm rapido vida icr1
	TCCR1B = 0b00011010; //prescaler = 8
	OCR1A = 2000;  //1ms
	OCR1B = 4000; //2ms
	
	USART_Init(MYUBRR);
	
	sei(); //interrupções globais on/off
	
	//inicializando o display
	nokia_lcd_init();
	
	while (1){	
		
		//controle dos servos
		bvm_o2_ctrl();
		
		//calculo bpm
		if(tempo_ms - tempo_anterior_bpm >= 1500){
			tempo_anterior_bpm = tempo_ms;
			calculo_bpm();
		}		

		//atualização dos valores dos potenciometros a cada 150ms	
		if (tempo_ms - tempo_anterior_potenciometros >= 150){
			tempo_anterior_potenciometros = tempo_ms;
			sensores_temp_oxig();
		}
		
		//buzzer
		if((porcentagem_oximetro < 60 || temperatura > 41 || temperatura < 35) && estado_bip){
			PORTD |= 0b10000000;
			_delay_ms(1);
		}
		else{
			PORTD &= 0b01111111;
			_delay_ms(1);
		}
		
		//atualização do lcd a 200ms
		if (tempo_ms - tempo_atualizacao_lcd >= 200){
			tempo_atualizacao_lcd = tempo_ms;
			lcd_ctrl();
		}
		
		//equipamento auxiliar - acionamento
		if(estado_eqaux)
			PORTD |= 0b01000000;
 		else
			PORTD &= 0b10111111;	
			
		//LEDS dos botões
		if(estado_leds)
		PORTD |= 0b00100000;
		else
		PORTD &= 0b11011111;
			
	}
}

void lcd_ctrl(){
	
	if(estado_bip == TRUE){
		estado_bip_lcd[0] = 'O';
		estado_bip_lcd[1] = 'N';
		estado_bip_lcd[2] = '\0';
	}
	else {
		estado_bip_lcd[0] = 'O';
		estado_bip_lcd[1] = 'F';
		estado_bip_lcd[2] = 'F';
		estado_bip_lcd[3] = '\0';
	}
	
	if(estado_eqaux == TRUE){
		estado_eqaux_lcd[0] = 'O';
		estado_eqaux_lcd[1] = 'N';
		estado_eqaux_lcd[2] = '\0';
	}
	else {
		estado_eqaux_lcd[0] = 'O';
		estado_eqaux_lcd[1] = 'F';
		estado_eqaux_lcd[2] = 'F';
		estado_eqaux_lcd[3] = '\0';
	}	
	if(estado_leds == TRUE){
		estado_leds_lcd[0] = 'O';
		estado_leds_lcd[1] = 'N';
		estado_leds_lcd[2] = '\0';
	}
	else {
		estado_leds_lcd[0] = 'O';
		estado_leds_lcd[1] = 'F';
		estado_leds_lcd[2] = 'F';
		estado_leds_lcd[3] = '\0';
	}
	
	sprintf(volume_lcd, "%d",(volume));
	sprintf(saturacao_lcd, "%d",(saturacao*10));
	sprintf(temperatura_dec_lcd, "%d", temperatura_num_dec);
	sprintf(temperatura_int_lcd, "%d", temperatura_num_int);
	sprintf(porcentagem_oximetro_lcd, "%d", porcentagem_oximetro );
	sprintf(bpm_lcd, "%d", bpm);
	sprintf(freq_lcd, "%d", FreqRespiracao);
	
	if(tipotela == -1){

	//ANIMAÇÃO TELA INICIAL
	
		nokia_lcd_clear();
		nokia_lcd_set_cursor(10, 1);
		nokia_lcd_write_string("RESPIRADOR",1);
		nokia_lcd_set_cursor(20, 40);
		nokia_lcd_write_string("resMec",1);

		for(int counteranim_ini = 20; counteranim_ini < 34; counteranim_ini++){
			nokia_lcd_set_pixel((counteranim_ini+20),counteranim_ini,1);
			nokia_lcd_render();		
		}

		for(int counteranim_2 = 34; counteranim_2 > 16; counteranim_2--){
			nokia_lcd_set_pixel(54,counteranim_2,1);
			nokia_lcd_render();
		}
		
		for(int counteranim_3 = 54; counteranim_3 > 20; counteranim_3--){
			nokia_lcd_set_pixel(counteranim_3,16,1);
			nokia_lcd_render();
		}
					
		for(int counteranim_4 = 16; counteranim_4 < 34; counteranim_4++){
			nokia_lcd_set_pixel(20,counteranim_4,1);
			nokia_lcd_render();
		}		

		for(int counteranim_5x = 20, counteranim_5y = 34; counteranim_5x < 34; counteranim_5x++, counteranim_5y--){
			nokia_lcd_set_pixel(counteranim_5x,(counteranim_5y),1);
			nokia_lcd_render();
		}		

	}
	
	if(tipotela == 0){
		nokia_lcd_clear();
		nokia_lcd_set_cursor(1, 1);
		nokia_lcd_write_string("Sinais Vitais",1);
		nokia_lcd_set_cursor(1,10);
		nokia_lcd_write_string(bpm_lcd, 1);
		nokia_lcd_set_cursor(44,10);
		nokia_lcd_write_string("BPM", 1);
		nokia_lcd_set_cursor(1,20);
		nokia_lcd_write_string(porcentagem_oximetro_lcd, 1);
		nokia_lcd_set_cursor(44,20);
		nokia_lcd_write_string("%Sp02", 1);
		nokia_lcd_set_cursor(1,30);
		nokia_lcd_write_string(temperatura_int_lcd, 1);
		nokia_lcd_set_cursor(14,30);
		nokia_lcd_write_string(".", 1);
		nokia_lcd_set_cursor(18,30);
		nokia_lcd_write_string(temperatura_dec_lcd, 1);
		nokia_lcd_set_cursor(44,30);
		nokia_lcd_write_string("C", 1);	
		nokia_lcd_set_cursor(1,40);
		nokia_lcd_write_string(pressao_pronta, 1);
		nokia_lcd_set_cursor(44,40);
		nokia_lcd_write_string("mmHg", 1);
		nokia_lcd_render();
	}
	
	if(tipotela == 1){
		nokia_lcd_clear();
		nokia_lcd_set_cursor(1, 1);
		nokia_lcd_write_string("Parametros",1);
		nokia_lcd_set_cursor(1, 12);
		nokia_lcd_write_string(freq_lcd,1);
		nokia_lcd_set_cursor(25, 12);
		nokia_lcd_write_string("*",1);		
		nokia_lcd_set_cursor(35, 12);
		nokia_lcd_write_string("Resp/Min",1);
		nokia_lcd_set_cursor(1, 22);
		nokia_lcd_write_string(saturacao_lcd,1);
		nokia_lcd_set_cursor(35, 22);
		nokia_lcd_write_string("%O2",1);
		nokia_lcd_set_cursor(1, 32);
		nokia_lcd_write_string(volume_lcd,1);
		nokia_lcd_set_cursor(35, 32);
		nokia_lcd_write_string("vol",1);
		nokia_lcd_render();			
	}
	
	if(tipotela == 2){
		nokia_lcd_clear();
		nokia_lcd_set_cursor(1, 1);
		nokia_lcd_write_string("Parametros",1);
		nokia_lcd_set_cursor(1, 12);
		nokia_lcd_write_string(freq_lcd,1);
		nokia_lcd_set_cursor(35, 12);
		nokia_lcd_write_string("Resp/Min",1);
		nokia_lcd_set_cursor(1, 22);
		nokia_lcd_write_string(saturacao_lcd,1);
		nokia_lcd_set_cursor(25, 22);
		nokia_lcd_write_string("*",1);
		nokia_lcd_set_cursor(35, 22);
		nokia_lcd_write_string("%O2",1);
		nokia_lcd_set_cursor(1, 32);
		nokia_lcd_write_string(volume_lcd,1);
		nokia_lcd_set_cursor(35, 32);
		nokia_lcd_write_string("vol",1);	
		nokia_lcd_render();
		}
		
	if(tipotela == 3){
		nokia_lcd_clear();
		nokia_lcd_set_cursor(1, 1);
		nokia_lcd_write_string("Parametros",1);
		nokia_lcd_set_cursor(1, 12);
		nokia_lcd_write_string(freq_lcd,1);
		nokia_lcd_set_cursor(35, 12);
		nokia_lcd_write_string("Resp/Min",1);
		nokia_lcd_set_cursor(1, 22);
		nokia_lcd_write_string(saturacao_lcd,1);
		nokia_lcd_set_cursor(35, 22);
		nokia_lcd_write_string("%O2",1);
		nokia_lcd_set_cursor(1, 32);
		nokia_lcd_write_string(volume_lcd,1);
		nokia_lcd_set_cursor(25, 32);
		nokia_lcd_write_string("*",1);
		nokia_lcd_set_cursor(35, 32);
		nokia_lcd_write_string("vol",1);	
		nokia_lcd_render();
	}	
	
	if(tipotela == 4){
		nokia_lcd_clear();
		nokia_lcd_set_cursor(1, 1);
		nokia_lcd_write_string("Configs",1);
		nokia_lcd_set_cursor(1, 12);
		nokia_lcd_write_string("Sound",1);
		nokia_lcd_set_cursor(50, 12);		
		nokia_lcd_write_string("*",1);		
		nokia_lcd_set_cursor(60, 12);
		nokia_lcd_write_string(estado_bip_lcd,1);
		nokia_lcd_set_cursor(1, 22);
		nokia_lcd_write_string("Eq AUX",1);
		nokia_lcd_set_cursor(60, 22);
		nokia_lcd_write_string(estado_eqaux_lcd,1);	
		nokia_lcd_set_cursor(1, 32);
		nokia_lcd_write_string("LED's",1);
		nokia_lcd_set_cursor(60, 32);
		nokia_lcd_write_string(estado_leds_lcd,1);	
		nokia_lcd_render();
	}
	
	if(tipotela == 5){
		nokia_lcd_clear();
		nokia_lcd_set_cursor(1, 1);
		nokia_lcd_write_string("Configs",1);
		nokia_lcd_set_cursor(1, 12);
		nokia_lcd_write_string("Sound",1);
		nokia_lcd_set_cursor(60, 12);
		nokia_lcd_write_string(estado_bip_lcd,1);
		nokia_lcd_set_cursor(1, 22);
		nokia_lcd_write_string("Eq AUX",1);
		nokia_lcd_set_cursor(50, 22);
		nokia_lcd_write_string("*",1);		
		nokia_lcd_set_cursor(60, 22);
		nokia_lcd_write_string(estado_eqaux_lcd,1);
		nokia_lcd_set_cursor(1, 32);
		nokia_lcd_write_string("LED's",1);
		nokia_lcd_set_cursor(60, 32);
		nokia_lcd_write_string(estado_leds_lcd,1);
		nokia_lcd_render();
	}
	
	if(tipotela == 6){
		nokia_lcd_clear();
		nokia_lcd_set_cursor(1, 1);
		nokia_lcd_write_string("Configs",1);
		nokia_lcd_set_cursor(1, 12);
		nokia_lcd_write_string("Sound",1);
		nokia_lcd_set_cursor(60, 12);
		nokia_lcd_write_string(estado_bip_lcd,1);
		nokia_lcd_set_cursor(1, 22);
		nokia_lcd_write_string("Eq AUX",1);
		nokia_lcd_set_cursor(60, 22);
		nokia_lcd_write_string(estado_eqaux_lcd,1);
		nokia_lcd_set_cursor(1, 32);
		nokia_lcd_write_string("LED's",1);
		nokia_lcd_set_cursor(50, 32);
		nokia_lcd_write_string("*",1);		
		nokia_lcd_set_cursor(60, 32);
		nokia_lcd_write_string(estado_leds_lcd,1);
		nokia_lcd_render();
	}	
			
}

void bvm_o2_ctrl(){

	//BVM
	
	bvm_time = ((60/FreqRespiracao)*1000)/16;
	
	//limitador do volume
	if(contador_bvm == volume ){
		sentido = !sentido;
		contador_bvm = 0;
		bip_fim_bvm = TRUE;
	}

	if(tempo_ms - tempo_anterior_bvm >= bvm_time){
		tempo_anterior_bvm = tempo_ms;
		if(OCR1A == 4000)
			sentido = FALSE;
		else
			if(OCR1A == 2000){
				sentido = TRUE;
					if(bip_fim_bvm){
						if(estado_bip){
							PORTD |= 0b10000000;
							_delay_ms(25);
							PORTD &= 0b01111111;
						}
						bip_fim_bvm = FALSE;
					}
			}
	
		if(contador_aux1 >= (35/FreqRespiracao)/volume){
			if(sentido){
				OCR1A +=250;
				contador_bvm++;
			}
			else
				if(sentido == FALSE)
					OCR1A -=250;
			contador_aux1 = 0;
		}	
		contador_aux1++;	
	}

	//O2
	OCR1B = (saturacao * 200) + 2000;
}


void calculo_bpm(){
		
	bpm = batimentos * 20;
	batimentos = 0;
}

void sensores_temp_oxig(){
	
	float temp = leitura_ADC;
	porcentagem_oximetro = (100*temp)/818;
	ADMUX ^= 0b00000001;
	_delay_ms(2);
	temp = leitura_ADC;
	temperatura = (15*(temp+205))/307;
	ADMUX ^= 0b00000001;
	_delay_ms(2);
	temperatura_num_int = temperatura;
	temperatura_num_dec = ((temperatura - temperatura_num_int)*10);	
}

void escrever_erro_pressao(){
	
	char error[7] = "ERRO!";
	
	for (int i=0; i<=6; i++){
		pressao_pronta[i] = error[i];
	}
}

void formatar_pressao(){
	
	for (int i=0; i<=6; i++){
		pressao_pronta[i] = pressao_recebida[i+1];
		}
	pressao_pronta[7]= '\0';	
	
	if(pressao_pronta[4] == '0'){
		pressao_pronta[4] = pressao_pronta[5];
		pressao_pronta[5] = pressao_pronta[6];
		pressao_pronta[6]= '\0';	
	}
}


//como foi feito o calculo de tempo para o servo bvm
//fiz (60/frequencia_de_respiração) e dividi pelas posições possiveis e depois multipliquei por 1000 para obter os milisegundos 

//calculo oximetro (utilizei a seguinte formula para calcular porcentagem (porcentagem = (100*valor_atual)/valor_maximo
//calculo de temperatura, utilizei a equação da reta (temperatura = (15*(valor_atual_adc+205))/307)