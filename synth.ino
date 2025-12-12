#include <Mozzi.h> //biblioteca pra manipulaçao dos sons

/* inclui a parte de osciladores, que criam formatos de onda, como senoidal, serra etc... */
#include <Oscil.h> 

/* inclusao de "wavetables". O teorema de fourier diz que
qualquer som complexo pode ser considerado uma combinaçao de ondas senoidais simples,
mozzi simplifica esse processo através de wavetables, que é nada mais do que um vetor
que armazena o ciclo de uma forma de onda especifica...
*/
#include <tables/sin2048_int8.h>
#include <tables/saw8192_int8.h>
#include <tables/smoothsquare8192_int8.h>

#include <EventDelay.h> //para controlar o metronomo
#include <mozzi_midi.h> 
#include <LowPassFilter.h> //para a modulação do timbre


#define RATE 128 //frequencia de "atualizacao" do loop controlador 

//potenciometros controladores
#define P_PITCH A0
#define P_PHASE A1
#define P_RATE A2
#define P_LEGATO A3
#define P_FILTER A4

//leds indicadores de grau de modulacao
#define L_GREEN 3
#define L_YELLOW 4
#define L_RED 5

#define F_SEQ 10 //avanca sequencia
#define B_SEQ 11 //retrocede sequencia
#define OCT_UP 13 //sobe oitava
#define OCT_DOWN 12 // desce oitava


//objetos osciladores de audio
Oscil<SAW8192_NUM_CELLS, AUDIO_RATE> osc_a(SAW8192_DATA);       
Oscil<SMOOTHSQUARE8192_NUM_CELLS, AUDIO_RATE> osc_b(SMOOTHSQUARE8192_DATA); 
//Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> oscThree(SIN2048_DATA);

LowPassFilter lpf; //filtro de timbre
EventDelay metronome; //gerenciador de ritmo
EventDelay print_delay;
char gain = 0; //0 = silencio, 1 = som
int sequence = 0; //indice da sequência atual
int current_note = 0; //nota midi
int oct_shift = 0; //oitavador

//constantes de controle
#define PITCH_OFFSET 12 //1 oitava (12 semitons)
#define ARP_RATE_MIN 32 //taxa minima do arpeggiador
#define ARP_RATE_MAX 1024 //taxa maxima do arpegiador
#define LPF_CUTOFF_MIN 10 //frequencia minima do filtro, abafada***
#define LPF_CUTOFF_MAX 245 //maxima que o arduino aguenta
const float DIV = 1.0 / 1023.0; //constante para evitar divisao lenta


const int n_sequences = 40; 
const int max_notes = 8;

// Variaveis de Debounce (Tempo minimo entre cliques)
unsigned long last_press = 0; // Usado para todos os botões de controle
const unsigned long debounce_time = 2*(10e5);//200ms

//sequencias de notas para o arpegiador
const int notes[n_sequences][max_notes] = {
    {60, 64, 67, 71, 60, 64, 67, 61}, //in the end 
    {62, 69, 69, 65, 64, 64, 64, 65}, //stranger things
    // 0: Maior Ascendente
    {60, 62, 64, 65, 67, 69, 71, 72}, 
    // 1: Maior Descendente
    {72, 71, 69, 67, 65, 64, 62, 60}, 
    // 2: Menor Ascendente
    {60, 62, 63, 65, 67, 68, 70, 72}, 
    // 3: Menor Descendente
    {72, 70, 68, 67, 65, 63, 62, 60}, 
    // 4: Pentatônica Maior
    {60, 62, 64, 67, 69, 67, 64, 62}, 
    // 5: Pentatônica Menor
    {60, 63, 65, 67, 70, 67, 65, 63},
    // 6: Dórico
    {60, 62, 63, 65, 67, 69, 70, 72},
    // 7: Frígio
    {60, 61, 63, 65, 67, 68, 70, 72},
    // 8: Lídio
    {60, 62, 64, 66, 67, 69, 71, 72},
    // 9: Mixolídio
    {60, 62, 64, 65, 67, 69, 70, 72},
    // 10: Eólio
    {60, 62, 63, 65, 67, 68, 70, 72},
    // 11: Lócrio
    {60, 61, 63, 65, 66, 68, 70, 72},
    // 12: Cmaj7
    {60, 64, 67, 71, 67, 64, 60, 64},
    // 13: Cm7
    {60, 63, 67, 70, 67, 63, 60, 63},
    // 14: Cdim
    {60, 63, 66, 69, 66, 63, 60, 63},
    // 15: Cdom7
    {60, 64, 67, 70, 67, 64, 60, 64},
    // 16: Csus2
    {60, 62, 67, 74, 67, 62, 60, 62},
    // 17: Caug
    {60, 64, 68, 76, 68, 64, 60, 64},
    // 18: Tônica-3ª-5ª-3ª
    {60, 63, 67, 63, 60, 63, 67, 63},
    // 19: Sequência Quádrupla
    {60, 62, 64, 65, 64, 62, 60, 62},
    // 20: Alternância Tônica
    {60, 67, 72, 67, 60, 67, 72, 67},
    // 21: Terças Saltadas
    {60, 63, 64, 67, 63, 67, 64, 60},
    // 22: Oitavas e Quintas
    {60, 72, 67, 79, 67, 72, 60, 67},
    // 23: Intervalos Fixos
    {60, 72, 60, 72, 60, 72, 60, 72},
    // 24: Blues Hexa
    {60, 63, 65, 66, 67, 70, 67, 63},
    // 25: Tons Inteiros
    {60, 62, 64, 66, 68, 70, 72, 70},
    // 26: Harmônica Menor
    {60, 62, 63, 65, 67, 68, 71, 72},
    // 27: Cigana Húngara
    {60, 62, 63, 66, 67, 68, 71, 72},
    // 28: Japonesa (In Sen)
    {60, 61, 65, 67, 70, 67, 65, 61},
    // 29: Arpejo Invertido
    {60, 67, 64, 72, 64, 67, 60, 64},
    // 30: Escala Cruzada
    {60, 74, 64, 77, 67, 79, 71, 84},
    // 31: Progressão de Quartas
    {60, 65, 70, 75, 70, 65, 60, 65},
    // 32: Cluster (Tensão)
    {60, 61, 62, 63, 64, 63, 62, 61},
    // 33: Aleatório Menor
    {63, 67, 70, 62, 68, 72, 60, 65},
    // 34: Aleatório Maior
    {60, 69, 64, 71, 67, 72, 62, 65},
    // 35: Quatro em Movimento
    {60, 62, 63, 64, 63, 62, 60, 62},
    // 36: Salto de Oitava
    {60, 72, 67, 79, 60, 72, 67, 79},
    // 37: Cromático Desc.
    {72, 71, 70, 69, 68, 67, 66, 65}
};

int amplitude = 0;

//altera os leds com base no modo
void refreshLeds(int mode) {
  digitalWrite(L_GREEN, HIGH);
  digitalWrite(L_YELLOW, (mode >= 1));
  digitalWrite(L_RED, (mode >= 2));
}

void setup() {
  Serial.begin(9600);
  //inicia a biblioteca mozzi
  startMozzi(RATE);
  print_delay.set(500);

  //inicializaçao de pinos digitais
  pinMode(L_GREEN, OUTPUT);
  pinMode(L_YELLOW, OUTPUT);
  pinMode(L_RED, OUTPUT);
  pinMode(F_SEQ, INPUT_PULLUP);
  pinMode(B_SEQ, INPUT_PULLUP);

  osc_a.setFreq(mtof(48)); //C3
  osc_b.setFreq(mtof(48)); //C3
  lpf.setResonance(128u); //ressonância média 
  metronome.set(1000); // Metrônomo com 1 segundo de delay inicial
}

/* funcao chamada na frequencia RATE,
   parte que nao precisa ser executada tao
   frequentemente quanto updateAudio...
*/
void updateControl(){
  //logica de debounce
  unsigned long time_now = mozziMicros();

  if ((time_now - last_press) > debounce_time) {
        //aumenta uma oitava
        if(digitalRead(OCT_UP) == LOW) {
          oct_shift += 12;
          last_press = time_now;
        }

        //diminui oitava
        if(digitalRead(OCT_DOWN) == LOW) {
          oct_shift -= 12;
          last_press = time_now;
        }
        
        //logica pra alternar entre as sequencias
        if(digitalRead(F_SEQ) == LOW) {
          if(sequence < n_sequences-1) {
            sequence++;
            current_note = 0;
          } else {
            sequence = 0;
            current_note = 0;
          }
          oct_shift = 0; 
          last_press = time_now;
         } else if (digitalRead(B_SEQ) == LOW) {
            if(sequence > 0) {
              sequence--;
              current_note = 0;
            } else {
              sequence = n_sequences-1;
              current_note = 0;
            }
            oct_shift = 0;
           last_press = time_now;
         } 
  }
 //le os valores dos potenciometros (0 - 1023)
 int pitch_val = mozziAnalogRead(P_PITCH); //tom do oscilador principal
 int phase_val = mozziAnalogRead(P_PHASE); //diferenca de afinacao entre osciladores
 int rate_val = mozziAnalogRead(P_RATE); //velocidade do arpegiador
 int legato_val = mozziAnalogRead(P_LEGATO); //controle do intervalo entre compassos
 int filter_val = mozziAnalogRead(P_FILTER); //controle de timbre (corte de frequencias)

                /*pega o valor de 0-1023 e transforma de 0 a 1, com -0.5 varia de -0.5 a 0.5.
                  multiplicado por uma oitava (12 semitons) * 2 vira 24 semi tons, ou seja, varia uma
                  oitava pra baixo e outra pra cima.
                */
 float note_interval = (pitch_val*DIV-0.5)*PITCH_OFFSET*2;
 float phase_amplitude = (phase_val*DIV)*100; // maximo de 100hz (bem dissonante)
 
                                                //inverte - aumenta o valor diminui a duracao do compasso
 float arp_rate = ARP_RATE_MIN+(ARP_RATE_MAX*(1-(rate_val*DIV)));//duracao de um compasso ritimico
 float legato = legato_val*DIV; //0 - 1
 int cutoff_freq = LPF_CUTOFF_MIN+(LPF_CUTOFF_MAX*(filter_val*DIV)); //analogo a arp_rate

 float freq_a = 0;
 float freq_b = 0;

  if (print_delay.ready()) {
        Serial.print("PITCH:");
        Serial.print(pitch_val);
        Serial.print(" | PHASE:");
        Serial.print(phase_val);
        Serial.print(" | RATE:");
        Serial.print(rate_val);
        Serial.print(" | LEGATO:");
        Serial.print(legato_val);
        Serial.print(" | FILTER:");
        Serial.print(filter_val);
        Serial.println("");
        
        print_delay.start(); // Reinicia o temporizador de impressão
    }
 //verifica se o metronomo esta pronto pro proximo evento
 if(metronome.ready()) {
  //se esta em silencio (intervalo entre notas), ativa nota
  if(gain == 0) {
    if(current_note >= max_notes) current_note = 0;
      
    //nota atual multiplicada pela variacao de pitch do potenciometro
    freq_a = mtof(notes[sequence][current_note]+note_interval+oct_shift);
    //nota do primeiro oscilador mais a phase de "desafinacao"
    freq_b = freq_a + phase_amplitude;

    //atribui frequencias aos osciladores
    osc_a.setFreq(freq_a);
    osc_b.setFreq(freq_b);
    
    gain = 1;
    current_note++;    
    
    //tempo ate o proximo ready
                  //tempo de nota
    metronome.set(arp_rate * (1-legato));
  } else {
    gain = 0;
                  //tempo entre notas
    metronome.set(arp_rate*legato);
  } 
  //reinicia o temporizador
  metronome.start();
 }

 //atribui frequencia de corte
 lpf.setCutoffFreq(cutoff_freq);

 if (aamplitude >= 0.66*127)
    refreshLeds(2);
 else if(amplitude >= 0.33*127)
    refreshLeds(1);
 else
    refreshLeds(0);
 
}



AudioOutput_t updateAudio(){
                              //soma a amplitude das duas ondas
                              //divide por quatro (desloca 2 bits)
                              // pois cada oscilador pode chegar a +127
                              //127 + 127 = 254
                              // como trabalhamos de -128 a 127 bits
                              // ai surge a necessidade de deslocar 2 bits, se nao ocorre saturacaoã
  int mixed_signal = (osc_a.next() + osc_b.next()) >> 2;

  amplitude = abs(mixed_signal*gain);
  
  //aplica o ganho instantaneo no sinal misto
  return lpf.next(mixed_signal * gain);
}

void loop() {
  audioHook();
}
