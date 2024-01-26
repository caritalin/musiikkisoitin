#include <Keypad.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "rgb_lcd.h"

//määritellään kaikki tarvittavat nuotit ja niiden taajuus.
  #define A2  110
  #define AS2 117
  #define B2  123
  #define C3  131
  #define CS3 139
  #define D3  147
  #define DS3 156
  #define E3  165
  #define F3  175
  #define FS3 185
  #define G3  196
  #define GS3 208
  #define A3  220
  #define AS3 233
  #define B3  247
  #define C4  262
  #define CS4 277
  #define D4  294
  #define DS4 311
  #define E4  330
  #define F4  349
  #define FS4 370
  #define G4  392
  #define GS4 415
  #define A4  440
  #define AS4 466
  #define B4  494
  #define C5  523
  #define CS5 554
  #define D5  587
  #define DS5 622
  #define E5  659
  #define F5  698
  #define FS5 740
  #define G5  784
  #define GS5 831
  #define A5  880
  #define AS5 932
  #define B5  988
  #define C6  1047
  #define CS6 1109
  #define D6  1175
  #define DS6 1245
  #define E6  1319
  #define F6  1397
  #define FS6 1480
  #define G6  1568
  #define GS6 1661
  #define A6  1760
  #define AS6 1865
  #define B6  1976
  #define C7  2093
  #define CS7 2217
  #define D7  2349
  #define DS7 2489
  #define E7  2637
  #define F7  2794
  #define FS7 2960
  #define G7  3136
  #define GS7 3322
  #define A7  3520
  #define AS7 3729
  #define B7  3951

int notes[][7] =      //int array jossa määritetty sävellajit ja oktaavit 
{ // Ensimmäinen oktaavi
  {A2, B2, CS3, D3, E3, FS3, GS3}, //a-duuri
  {A2, B2, C3, D3, E3, F3, G3}, // a-molli
  {B2, CS3, DS3, E3, F3, GS3, AS3}, // h-duuri
  {B2, CS3, D3, E3, FS3, G3, A3}, // h-molli
  {C3, D3, E3, F3, G3, A3, B3}, // c-duuri
  {C3, DS3, F3, G3, GS3, AS3, B3}, // c-molli
  {D3, E3, FS3, G3, A3, B3, CS4}, // d-duuri
  {D3, E3, F3, G3, A3, AS3, C4}, // d-molli
  {E3, FS3, GS3, A3, B3, CS4, DS4}, //e-duuri
  {E3, FS3, G3, A3, B3, C4, D4}, // e-molli

// Toinen oktaavi
  {A3, B3, CS4, D4, E4, FS4, GS4}, //a-duuri
  {A3, B3, C4, D4, E4, F4, G4}, // a-molli
  {B3, CS4, DS4, E4, F4, GS4, AS4}, // h-duuri
  {B3, CS4, D4, E4, FS4, G4, A4}, // h-molli
  {C4, D4, E4, F4, G4, A4, B4}, // c-duuri
  {C4, DS4, F4, G4, GS4, AS4, B4}, // c-molli
  {D4, E4, FS4, G4, A4, B5, CS5}, // d-duuri
  {D4, E4, F4, G4, A5, AS5, C5}, // d-molli
  {E4, FS4, GS4, A4, B4, CS5, DS5}, //e-duuri
  {E4, FS4, G4, A4, B4, C5, D5}, // e-molli

// kolmas oktaavi
  {A4, B4, CS5, D5, E5, FS5, GS5}, //a-duuri
  {A4, B4, C5, D5, E5, F5, G5}, // a-molli
  {B4, CS5, DS5, E5, F5, GS5, AS5}, // h-duuri
  {B4, CS5, D5, E5, FS5, G5, A5}, // h-molli
  {C5, D5, E5, F5, G5, A5, B5}, // c-duuri
  {C5, DS5, F5, G5, GS5, AS5, B5}, // c-molli
  {D5, E5, FS5, G5, A5, B5, CS6}, // d-duuri
  {D5, E5, F5, G5, A5, AS5, C6}, // d-molli
  {E5, FS5, GS5, A5, B5, CS6, DS6}, //e-duuri
  {E5, FS5, G5, A5, B5, C6, D6}, // e-molli

// Neljäs oktaavi
  {A5, B5, CS6, D6, E6, FS6, GS6}, //a-duuri
  {A5, B5, C6, D6, E6, F6, G6}, // a-molli
  {B5, CS6, DS6, E6, F6, GS6, AS6}, // h-duuri
  {B5, CS6, D6, E6, FS6, G6, A6}, // h-molli
  {C6, D6, E6, F6, G6, A6, B6}, // c-duuri
  {C6, DS6, F6, G6, GS6, AS6, B6}, // c-molli
  {D6, E6, FS6, G6, A6, B6, CS7}, // d-duuri
  {D6, E6, F6, G6, A6, AS6, C7}, // d-molli
  {E6, FS6, GS6, A6, B6, CS7, DS7}, //e-duuri
  {E6, FS6, G6, A6, B6, C7, D7}, // e-molli

};

//näppäimistön määritys
const byte ROWS = 4; 
const byte COLS = 4; 
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

//näppäimistön pinnien määritys
byte rowPins[ROWS] = {5, 4, 3, 2}; 
byte colPins[COLS] = {9, 8, 7, 6}; 

//näppäimistön alustus
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// lcd näytön määritys
rgb_lcd lcd;
const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

// käytettyjen pinnien määritys, ensimmäinen pinni on rekistereillä
const int trigPin = B00000100;
const int echoPin = 16;
const int interruptPin = 11;
const int potPin = A0;

// napin painalluksille varatut merkit
char key = NULL;
char key2 = NULL;

//käytetyt muuttujat, pituus ja aika
long cm, t;

// aika keskeytyksen counter
volatile uint16_t timerCounter = 0;


void setup()
{
  //LCD näytön alustus
  lcd.begin(16, 2);
  lcd.setRGB(colorR, colorG, colorB);
  
  // 16bit T/C 1 (ajastin/laskuri)
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); 
  OCR1A = 15624; 
  TIMSK1 = (1 << OCIE1A); 

  // määritetään pinniasiat 
  DDRB = trigPin;
  pinMode(echoPin, INPUT);
  pinMode(potPin, INPUT);
  
   // Aseta ADEN päälle, otetaan käyttöön ADC
  ADCSRA |= (1 << ADEN);
  // Aseta jännitereferenssi AVcc:ksi
  ADMUX |= (1 << REFS0); 
  // Tyhjennä ADLAR, jotta tulokset ovat oikein perustasolla
  ADMUX &= ~(1 << ADLAR); 

  //hiljennetään kaiutin
  noTone(12);

  // alustetaan watchdog
  wdt_enable(WDTO_8S);
}

// Funktio näppäimistön syötteen ottoon.
void check_key()
{
  lcd.clear();
  lcd.print("valitse savel");
  lcd.setCursor(0, 2);
  lcd.print( "0 - 9");

  // odotetaan että nappia painetaan oikealla arvolla.
  while (!key)
  {
    key = keypad.getKey();
    // tarkistetaan, että vääriä nappeja ei ole käytetty
    if (key =='A' ||key =='B' ||key =='C' ||key =='D' || key == '*' || key == '#')
      key = NULL;
  }

  lcd.clear();
  lcd.print("Valitse oktaavi");
  lcd.setCursor(0, 2);
  lcd.print("A - D");
  // odotetaan että nappia painetaan oikealla arvolla.
  while (!key2)
  {
    key2 = keypad.getKey();
    // tarkistetaan, että vääriä nappeja ei ole käytetty
    if (key2 =='0' || key2 =='1' || key2 =='2' || key2 =='3' || key2 =='4' || key2 =='5' || key2 =='6' || key2 =='7' || key2 =='8' ||key2 =='9' ||  key2 == '*' || key2 == '#')
      key2 = NULL;
  }
  lcd.clear();
  lcd.print("Savel ");
  lcd.print(key);
  lcd.setCursor(0, 2);
  lcd.print("Oktaavi ");
  lcd.print(key2);
}

// Pääohjelma
void loop()
{
  //jos näppäimistöllä ei ole annettu vielä arvoja kutsutaan check_key funktiota
  if (!key2 && !key)
    check_key();
  
  // tarkistetaan jos käyttäjä on tehnyt nappi keskeytyksen ja onko aikakeskeytys täynnä
  if (digitalRead(interruptPin) == HIGH ||  timerCounter >= 1800)
  {
    lcd.clear();
    noTone(12);
    delay(500);
    key = NULL;
    key2 = NULL;
    timerCounter = 0;
  }

  //rekistereillä laitetaan trigPinnin tila alas
  PORTB ^= trigPin;
  delayMicroseconds(4);
  //rekistereillä laitetaan trigPinnin tila ylös ja ultraääni sensori lähettää ultraääntä
  PORTB = trigPin;
  delayMicroseconds(10);
  //rekistereillä laitetaan trigPinnin tila alas
  PORTB ^= trigPin;

  // lasketaan kuinka kauan ultraäänellä kesti palauta sensorille
  t = pulseIn(echoPin, HIGH);

  //muutetaan mitattu aika senttimetreiksi
  cm = t / 29 / 2;

  //kutustaan funktiota, joka lukee potenttiometrin arvon, josta saadaan nuotin kesto
  int dur = note_duration();

  // kutsutaan funktiota, joka soittaa nuottia
  tone_to_play(cm,(int)key -48, dur);

  // nollataan watchdog
  wdt_reset();

}

//funktio, jolla tehdään AD - muunnos ja saadaan potikan arvo
int readPotentiometer(int pin)
{
  // Aseta ADC-muunnos
  ADCSRA |= (1 << ADSC);  // Aloita muunnos

  // Odota muunnoksen valmistumista
  while (ADCSRA & (1 << ADSC))
  ;

  // Palauta luettu arvo
  return ADC;
}

int   note_duration()
{
  // Tyhjentää kolme alinta bittiä ADMUX-rekisterissä
  ADMUX &= 0xF8;
  // Asettaa jännitereferenssin AVcc:ksi ja asettaa kolme alinta bittiä potPin:stä riippuen
  ADMUX |= (1 << REFS0) | (potPin & 0x07);

  //kutsutaan funktiota, joka lukee potikan arvon
  int val = readPotentiometer(potPin);

  // muutetaan luettu arvo nuotin kestoksi 2 - 16 sekunnin osaa
  int dur = map(val, 150, 1000, 2, 16);

  return (dur);
}  

// funktiot joka määrittää mitä nuottia soitetaan mistäkin etäisyydestä
void  tone_to_play(long cm, int mod, int dur)
{
  int noteplay = 0;
  int oct;

  // lisätään mod:ia, joka vaihtaa arraysta seuraavan oktaavin
  if (key2 == 'A')
    oct = 0;
  else if (key2 == 'B')
    oct = 10;
  else if (key2 == 'C')
    oct = 20;
  else if (key2 == 'D')
    oct = 30;

  // etäisyyden tarkistus ja oikean nuotin asetus. mod muuttaa säveltä ja oct oktaavia
  if (cm >= 2 && cm < 5)
    noteplay = notes[oct + mod][0];
  if (cm >= 5 && cm < 8)
    noteplay = notes[oct + mod][1];
  if (cm >= 8 && cm < 11)
    noteplay = notes[oct + mod][2];
  if (cm >= 11 && cm < 14)
    noteplay = notes[oct + mod][3];
  if (cm >= 14 && cm < 17)
    noteplay = notes[oct + mod][4];
  if (cm >=  17 && cm < 20)
    noteplay = notes[oct + mod][5];
  if (cm >= 20 && cm < 23)
    noteplay = notes[oct + mod][6];
  if (cm > 21 && dur == 0)
    noTone(12);
  // jos etäisyys on löytynyt soitetaan ääni
  else
  {
    //määritetään nuotin pituus sekuntti/dur
    int notedur = 1000/dur;
    //syötetään nuotti kaijuttimeen
    tone(12, noteplay, notedur);
    // nuotin pituuden 1.3 kertainen viive, jotta lyhyimmät nuottivälit erottuvat
    delay(notedur* 1.3);
  }
}

// Kutsutaan kun ylivuoto tapahtuu
ISR(TIMER1_COMPA_vect)
{
  timerCounter++; // inkrementoituu joka sekunti
}
