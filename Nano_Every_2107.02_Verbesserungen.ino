/*
Carstens Rollladentimer hat die folgenden Features Stand 25.07.2021.
    
Sommer- / Winterzeitumstellung
    - Die Umstellung Sommer-/Winterzeitumstellung erfolgt automatisch. Die Umstellungstage sind bis 2025 eingegeben.
    - Wird ein Umstelltag verpasst (weil die Steuerung stromlos ist) wird die Umschaltung bei Inbetriebnahme nachgeholt!
    - Im Display erscheint zwischen den Stunden und Minuten der Uhrzeit ein "s" für Sommerzeit oder ein "w" für Winterzeit

Timer/Schaltpunkte
    - Die Steuerung verfügt über 6 Timer / Schaltpunkte pro Tag 
    - Jeder Timer muss  mit einer der folgenden Möglichkeiten belegt werden:
            * Feste Uhrzeit: Dauerhaft feste Schaltzeit. 
            * Astrofunktion: Die Schaltzeit verändert sich abhängig vom Sonnenaufgang/Sonnenuntergang
            * Auslassen/frei: Der Timer wird nicht genutzt
    - Jeder Timer muss mit einem Wochentag oder einer Wochentagsgruppe belegt werden (Mo-So, Mo-Fr, Sa-So,...). 
      Der Timer schaltet nur an den angegebenen Wochentag(en)!
    - Bei Astrofunktion 
            * kann ein Offset (+/-) zum Sonnenaufgang/-untergang festgelegt werden.
            * können Grenzen eingegeben werden, wann der RL trotz Astrofunktion frühestend und spätestens öffnen/schließen soll.  
    - Für jeden Timer muss natürlich eine Zielposition (zwischen 0% und 100%) eingegeben werden.
      0% bedeutet ganz offen. 100% ist komplett geschlossen. Beliebige Zwischenpositionen sind möglich.
    - Für jeden Timer empfiehlt es sich eine Bewegungsrichtung (auf / ab / auf + ab) festzulegen. Nur wenn sich der RL 
      in die angegebene Richtung bewegen kann, wird der Timer auch ausgelöst.
      Der Grund für diese Funktion ist schwierig zu erklären: 
      Durch die sich verändernden Astro Zeiten kann sich die Reihenfolge der Timer ändern (Astro Timer wandert
      z.B. vor einen Festzeittimer). Hat der Festzeittimer 50% und der Astrotimer 0% dann fährt der RL normalerweise 
      erst auf 50% (aufwärts) und dann per Astrotimer auf 0% ganz auf. ERGEBNIS: RL ist ganz offen. 
      Wandert der Astrotimer jedoch vor den Festzeittimer, dann ändert sich das Verhalten: Der RL fährt dann zuerst auf 0%
      und wird dann über den Festzeittimer wieder auf 50% (abwärts) zugefahren. ERGEBNIS: RL ist 50% geschlossen!
      In diesem Fall könnte man den Festzeittimer so einstellen, dass er nur aufwärts ausgefüht werden soll. 
      Beispiele: 
        - Der RL steht bei 50%, nächster Schaltpunkt "100% nur abwärts":  Wird ausgeführt
        - Der RL steht bei 50%, nächster Schaltpunkt "30% nur abwärts":   Wird NICHT ausgeführt
        - Der RL steht bei 50%, nächster Schaltpunkt "30% auf + abwärts": Wird ausgeführt

Astrofunktion:
    - Die Sonnenauf- und Sonnenuntergangszeiten für die Astrotimer werden mit Hilfe des Längen- und Breitengrades berechnet 
      (und nicht aus Tabellen entnommen.)
    - Die Genauigkeit beträgt rund 2 Minuten
    - Aktuell werden Längen- und Breitengrad von Bauschheim verwendet und können nur im Programmcode geändert werden.

Rollladenposition:
    - Die RL-Position in % (0%= ganz auf, 100% = geschlossen) wird bei jeder Bewegung mitprotokolliert. 
    - Zur Bestimmung der RL-Position benötigt das Programm die Laufzeit des Rolladens (von ganz auf nach ganz zu).
      Die Laufzeiten MÜSSEN in den Einstellungen eingegeben oder gemessen werden.
    - Bei jeder Fahrt in eine Endlage (Rollmotor schaltet ab) erfolgt automatisch eine Kalibrierung auf 0% oder 100%.

Datensicherung
    - Alle Daten (Timer, Laufzeit, aktuelle Position) werden im EEPROM gesichert
    - Nach jeder Änderung von Einstellungen werden die Daten ins EEPROM geschrieben.
    - Nach jedem Stop des Rollladens wird außerdem die Rolladenposition ins EEPROM geschrieben 
      (der Flashspeicher ist also nach rund 100.000 Stopvorgängen kaputt)
    - Das Uhrmodul (RTC) ist batteriegepuffert und behält Zeit und Datum auch bei einem Stromausfall.

Neustart / Stromausfall:
    - Bei jedem Neustart/Reset werden alle Daten automatisch aus dem EEPROM wieder eingelesen
      und die RL-Steuerung setzt den Betrieb problemlos fort.
    - INAKTIV: Als Hinweis auf einen Stromausfall zeigt das Display nach einem Neustart eine Meldung, 
      die durch Drücken der Stop-Tast quittiert werden muss

Manuelle Bedienung
    - Eine manuelle Steuerung des Rolladens ist jederzeit möglich (Auf/Ab/Stop), auch wenn die Zeitsteuerung aktiv ist. 
    - Über einen Schalter können Zeit- und UV-Steuerung an- und ausgeschaltet werden.
    - Wird bei aktiver Zeitsteuerung eine manuelle Bediehnung durchgeführt, bleibt die Zeitsteuerung aktiv.
      Der nächste Timer/Schaltpunkt wird also bei Erreichen der Zeit ausgeführt.

Displaybeleuchtung
    - Die Displaybeleuchtung dimmt sich nach einer einstellbaren Zeit langsam aus
      und geht bei jeder RL-Bewegung oder bei Betätigung eines Taster/Schalters sofort wieder an.
    - Da es sich um ein OLED Display handelt ist eine dauerhafte Beleuchtung nicht gut für das Display (Einbrennen).
    - In den Einstellungen können Helligkeit und Dauer der Beleuchtung eingestellt werden. 

UV Sensor
    - Der UV Sensor misst die UVa und UVB Strahlung hinter der Scheibe. Oberhalb eines einstellbaren Grenzwerts, schließt der RL auf 75%
      Unterschreitet die UV Strahlung den Grenzwert, fährt der RL in die vorherige Position (die er sich gemerkt hat) zurück.
    - Natürlich triggert eine Überschreitung des Grenzwertes nur ein Zufahren des RLs. Ist er vorher schon weiter geschlossen (zB. auf 80%), 
      ist die UV Funktion deaktiviert.
    - Manuelle Eingriffe sind jederzeit möglich. Steht der RL auf 75% und wird manuell aufgefahren, fängt die Erfassung der UV Strahlung von vorne
      an. Nach einigen Minuten fährt der RL dann wieder auf 75%, wenn der UV Wert zu hoch ist.
    - Zum Ab- und Anschalten der UV Funktion, einfach den Ab-Taster lange (3 Sekunden) drücken.
    - Es gibt einen zweiten Faktor, mit dem eingestellt werden kann, wie schnell der Rollladen reagieren soll. Je höher die Zahl, desto schneller fährt der Rollladen 
      bei hohen UV-Werten zu (und auch wieder auf).
    - Überprüfung des Sensors: Die Ausgaben des Sensors werden überprüft. Springen die Werte zu sehr hin und her, dann ist der Sensor nicht richtig angeschlossen. 
      Das Display meldet dann für 5 Sekunden "Fehler UV-Sensor!". 

Einstellungsmenü
    - Das Einstellungsmnenü wird durch langes Drücken (3 Sekunden) der Stop-Taste geöffnet
    - Alle Einstellungen werden nur über die 3 Tasten vorgenommen.
    - Die 6 Timer werden im Einstellungsmenü programmiert.
    - Datum und Uhrzeit des Uhrmoduls (RTC) können gestellt werden
    - Die RL-Laufzeit MUSS (getrennt für komplettes Auf- und Zufahren) in den Einstellungen eingegeben werden 
      Alternativ kann ein Messprogramm zur halbautomatischen Ermittlung der RL-Laufzeit gestartet werden.
      Im Messprogramm wird der RL einmal ganz zugefahren und einmal ganz aufgefahren. Jeweils zum Start- und Stopzeitpunkt des RL muss 
      die rote Taste gedrückt werde. Das Programm stopt dann die Zeit zwischen dem Drücken der roten Taste (= Fahrtzeit).
    - Die Dauer und Helligkeit der Displaybeleuchtung kann eingestellt werden
        - Dauer 10 bis 120 Sekunden
        - Helligkeit 5 bis 110% (enspricht dem PWM Wert der LED Ansteuerung)
    - Alle Einstellungen werden automatisch im EEPROM gegen Stromausfall gesichert.

HARDWARE
    - Arduino Nano EVERY (MUSS: ~50k Speicher + 256 byte EEPROM), Mega oder Uno
    - RTC Modul DS3231 I2C
    - 2,8" OLED Display von AZ Delivery 
    - 3 Taster (Auf, Ab, Stop)
    - Schalter Automatik ein/aus 
    - Schalter Sonnensensor ein/aus 
    - Relaisplatte mit 2 Relais. Geeignet für 220V !!!! Stromlos nicht angezogen!
        - Relais 1 zum Schalten der Stroms
        - Relais 2 zum Einstellen der Richtung 
    - UV Sensor GUVA-S12D, betrieben mit 3,3V

CHANGE LOG
Juli 2021
    a. Sommer/Winterzeitumstellung geändert. Die Umstellung erfolgt jetzt nicht mehr nur am Tag 
        der Zeitumstellung sondern auch rückwirkend. Falls die Steuerung länger stromlos war  
        und wieder in Betrieb genommen wird, wird die Umstellung (falls erforderlich) sofort nachgeholt.
        Der Sommerzeitstatus wird im EEPROM gespeichert, damit nur einmal umgestellt wird.
    b. Die Wochentagfunktion geht jetzt einwandfrei. Die Potenzberechnungen der Gruppen war fehlerhaft.
        Alle Wochentage können jetzt auch einzeln ausgewählt werden.
    c. Abbruchfunktion bei den Einstellungen eingebaut. Ziel: Langes Drücken der Stop-Taste
        soll bei jedem Eingabefeld einen Abbruch ermöglichen, so dass die alten Werte erhalten bleiben.
        Geht aber erst so halb.
    d. Bei 0% zeigt das Display jetzt "offen". Bei 100% wird "geschlossen" angezeigt.
    e. Sämtliche weitere Korrekturen der RL Position außer über die verschiedenen Laufzeiten
        für auf/ab rausgeschmissen. Bringt nichts und ist extrem aufwändig.
        Die Messungen der Laufzeit zeigen, dass der RL nach mehreren Fahrten langsamer wird (bis zu 5%)
        weil der Motor heiß wird.
August 2021
    a. Die Abbruch-Funktion in den Einstellungen funktioniert jetzt perfekt. Ein Abbruch 
        ist bei jedem Menü/Eingabepunkt möglich. Danach erfolgt keine Speicherung der Werte, 
        die alten Werte bleiben erhalten.
    b. Speicherung der Werte umgestellt. Nicht mehr beim Verlassen des Einstellungsmenüs, sondern
        nach jeder Einstellung wird gespeichert, wenn nicht abgebrochen wurde (siehe a.)
    c. Das Einstellungsmenue wird nach 1min ohne Tastendruck automatisch verlassen bzw. springt 
        das Menü jede Minute um eine Eben nach oben. Funktioniert nur bei Menüs (EINSTELLUNGEN2()) und 
        nicht bei der Einstellung von Werten (EINSTELLUNGEN()).
    d. Soweit so perfekt. Läuft jetzt praktisch bugfrei... bis auf die Sonnensteuerung
*/


/// Bibliotheken
    #include <Wire.h>                 // Kommunikation über I2C für die RTC
    #include <RTClib.h>               // RTCLib by NeiroN. Für das Zeitmodul
    #include <EEPROM.h>               // Zum Schreiben/Lesen des EEPROMS
    #include <SPI.h>
    #include <Adafruit_GFX.h>         // Adafruit Grafik-Bibliothek
    #include <Adafruit_ST7735.h>      // Adafruit ST7735-Bibliothek
    //#include <Fonts\FreeSans9pt7b.h>
    //#include <TFT.h>                  // Arduino eingene TFT Bibliothek. Zirka 3,5kB kleiner als die beiden Adafruits aber deutlich langsamere Anzeige

/// OLED Initialisieren
    #define cs 3
    #define dc 2
    #define rst 4
    //TFT tft = TFT(cs, dc, rst);  // Wird für TFT.h benötigt
    Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, rst);  // (CS, DC, RST) Wird für Adafruit Bibliotheken benötigt

/// Real Time Clock definieren
    DS3231 myRTC;  // Wird für die RTC benötigt

/// Variablen und Konstranten
    
    //********************************************
    const String Version = "2107.02 Verbesserungen";
    //********************************************
    
    // byte: 1 byte / int: 2 byte / unsigned long: 4 byte / float: 4 byte / double: 8 byte
    const byte RELAIS_Strom = 16;   // Das 1. Relais schaltet den Strom, der zum 3. Reils fließt ein/aus
    const byte RELAIS_Richt = 17;   // Das 2. Relais schaltet die Drehrichtung des Rollladenmotors.
    
    const byte PIN_Taster_auf = 7;  
    const byte PIN_Taster_ab = 9;
    const byte PIN_Taster_stop = 8;

    const byte PIN_Schalter_Auto = 14;
    const byte PIN_Schalter_Sensor = 15;
    
    const byte TFT_Licht = 5;

    const float Pi = 3.14159265;  // Wird zur Berechnng der Astrozeiten benötigt
    

    // Aktueles Datum und aktuelle Uhrzeit
    DateTime Jetzt; 
    int   Jetzt_Julian, Tagesminuten, Startsekunde;
    byte  Tag_heute;
    // Taster und Schalter
    byte  Taster_auf, Taster_ab, Taster_stop, Schalter_Auto, Schalter_Sensor;
    byte  Licht_PWM, Licht_PWM_max; // aktuelle Helligkeit und max. Helligkeit als PWM Wert
    int   Licht_Dauer;    // Beleuchtungsdauer in s
    // Rollladen
    float Laufzeit_auf, Laufzeit_zu; // Laufzeit des RL auf/zu
    float RollPos_k;  // aktuelle Position des RL
    float StartPos_k; // Position aus der der RL startet
    int   ZielPos;    // Position, die der RL anfahren soll
    //Timerdaten
    byte  T_next;   // Nummer des bevorstehenden Timers
    byte  Ttyp[8], Tpos[8], Trichtung[8], Ttage[8];
    int   Tzeit[8], Tfrueh[8], Tspaet[8], Toffset[8]; // Zur Speicherung der Timerdaten. Array geht von 0 bis 6. Timerdaten werden in 1 bis 6 gespeichert. Alle Werte in Tagesminuten (z.B. 6:00 Uhr: 6*60=360 Minuten)
    byte  Tag_T[12] = {0, 127, 62, 65, 126, 1, 2, 4, 8, 16, 32, 64}; // Binärcode Wochentage: So=2^0=1, Mo=2^1=2, Di=4, Mi=8, Do=16, Fr=32, Sa=64 
    // Astrofunktion
    int   Astro_auf, Astro_zu;  // Sonnenaufgang und -untergang des aktuellen Tages
    //float Geo_L; Geo_B; // Längen- und Breitengrad für die Bestimmung der Astrozeiten
    // Lichtsensor
    byte  UVPos, UVPosMerker;
    long  UVSignal, UVSignalI, UVLimit;
    bool  UVModus, Refresh, UVStatus;
    unsigned long UV_Delay_ein, UV_Delay_aus;
    // Statuswerte und Hilfsgrößen
    int   RollStatus, Tagescheck; 
    char  Modus = 'A'; 
    bool  Neustart = true;
    bool  Abbruch = false;  // Wird true wenn die Einstellungen abgebrochen werden 
    unsigned long Startmillis, Lichtmillis, Tastermillis, Refreshmillis; // Zur Speicherung von millis() 
    int   i, x;
    // Sommer/Winterzeit
    char  Sommerzeit;
    int   Monatstage[12] = {31,59,90,120,151,181,212,243,273,304,334,365}; // Kumulierte Monatstage normales Jahr
    int   Sommerzeit_Jahr[4]  = {2021,2022,2023,2024};
    byte  Sommerzeit_Start[4] = {87,86,85,91}; // Tag des Jahres an dem die Sommerzeit startet
    int   Sommerzeit_Ende[4]  = {306,305,304,303};  // Tag des Jahres an dem die Sommerzeit endet
    // Einstellungsmenü
    String Opt[12];  // als lokale Variable verschieben??

/// Farbdefinitionen für das TFT
    #define BLACK    0x0000  // schwarz
    #define WHITE    0xFFFF  // weiß 
    #define GRAY     0x8410 
    #define SILVER   0xC618
    #define YELLOW   0xFFE0  // gelb 
    #define GREEN    0x07E0  // ST77XX_Green
    #define CYAN     0x07FF  // cyan         
    #define BLUE     0x001F  // blau (leuchtend)
    #define RED      0xF800  // rot
    // Nicht benutzte Farben:
    // #define MAROON   0x8000
    // #define GREEN    0x0400  // schwarz
    // #define GREEN    0x0E81  // gelb (Dez 3713)
    // #define OLIVE    0x8400
    // #define NAVY     0x0010  // blau (dunkel)
    // #define PURPLE   0x8010  // blau
    // #define TEAL     0x0410
    // #define LIME     0x07E0  // gelb
    // #define BLUE     0x000F  // Dez 15 // blau
    // #define MAGENTA  0xF81F  // Dez 63519 // blau
    // #define BROWN    0xFC00  // braun (Dez 64512)

// SETUP =========================================================================
void setup()
{
    /// TFT einstellen 
    //tft.begin();
    tft.initR(INITR_BLACKTAB);    // Damit die Farben stimmen
    tft.setRotation(1);           // Querformat
    tft.setTextWrap(false);       // Kein Textumbruch in neue Zeile
 
    /// Dienste starten 
    Wire.begin();  // für I2C zur RTC. Sonst geht nur manchmal die RTC nicht
    //Serial.begin(9600); // Wird nur für Ausgaben auf dem seriellen Monitor benötigt

    /// Ausgabepins der Relais definieren
    pinMode(RELAIS_Strom, OUTPUT);  // schaltet den Strom an/aus
    pinMode(RELAIS_Richt, OUTPUT);  // schatet die Richtung hoch/runter
    pinMode(TFT_Licht, OUTPUT);

    /// Interne Pullup Widerstände
    pinMode(PIN_Taster_auf,INPUT_PULLUP);
    pinMode(PIN_Taster_ab,INPUT_PULLUP);
    pinMode(PIN_Taster_stop,INPUT_PULLUP);
    pinMode(PIN_Schalter_Auto, INPUT_PULLUP);
    pinMode(PIN_Schalter_Sensor, INPUT_PULLUP);

    /// Rolladen Systemdaten initialisieren und EEPROM Daten einlesen *** 
    EPROM_LESEN();      // Daten aus dem Speicher lesen
    RollStatus = 0;     // 0:steht 1:fährt zu -1:fährt auf
    Tagescheck = 0;     // 0 erlaubt, dass der Tagescheck durchgeführt wird
    Refresh    = true;  // true erlaubt einen Display Refresh

} // ENDE SETUP


// =============================================================================================
// Hauptprogramm 
// =============================================================================================
void loop() {
       
    // Initialisierung, wenn neuer Tag begonnen wurde. Einmal pro Tag um 3 Uhr (180 Tagesminuten)!
    // oder nach Neustart
    if (Jetzt.day() != Tagescheck && Tagesminuten == 180 || Tagescheck == 0) {
        Licht_an();
        ROLLADEN_STOP();
        tft.fillScreen(BLUE); 
        tft.setTextColor(YELLOW, BLUE);
        tft.setTextSize(2);
        tft.setCursor(20,10);
        tft.print("Taeglicher");
        tft.setCursor(52,30);
        tft.print("Check");
        tft.setTextSize(1);
        tft.setCursor(0,70);
        tft.print("Version:");
        tft.setCursor(1,85);
        tft.print(Version);
        Jetzt = myRTC.now();        // holt die Zeit aus der RTC in die Variable "Jetzt"
        JULIAN();                   // speichert den laufenden Tag in Jetzt_Julien. Vor ASTRO() und SOMMERZEIT() ausführen!!
        SOMMERZEIT();               // prüfen auf Sommerzeit und ggf. Zeit um 1h verstellen. Vorher JULIAN() ausführen!
        ASTRO();                    // schreibt die SA/SU Zeiten des aktuellen Tages in alle Astro-Timer. Vorher JULIAN() ausführen! 
        TIMER_SORT();               // sortiert die Timer nach aufsteigender Uhrzeit
        TIMER_NEXT();               // ermittelt den nächsten Timer unabhängig von deren Reihenfolge
        Tagescheck = Jetzt.day();   // aktualisiert den Tagescheck- Status mit dem heutigen Tag
        delay(3000);    
        tft.fillScreen(ST7735_BLACK);
    }// endif
    
    
    // Zeitabfrage des RTC Moduls zu jeder vollen Minute
    if (((millis() - Refreshmillis)/1000 + Jetzt.second()) > 60) {
        Jetzt = myRTC.now();
        TAGESMINUTEN(); // Tageszeit in Minuten ab 0 Uhr 
        Refreshmillis = millis();
        Refresh = true;
    } // end if
    
    // Abfrage des Helligkeitssensors
    //if (UVModus && Sekunde && Modus == 'A') UVSENSOR();

    
    // Abfragen der Taster & Schalter
    Taster_auf  = digitalRead(PIN_Taster_auf);
    Taster_ab   = digitalRead(PIN_Taster_ab);
    Taster_stop = digitalRead(PIN_Taster_stop);
    Schalter_Auto   = digitalRead(PIN_Schalter_Auto);
    Schalter_Sensor = digitalRead(PIN_Schalter_Sensor);
  
    // Zeitsteuerung Ein-/Ausschalten
    if (Modus =='M' && Schalter_Auto == 0){
        Licht_an();
        Modus = 'A';
        Refresh = true;        
    }//end if
    if (Modus == 'A' && Schalter_Auto == 1){
        Licht_an();
        Modus = 'M';
        Refresh = true;
    }//end if

    if (UVModus == true && Schalter_Sensor == 1){
        Licht_an();
        UVModus = false;
        Refresh = true;
    }//end if
    if (UVModus == false && Schalter_Sensor == 0){
        Licht_an();
        UVModus = true;
        Refresh = true;
    }//end if
    

    /// Licht nach Ablauf von Licht_Dauer ausdimmen 
    if (Licht_PWM > 0) {
        x = (millis()-Lichtmillis)/1000;
        if ( x > Licht_Dauer) analogWrite(TFT_Licht, Licht_PWM - x + Licht_Dauer);
    };

    /// STOP-Taste wenn Rolladen fährt -> Anhalten       
    if (Taster_stop == 0 && RollStatus != 0) ROLLADEN_STOP();

    /// STOP-Taste wenn Rolladen steht --> Einstellungen
    if (Taster_stop == 0 && RollStatus == 0) SCHNELLMENU();

    /// Rollladen steht & Taste AUF gedrückt *******************************
    if (Taster_auf == 0 && RollStatus == 0){                   
        UVStatus = false;
        UVSignalI = 0;
        ROLLADEN_START(0);     
    }
       
    /// Rollladen steht & Taste AB gedrückt ********************************
    if (Taster_ab == 0 && RollStatus == 0){
        UVStatus = false;
        UVSignalI = 0;
        ROLLADEN_START(100);
    }

    //if (Neustart) {
        // hier muss der Code hin
    //}

    /// Displayanzeige
    if (Refresh && RollStatus == 0)  {    // Anzeige der Uhrzeit
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(40,0);
        tft.setTextSize(3);
        tft.print(TEXT(Jetzt.hour(),2,0) + ":" + TEXT(Jetzt.minute(),2,0));

        // Anzeige des Datums
        tft.setTextColor(SILVER, BLACK);
        tft.setCursor(22,32);
        tft.setTextSize(2);
        tft.print(TEXT(Jetzt.day(),2,0) + "." + TEXT(Jetzt.month(),2,0) + "." + TEXT(Jetzt.year(),4,0) + " ");
        //tft.setCursor(150,32);
        //tft.print(TEXT(Tag_heute,2,0));
        //tft.print(TEXT(Jetzt.dayOfWeek(),1,0));
        
        // Symbol für Sommer-/Winterzeit
        tft.setCursor(81,17);
        tft.setTextSize(1);
        tft.setTextColor(YELLOW, BLACK);
        tft.print(Sommerzeit);
        tft.drawLine(0,55,160,55, GRAY);
        tft.drawLine(0,89,160,89, GRAY);

        // Anzeige des nächsten Timers
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(0,99);
        tft.setTextSize(2);
        if (Modus == 'A') {
            tft.print(TEXT(UHRZEIT_h(Tzeit[T_next]),2,0) + ":" + TEXT(UHRZEIT_m(Tzeit[T_next]), 2,0) + " >> "+ TEXT(Tpos[T_next],13,0) + "% ");
        } else {
            tft.setTextColor(RED,BLACK);
            tft.setCursor(0,99);
            tft.print(" Handbetrieb ");
        } // End if

        // Anzeige Lichtsensor
        tft.setTextColor(BLACK, WHITE);
        tft.setCursor(0,120);
        tft.setTextSize(1);
        if (UVModus) tft.print("Lichtsensor: " + TEXT(UVSignalI/10000,2,0) + " Limit: " + TEXT(UVLimit,2,0) + "    ");
        else tft.print("Lichtsensor: AUS           ");   
 
        // Anzeige der RL-Position
        if (RollPos_k < 100 && RollPos_k >0){
            tft.setTextColor(CYAN, BLACK);
            tft.setCursor(48,62);
            tft.setTextSize(3);
            tft.print(TEXT(constrain(RollPos_k,0,100),3,0) + "%");
        }
        if (RollPos_k == 0){
            tft.setTextSize(2);
            tft.setTextColor(GREEN, BLACK);
            tft.setCursor(50,65);
            tft.print("offen");
        }
        if (RollPos_k == 100) { 
            tft.setTextSize(2);
            tft.setTextColor(GREEN, BLACK);
            tft.setCursor(13,65);
            tft.print("geschlossen");
        }
        // ANZEIGE VON HILFSGRÖSSEN FÜRS DEBUGGING ***********************
        /*tft.setTextColor(YELLOW, BLACK);
        tft.setTextSize(1);
        tft.setCursor(142,1);
        tft.print(TEXT(Tag_T[Ttage[T_next]],13,0));
        tft.setCursor(142,10);
        tft.print(TEXT(Tag_heute,13,0));
        tft.setCursor(142,19);
        tft.print(TEXT(Jetzt.dayOfWeek(),13,0));
        */

        Refresh = false;
    } // end if

    // Aktualisierung der RL-Position bei RL in Fahrt
    if (RollStatus != 0){
        Refresh = false;
        tft.setTextColor(BLACK, GREEN);
        tft.setCursor(48,62);
        tft.setTextSize(3);
        tft.print(TEXT(constrain(RollPos_k,0,100),3,0) + "%");
    } // end if

    // Prüft den nächsten Timer und startet den Rolladen, wenn der Timer erreicht wurde
    if (Modus == 'A' && RollStatus == 0 && Tzeit[T_next] == Tagesminuten){
        UVStatus = false;
        UVSignalI = 0;
        ROLLADEN_START(Tpos[T_next]);
    }//End if

    // Faktor = (1.0 - (0.7 * K1)) + constrain(RollPos*K1/50, 0, K1) - constrain((RollPos-90)*K1/10, 0, K1); 

    if (RollStatus == 1){
        RollPos_k = StartPos_k + (millis() - Startmillis) / (10.0 * Laufzeit_zu) ; // RL Position laufzeitbezogen    
        if (RollPos_k >= ZielPos) ROLLADEN_STOP();
    }// End if

    if (RollStatus == -1){ 
        RollPos_k = StartPos_k - (millis() - Startmillis) / (10.0 * Laufzeit_auf);
        if (RollPos_k <= ZielPos) ROLLADEN_STOP();
    }// End if
        
} // Ende loop 


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                                      Unterprogramme ab hier
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



// *** EEPROM schreiben *****************************************************************
void EPROM_SCHREIBEN(){

    /// EEPROM schreiben
    byte Adresse = 0;
    for (i=1; i<7; i++){                    // 96 byte (6 x 16byte)
        Adresse = 2 + (i-1)*16;
        EEPROM.put(Adresse + 0,Ttyp[i]);         
        EEPROM.put(Adresse + 2,Tzeit[i]);
        EEPROM.put(Adresse + 4,Tpos[i]);
        EEPROM.put(Adresse + 6,Toffset[i]);
        EEPROM.put(Adresse + 8,Tfrueh[i]);
        EEPROM.put(Adresse + 10,Tspaet[i]);    
        EEPROM.put(Adresse + 12,Trichtung[i]);
        EEPROM.put(Adresse + 14,Ttage[i]);
    }// End for
 
        EEPROM.put(100, Laufzeit_zu);        // 4 byte (float 32 bit)
        EEPROM.put(105, Laufzeit_auf);       // 4 byte (float 32 bit)
        EEPROM.put(110, Sommerzeit);         // 1 byte (char)
        EEPROM.put(115, Modus);              // 1 byte (char)
        EEPROM.put(120, UVModus);            // 1 byte (bool)
        EEPROM.put(125, UVLimit);            // 4 byte (long 32 bit)
        EEPROM.put(130, UVPos);              // 1 byte (byte 8 bit)
        EEPROM.put(135, UV_Delay_ein);       // 4 byte (unsigned long 32 bit)
        EEPROM.put(140, UV_Delay_aus);       // 4 byte (unsigned long 32 bit)
        EEPROM.put(145, Licht_PWM_max);      // 1 byte (byte 8 bit)
        EEPROM.put(150, Licht_Dauer);        // 1 byte (byte 8 bit)

        
}// Ende Funktion


// *** EEPROM auslesen und anzeigen *******************************************************************
void EPROM_LESEN()
{
    /// EEPROM einlesen  
    byte Adresse = 0;
    for (i=1; i<7; i++){
        Adresse = 2 + (i-1)*16;
        EEPROM.get(Adresse + 0,Ttyp[i]);
        EEPROM.get(Adresse + 2,Tzeit[i]);
        EEPROM.get(Adresse + 4,Tpos[i]);
        EEPROM.get(Adresse + 6,Toffset[i]);
        EEPROM.get(Adresse + 8,Tfrueh[i]);
        EEPROM.get(Adresse + 10,Tspaet[i]);
        EEPROM.get(Adresse + 12,Trichtung[i]);
        EEPROM.get(Adresse + 14,Ttage[i]);         
    }// End for
        EEPROM.get(100, Laufzeit_zu);
        EEPROM.get(105, Laufzeit_auf);
        EEPROM.get(110, Sommerzeit);    // Char
        EEPROM.get(115, Modus);         // Char
        EEPROM.get(120, UVModus);       // Bool
        EEPROM.get(125, UVLimit);       // Long
        EEPROM.get(130, UVPos);         // byte
        EEPROM.get(135, UV_Delay_ein);  // unsigned long
        EEPROM.get(140, UV_Delay_aus);  // unsigned long
        EEPROM.get(145, Licht_PWM_max); // byte
        EEPROM.get(150, Licht_Dauer);   // byte
        
        EEPROM.get(200, RollPos_k);
}// Ende Funktion

// Displaybeleuchtung einschalten
void Licht_an()
{
    Licht_PWM = Licht_PWM_max;
    analogWrite(TFT_Licht, Licht_PWM);
    Lichtmillis = millis();
} // Ende der Funktion

// *** Gibt Zahl als String zurück. ************************************************************** 
    //VK > 0:  Anzahl der Vorkommastellen mit führenden Nullen. 
    //VK > 10: Anzahl der Vorkommastellen mit führenden Leerzeichen. 
    //NK:      Anzahl der Nachkommastellen
String TEXT(float Zahl, byte VK, byte NK)
{
    String Text = "";
    
    Text = String(Zahl, NK);
    // Das erste Zeichen muss weg, da die Stringumwandlung bei Zahlen < 10 ein führendes Leerzeichen hat
    if (Zahl<10) Text = Text.substring(1,Text.length()); 

    // Mit führenden Nullen (zB 005)
    if (VK < 10 && NK == 0) while (Text.length() < VK) Text = "0" + Text;
    if (VK < 10 && NK >= 1) while (Text.length() - NK - 1 < VK) Text = "0" + Text;

    // Mit führenden Leerzeichen (z.B. __5)
    if (VK > 10 && NK == 0) while (Text.length() < VK - 10) Text = " " + Text;
    if (VK > 10 && NK >= 1) while (Text.length() - NK - 1 < VK - 10) Text = " " + Text;

    return Text;
}

// *** Aktuelles Datum in laufenden Tag des Jahres umrechnen (Julian Date) ***********************************************************
void JULIAN() 
{
    /// Initialisierung
    Jetzt_Julian = 0;
    
    /// Tage aus vollendeten Monaten. Ohne Schaltjahr. M-2 weil vollständige Monate bis 1 Monat vorher und weil Index bei 0 beginnt
    if (Jetzt.month()>= 2) Jetzt_Julian = Monatstage[Jetzt.month() - 2];

    /// Tage aus angefangenem Monat
    Jetzt_Julian = Jetzt_Julian + Jetzt.day();  

    /// Schaltjahrestag ab März
    if ((Jetzt.year()== 2020 || Jetzt.year()== 2024 || Jetzt.year()== 2028 || Jetzt.year()== 2032) && Jetzt.month()> 2 )Jetzt_Julian = Jetzt_Julian+1; 

} // Ende der Funktion

// *** Aktuelle Uhrzeit in Tagesminuten umrechnen ******************************************************************
void TAGESMINUTEN() 
{
    Tagesminuten = 60 * Jetzt.hour() + Jetzt.minute(); // rechnet die Uhrzeit in laufende Minuten des Tages um 
 
} // Ende der Funktion

// *** Rolladen starten *************************************************************************************************** 
void ROLLADEN_START(int Wert)   
{    
    Licht_an();
    
    /// Setzen der Startposition
    StartPos_k = RollPos_k;
    ZielPos = Wert;

    /// Kalibrierung der Endanschläge  
    if (ZielPos == 0)    ZielPos = -15;
    if (ZielPos == 100)  ZielPos = 115;

    /// Richtungsrelais ansteuern SCHLIEßEN.
    if (RollPos_k - ZielPos < 0) {
        digitalWrite(RELAIS_Richt, LOW); // Fahrtrichtung: LOW = runter. HIGH = hoch.
        RollStatus = +1;
    } // end if
    
    /// Richtungsrelais ansteuern ÖFFNEN.
    if (RollPos_k - ZielPos > 0) {
        digitalWrite(RELAIS_Richt, HIGH); 
        RollStatus = -1;
    } // end if
      
    /// Strom Einschalten - Los gehts
    delay(50); // Verzögerung, damit das Richtungsrelais immer vor dem Stromrelais geschaltet hat.
    digitalWrite(RELAIS_Strom, HIGH); // HIGH = Strom an!
    tft.fillRect(0,56,160,33,GREEN);
    Startmillis = millis();
    Taster_auf = 1;
    Taster_ab = 1;
    Taster_stop = 1;
    
} // Ende der Funktion



// *** Rolladen stoppen und Position ins EEPROM schreiben ***************************************************************
void ROLLADEN_STOP()
{    
    /// Rolladen stoppen
    digitalWrite(RELAIS_Strom, LOW);
    Licht_an();
    delay(100); // Verzögerung, damit der Strom immer aus ist bevor das Richtungsrelais geschaltet wird.
    digitalWrite(RELAIS_Richt, LOW);
    RollStatus = 0;
          
    /// Kalibrierung der Position bei Endanschlag
    RollPos_k = constrain(RollPos_k, 0, 100);
        
    /// Positionen in EEPROM schreiben
    EEPROM.put(200, RollPos_k);

    /// Abschluss
    TIMER_NEXT();             // Nächster Timer wird gesucht
    Refresh = true;           // Triggert den Refresh des Displays
    tft.fillRect(0,56,160,33,BLACK);
    Taster_auf = 1;
    Taster_ab = 1;
    Taster_stop = 1;

} // Ende der Funktion



// *** Sortiert die Timer aufsteigend ***************************************************************
void TIMER_SORT()
{
    /// Sortiert die Timer der Zeit nach. Wenn der nachfolgende Timer früher ist wird er mit dem vorherigen Timer getauscht. Freie Timer kommen ans Ende (Typ 4)
    for (byte i = 1; i < 7; i++) {
        for (byte k = 1; k < 6; k++) {
            if (Ttyp[k] == 4 || Tzeit[k] > Tzeit[k+1] && Ttyp[k+1] != 4){
                     Ttyp[0] = Ttyp[k];              Ttyp[k] = Ttyp[k+1];            Ttyp[k+1] = Ttyp[0];
                    Tzeit[0] = Tzeit[k];            Tzeit[k] = Tzeit[k+1];          Tzeit[k+1] = Tzeit[0];
                     Tpos[0] = Tpos[k];              Tpos[k] = Tpos[k+1];            Tpos[k+1] = Tpos[0];
                Trichtung[0] = Trichtung[k];    Trichtung[k] = Trichtung[k+1];  Trichtung[k+1] = Trichtung[0];
                  Toffset[0] = Toffset[k];        Toffset[k] = Toffset[k+1];      Toffset[k+1] = Toffset[0];
                   Tfrueh[0] = Tfrueh[k];          Tfrueh[k] = Tfrueh[k+1];        Tfrueh[k+1] = Tfrueh[0];
                   Tspaet[0] = Tspaet[k];          Tspaet[k] = Tspaet[k+1];        Tspaet[k+1] = Tspaet[0];
                    Ttage[0] = Ttage[k];            Ttage[k] = Ttage[k+1];          Ttage[k+1] = Ttage[0];
            }// End if
        }// End for
    }// End for
} // Ende der Funktion

        
// *** Ermittelt den nächsten Timer, der als nächstes auf die aktuelle Zeit folgt. ******************************* 
void TIMER_NEXT()
{ 
    // Binärcode der Wochentagsgruppe: So=1, Mo=2, Di=4, Mi=8, Do=16, Fr=32, Sa=64 
    // Gruppe 1: Mo - So = 127 (2+4+8+16+32+64+1) 
    // Gruppe 2: Mo - Fr =  62 (2+4+8+16+32)
    // Gruppe 3: Sa - So =  65 (64+1)
    // Gruppe 4: Mo - Sa = 126 (2+4+8+16+32+64)
    // Gruppe 5 bis Gruppe 11: einzelne Wochentage So - Sa (So=1, Mo=2, Di=4, Mi=8, Do=16, Fr=32, Sa=64) 
    // Tag_T = {0, 127, 62, 65, 126, 1, 2, 4, 8, 16, 32, 64}; // Binärcodes der Wochentagsgruppen
    byte Tag_morgen; // byte = 0 bis 128
    Tag_heute  = pow(2,Jetzt.dayOfWeek() + 0);   // Binärcode heutiger Wochentag (RTC Wochentage: So=0 bis Sa=6)
    Tag_morgen = pow(2,Jetzt.dayOfWeek() + 1);   // Binärcode des morgigen Wochentags
    if (Tag_morgen = 128) Tag_morgen = 1;
    
    // Schleife über alle Timer (1-6). Ermittelt den nächsten Timer heute und den ersten Timer morgen
    // Timer müssen nicht vorsortiert sein.
    TAGESMINUTEN();
    T_next = 0;  // Index, welcher Timer als nächstes kommt
    for (i=1; i<7; i++){
        // Ermittlung von T_next
        // 0. Bedingung: Timertyp ungleich auslassen (Typ4)
        if (Ttyp[i] != 4) {
            // 1. Bedingung: Timerzeit muss größer sein als aktuelle Tagesminuten    
            if (Tzeit[i] > Tagesminuten){
                // 2. Bedingung: Wochentag muss passen (Binärvergleich)
                if (Tag_heute & Tag_T[Ttage[i]]){
                    //3. Bedingung: Timerzeit [i] muss vor der Timerzeit [T_next] liegen, oder [T_next] wurde noch nicht gesetzt  
                    if ( T_next > 0 && Tzeit[i] < Tzeit[T_next] || T_next == 0){
                        // 4. Bedingung: Die Rolladenposition muss so sein, dass die Fahrbedingungen erfüllt werden.
                        if (Trichtung[i] == 1 && Tpos[i] != RollPos_k) T_next = i; // auf oder ab      
                        if (Trichtung[i] == 2 && Tpos[i]  < RollPos_k) T_next = i;  // nur auf
                        if (Trichtung[i] == 3 && Tpos[i]  > RollPos_k) T_next = i;  // nur ab
                    } // end if
                } // end if Wochentag
            } // end if Timerzeit
        } // end if Timertyp
    } // next i I

    
    // Erster Timer am darauffolgenden Tag. WIRD BENÖTIGT BEI WOCHENTAGEN!
    
    if (T_next == 0) for (i=1; i<7; i++){        
        // Ermittlung von T_next
        // 0. Bedingung: Timertyp ungleich 4 (auslassen)
        if (Ttyp[i] != 4) {        
            // 1. Bedingung: Wochentag muss passen 
            if (Tag_morgen & Tag_T[Ttage[i]]){
                //2. Bedingung: Entweder ist [T_next] noch nicht gesetzt oder die Timerzeit [i] muss vor der Timerzeit [T_next] liegen 
                if (T_next == 0 || T_next >0 && Tzeit[i] < Tzeit[T_next]){
                    // 3. Bedingung: Die Rolladenposition muss so sein, dass die Fahrbedingungen erfüllt werden.
                    if (Trichtung[i] == 1 && Tpos[i] != RollPos_k) T_next = i; // auf oder ab      
                    if (Trichtung[i] == 2 && Tpos[i]  < RollPos_k) T_next = i;  // nur auf
                    if (Trichtung[i] == 3 && Tpos[i]  > RollPos_k) T_next = i;  // nur ab
                }
            }
        }    
    }
    
} // Ende der Funktion



// Rechnet Uhrzeit im Format hhmm in Tagesminuten um ********************************************************
int MINUTEN1(int Uhrzeit){ 
    int Minuten = int(Uhrzeit / 100) * 60 + (Uhrzeit - int(Uhrzeit / 100) * 100);
    return Minuten;
} // Ende der Funktion


// Rechnet Tagesminuten in Stunde und Minute um ***********************************************************************
int UHRZEIT_h(int Tagesminuten){
    int Stunde = int(Tagesminuten / 60);
    return Stunde;
} // Ende der Funktion

int UHRZEIT_m(int Tagesminuten){
    int Minute = Tagesminuten - (int(Tagesminuten / 60)*60);
    return Minute;
} // Ende der Funktionen


// Sonnenaufgang und -untergang berechnen in Tagesminuten **********************************************************************
void ASTRO()  
{
    //JULIAN();
    // Konstanten
    float Lage_B = 49.963270;    // Breitengrad Lothringer Straße
    float Lage_L =  8.375450;    // Längengrad Lothringer Straße

    /// Initialisierung
    Jetzt = myRTC.now();
  
    /// Korrektur der Sommer/Winterzeit (Im Sommer 2h, im Winter nur 1h)
    byte Zeitzone;
    Zeitzone = 1;
    if (Sommerzeit == 'S') Zeitzone = 2;      // Zeitzone (1h = Winterzeit / 2h=Sommerzeit)
      
    /// Berechnung 
    float B = Pi*Lage_B/180;
    double Deklination = 0.4095*sin(0.016906*(Jetzt_Julian-80.086));
    float h = 0 ;  // -50 Bogenminuten = 0.0145
    float Stunden = 12*acos((sin(h) - sin(B)*sin(Deklination)) / (cos(B)*cos(Deklination)))/Pi;
    float Ortskorrektur =  -0.171*sin(0.0337 * Jetzt_Julian + 0.465) - 0.1299*sin(0.01787 * Jetzt_Julian - 0.168);
      
    /// Sonnenaufgang in Stundenbruchteilen
    Astro_auf = (12 - Stunden - Ortskorrektur - Lage_L/15 + Zeitzone)*60;   // MEZ-Zeit = Normzeit - Ortskurrektur - Lage + Zeitzone
    // am 25.Februar (Tag 56):  7,407744934 Stunden = 07:24:28 = 444,46 Minuten
    // am 19.Oktober (Tag 293): 8,023281466 Stunden = 08:01:24 = 481,39 Minuten
      
    /// Sonnenuntergang in Stundenbruchteilen und Tagesminuten
    Astro_zu = (12 + Stunden - Ortskorrektur  - Lage_L/15 + Zeitzone)*60; // MEZ-Zeit = Normzeit - Ortskurrektur - Lage + Zeitzone
    // am 25.Februar (Tag 56):  17,91051654 Stunden = 17:54:38 = 1074,63 Minuten
    // am 19.Oktober (Tag 293): 18,34552624 Stunden = 18:20:44 = 1100,73 Minuten
      
    // Timner mit neuen Astrozeiten aktualisieren
    for (i = 1; i < 7; i++)  {   
        // Sonnenaufgang
        if (Ttyp[i] == 2) Tzeit[i] = Astro_auf + Toffset[i]; 
        // Sonnenuntergang
        if (Ttyp[i] == 3) Tzeit[i] = Astro_zu + Toffset[i];
        // Korrektur über frühesten/spätesten Zeitpunkt
        Tzeit[i] = constrain(Tzeit[i], Tfrueh[i], Tspaet[i]);
    } // Ende for
        
}// Ende der Funktion


// Sommer- / Winterzeit umstellen **********************************************************************************************
void SOMMERZEIT()
{
    //JULIAN();
    // Sucht anhand des Jahres Start und Ende der Sommerzeit.
    byte Index;
    for (byte i=0 ; i<5; i++) if (Sommerzeit_Jahr[i] == Jetzt.year()) Index = i;

    if (Jetzt_Julian >= Sommerzeit_Start[Index] && Jetzt_Julian < Sommerzeit_Ende[Index]) {  
        if (Sommerzeit == 'W') {
            Jetzt = myRTC.now();
            myRTC.adjust(DateTime(Jetzt.year(), Jetzt.month(), Jetzt.day(), Jetzt.hour()+1, Jetzt.minute(), Jetzt.second()));
            Sommerzeit = 'S';
        }//endif
    } else {
        if (Sommerzeit == 'S') {
            Jetzt = myRTC.now();
            myRTC.adjust(DateTime(Jetzt.year(), Jetzt.month(), Jetzt.day(), Jetzt.hour()-1, Jetzt.minute(), Jetzt.second()));
            Sommerzeit = 'W';
        }//endif
    }//endif
    EPROM_SCHREIBEN();
} // Ende der Funktion

//==================================================================================================================
// AB HIER KOMMEN DIE EINSTELLUNGSMENÜS
//==================================================================================================================

// Schnellmenue
void SCHNELLMENU()
{
    // Verzögerung der Anzeige
    Licht_an();
    Tastermillis = millis();
    while (millis() - Tastermillis <= 1000 && digitalRead(PIN_Taster_stop) == 0);
        if (digitalRead(PIN_Taster_stop) == 0){
            long i = 0;
            int Warte = 2000; // in Millisekunden
            tft.fillScreen(BLACK); 
            tft.setTextColor(WHITE, BLACK);
            tft.setTextSize(2);
            tft.setCursor(02,50);
            tft.print("Einstellungen");
            while (digitalRead(PIN_Taster_stop) == 0 && i <= 160){  
                i = i + 1;
                tft.drawLine( i, 75, i, 95, GREEN);  // Bei i=160 ist der Balken voll
                delay(10);
            }// End while
            
            if (i >= 160) MENU();
            tft.fillScreen(BLACK);
        }
        Taster_auf  = 1;
        Taster_ab   = 1;
        Taster_stop = 1;
        Refresh = true;
        Licht_an();

}// Ende Schnellmenue

// *** MELDUNG *****************************************************************************************
// Zaubert eine Meldung auf das Display
void MELDUNG(String Zeile1, String Zeile2, unsigned int Farbe_V, unsigned int Farbe_H, int Zeit) 
{    
    // tft.fillScreen(BLACK);
    tft.fillRoundRect(2,35,156,60,10,Farbe_H); 
    tft.setTextColor(Farbe_V);
    tft.setCursor(15,46);
    tft.print(Zeile1);
    tft.setCursor(15,70);
    tft.print(Zeile2);
    delay(Zeit);
    tft.fillScreen(BLACK);
}

// HAUPTMENÜ ********************************************************************************************
void MENU()
{
    /// Initialisierung
    int Wahl = 2;
    Abbruch = false;

    while (!Abbruch) {

        /// Menü aufbauen und anzeigen 
        MASKE1("HAUPTMENUE", "Mit schwarzen Tasten", "Menuepunkt waehlen",0,"",0,"",0,"");        
        Opt[0] = "";
        Opt[1] = "";
        Opt[2] = "Timer";
        Opt[3] = "Uhr & Datum";
        Opt[4] = "Lichtsensor";
        Opt[5] = "Rollladen";
        Opt[6] = "Display";
    
        while (digitalRead(PIN_Taster_stop) == 0) delay(100);
            
        Wahl = EINSTELLUNG2(2, 6, 2); // (Min, Max, Vorauswahl)
        if (!Abbruch){
            if (Wahl == 2) TIMER_STELLEN();
            if (Wahl == 3) RTC_STELLEN();
            if (Wahl == 4) UVSENSOR_STELLEN();
            if (Wahl == 5) LAUFZEIT_STELLEN();
            if (Wahl == 6) ANZEIGE();
            Abbruch = false;
        }
    } // End While

    // Astrozeiten setzen, Timer sortiern und nächsten Timer ermitteln
    ASTRO();
    TIMER_SORT();
    TIMER_NEXT();
       
} //Ende Funktion



// *** Einstellungsmenü der Timer *********************************************************************
void TIMER_STELLEN()
{
    /// Initialisierung
    int Stunde = 0;
    int Minute = 0;
    byte NR = 1;
    Abbruch = false;
      
    /// Auswahl des zu stellenden Timers
    if (!Abbruch) {
        MASKE1("Timer", "Bitte den Timer waehlen","",0,"",0,"",0,"");
        Opt[0] = "";
        for (i = 1; i < 7; i++) {
            Opt[i] = TEXT(UHRZEIT_h(Tzeit[i]), 2,0) + ":" + TEXT(UHRZEIT_m(Tzeit[i]), 2,0) + " " + TEXT(Tpos[i],3,0) + "% ";
            if (Ttyp[i] == 1) Opt[i] = Opt[i] + "FZ ";
            if (Ttyp[i] == 2) Opt[i] = Opt[i] + "SA ";  
            if (Ttyp[i] == 3) Opt[i] = Opt[i] + "SU ";
            if (Ttyp[i] == 4) Opt[i] = "frei         ";
        }// End for
        x = EINSTELLUNG2(1, 6, 1);
        if (!Abbruch) NR = x;
    }

    /// Typ für den ausgewählten Timer einstellen
    if (!Abbruch){
        MASKE1("Timertyp", "Bitte den Timertyp waehlen","",0,"",0,"",0,"");
        Opt[1] = "feste Zeit";
        Opt[2] = "Sonnenaufgang";
        Opt[3] = "Sonnenuntergang";
        Opt[4] = "auslassen"; 
        Opt[5] = "";
        x = EINSTELLUNG2(1, 4, Ttyp[NR]);
        if (!Abbruch) Ttyp[NR] = x;
    }    

    // festen Zeitpunkt einstellen
    if (!Abbruch && Ttyp[NR] == 1) {

        Stunde = UHRZEIT_h(Tzeit[NR]);
        Minute = UHRZEIT_m(Tzeit[NR]);
        
        MASKE1("Zeitpunkt", "Bitte die Schaltuhrzeit ", "einstellen", 30, TEXT(Stunde,2,0) + ":", 65, TEXT(Minute,2,0) + " Uhr",0,"");   
        
        Stunde = EINSTELLUNG(0, 23, Stunde, 1, 30, 65, CYAN, BLACK);
        Minute = EINSTELLUNG(0, 59, Minute, 1, 65, 65, CYAN, BLACK);
        if (!Abbruch) Tzeit[NR] = Stunde*60 + Minute;
   }// End if
    
    if (Ttyp[NR] != 4 ) {
       
        // Wochentage einstellen 
        if(!Abbruch){
            MASKE1("Wochentage", "Bitte Wochentage festlegen","",0,"",0,"",0,"");
            Opt[1]  = "Mo - So";
            Opt[2]  = "Mo - Fr";
            Opt[3]  = "Sa - So";
            Opt[4]  = "Mo - Sa"; 
            Opt[5]  = "nur So ";
            Opt[6]  = "nur Mo ";
            Opt[7]  = "nur Di ";
            Opt[8]  = "nur Mi ";
            Opt[9]  = "nur Do ";
            Opt[10] = "nur Fr ";
            Opt[11] = "nur Sa ";

            x = EINSTELLUNG2(1, 11, Ttage[NR]);
            if (!Abbruch) Ttage[NR] = x;
        }
        // Position einstellen
        if (!Abbruch) {
            MASKE1("Position", "Auf welche Position soll", "der Rollladen fahren", 50, "    %",0,"",0,"");  
            x = EINSTELLUNG(0, 100, Tpos[NR], 1, 50, 65, CYAN, BLACK);
            if (!Abbruch) Tpos[NR] = x;
        }

        // Richtung einstellen 
        if (!Abbruch) {
            MASKE1("Richtung", "In welche Richtung darf","der Rollladen fahren?",0,"",0,"",0,"");
            Opt[1] = " auf und ab  ";
            Opt[2] = " nur auf     ";
            Opt[3] = " nur ab      ";        
            x = EINSTELLUNG2(1, 3, Trichtung[NR]);
            if (!Abbruch) Trichtung[NR] = x;
        }
    } // End if
           
    /// Bei ASTRO Funktion (Typ 2 und 3) müssen noch Offset, Spät und Früh eingegeben werden 
    if (Ttyp[NR] == 2 || Ttyp[NR] == 3){
          
        // Offset
        if (!Abbruch) {
            MASKE1("Offset", "Zeitliche Verschiebung zum", "Sonnenaufgang / Sonnenuntergang", 50, "    Min.", 0,"",0,""); 
            x = EINSTELLUNG(-99, 99, Toffset[NR], 1, 50, 65, CYAN, BLACK);
            if (!Abbruch) Toffset[NR] = x;
        }
              
        //Frühester Zeitpunkt
        if (!Abbruch) {
            Stunde = UHRZEIT_h(Tfrueh[NR]); // Muss vorab definiert werden. Der Funktionsaufruf MASKE1() verträgt keine Funktionen im Aufruf
            Minute = UHRZEIT_m(Tfrueh[NR]);
            MASKE1("Fruehestens", "Uhrzeit, wann der Rollladen", "fuehestens fahren darf", 35, TEXT(Stunde,2,0) + ":", 70, TEXT(Minute,2,0) + " Uhr",0,"");        
            Stunde = EINSTELLUNG(0, 23, Stunde, 1, 35, 65, CYAN, BLACK);
            Minute = EINSTELLUNG(0, 59, Minute, 1, 70, 65, CYAN, BLACK);
            if (!Abbruch) Tfrueh[NR] = Stunde*60 + Minute;
        }
        //Spätester Zeitpunkt
        if (!Abbruch){
            Stunde = UHRZEIT_h(Tspaet[NR]); 
            Minute = UHRZEIT_m(Tspaet[NR]);
            MASKE1("Spaetestens", "Uhrzeit, wann der Rollladen", "spaetestens fahren muss", 35, TEXT(Stunde,2,0) + ":", 70, TEXT(Minute,2,0) + " Uhr",0,"");        
            Stunde = EINSTELLUNG(0, 23, Stunde, 1, 35, 65, CYAN, BLACK);
            Minute = EINSTELLUNG(0, 59, Minute, 1, 70, 65, CYAN, BLACK);
            if (!Abbruch) Tspaet[NR] = Stunde*60 + Minute;
        }
    } //End if

    if (!Abbruch) {
        EPROM_SCHREIBEN(); 
        MELDUNG("Im EEPROM", "gespeichert", BLACK, GREEN, 3000);    
    }

} // Ende der Funktion


// *** REAL TIME CLOCK einstellen *********************************************************************
void RTC_STELLEN()
{
    int Tag, Monat, Jahr, Stunde, Minute, Sekunde;
    Jetzt = myRTC.now(); // Zeit und Datum aus RTC abfragen
    Abbruch = false;

    MASKE1("Datum", "Datum der RealTimeClock", "einstellen", 15, TEXT(Jetzt.day(),2,0) + ".", 50, TEXT(Jetzt.month(),2,0) + "." , 85, TEXT(Jetzt.year(),4,0));
    if (!Abbruch) Tag   = EINSTELLUNG(0 , 31, Jetzt.day() , 1, 15, 65, CYAN, BLACK);
    if (!Abbruch) Monat = EINSTELLUNG(0 , 12, Jetzt.month(), 1, 50, 65, CYAN, BLACK);
    if (!Abbruch) Jahr  = EINSTELLUNG(2020 , 2030, Jetzt.year(), 1, 85, 65, CYAN, BLACK);
    
    Jetzt = myRTC.now(); // Zeit und Datum aus RTC abfragen

    MASKE1("Uhrzeit", "Uhrzeit der Real Time Clock", "einstellen", 20, TEXT(Jetzt.hour(),2,0) + ":", 55, TEXT(Jetzt.minute(),2,0) + " Uhr",0,"");
    if (!Abbruch) Stunde = EINSTELLUNG(0 , 23, Jetzt.hour(), 1,  20, 65, CYAN, BLACK);
    if (!Abbruch) Minute = EINSTELLUNG(0 , 59, Jetzt.minute(), 1, 55, 65, CYAN, BLACK);

    // Daten in die RTC schreiben
    if (!Abbruch) {
        Sekunde = 1; 
        myRTC.adjust(DateTime(Jahr, Monat, Tag, Stunde, Minute, Sekunde)); 
        MELDUNG("Im EEPROM", "gespeichert", BLACK, GREEN, 3000);    
        SOMMERZEIT();
    }      

} // Ende der Funktion


// *** Displaybeleuchtung einstellen**** ****************************************************
void ANZEIGE()
{
    Abbruch = false;

    if (!Abbruch){ 
        MASKE1("Helligkeit", "Helligkeit des Displays,","in einstellen",50,"   %  ",0,"",0,"");
        x = EINSTELLUNG(05 , 110, Licht_PWM_max, 1, 50, 65, CYAN, BLACK);
        if (!Abbruch) Licht_PWM_max = x;
    }
    if (!Abbruch){
        MASKE1("Dauer", "Beleuchtungsdauer","in s einstellen",50,"   s ",0,"",0,"");
        x = EINSTELLUNG(10, 120, Licht_Dauer, 1, 50, 65, CYAN, BLACK);
        if (!Abbruch) Licht_Dauer = x;
    }
    if (!Abbruch) {
        EPROM_SCHREIBEN(); 
        MELDUNG("Im EEPROM", "gespeichert", BLACK, GREEN, 3000);    
    }
}


// *** Laufzeit des Rollladens einstellen ****************************************************
void LAUFZEIT_STELLEN()
{// Messung der Laufzeiten ab- und aufwärts
       
    /// TFT aufbauen
    float Laufzeit_auf_alt = Laufzeit_auf;
    float Laufzeit_zu_alt  = Laufzeit_zu;
    Abbruch = false;
    
    MASKE1("Laufzeit ZU", "auf: " + String(Laufzeit_auf) + "s ","zu:  " + String(Laufzeit_zu) + "s",0,"",0,"",0,"");
    Opt[1] = "Messung";
    Opt[2] = "Werte eingeben";
    Opt[3] = "";
    Opt[4] = ""; 
    byte Wahl = EINSTELLUNG2(1, 3, 1);
    
    if (!Abbruch && Wahl == 1) {
        /// Rollladen in Ausgangsposition oben bringen 
        digitalWrite(RELAIS_Richt, HIGH);   // aufwärts
        delay(100);
        digitalWrite(RELAIS_Strom, HIGH);  // Strom an

        // Start der Messung ZU
        if (!Abbruch) {
            MASKE1("Laufzeit ZU", "Wenn Rollladen ganz geoeffnet","ROTE Taste druecken",0,"",0,"",0,"");
            Opt[1] = "Start!";
            Wahl = EINSTELLUNG2(1, 1, 1);    
            if (!Abbruch) digitalWrite(RELAIS_Richt, LOW);    // abwärts
            Startmillis = millis();             // Start der Messung. 
        }

        // Stop der Laufzeitmessung ZU
        if (!Abbruch) {
            MASKE1("Laufzeit ZU", "Wenn Rollladen geschlossen","SOFORT ROTE Taste druecken",0,"",0,"",0,"");
            Opt[1] = "geschlossen";
            Wahl = EINSTELLUNG2(1, 1, 1);    
            Laufzeit_zu = (float) 10.0 * (millis() - Startmillis)/1000;    
            Laufzeit_zu  = abs(Laufzeit_zu) / 10.0;

            /// Start der Laufzeitmessung AUF nach 1 Sekunde
            delay(5000); // 5 Sekunden warten   
            digitalWrite(RELAIS_Richt, HIGH);   // aufwärts
            Startmillis = millis();             // Start der Messung

            /// Stop der Laufzeitmessung ZU 
            MASKE1("Laufzeit AUF", "Wenn Rollladen offen,","SOFORT ROTE Taste druecken",0,"",0,"",0,"");
            Opt[1] = "Stop    ";
            Wahl = EINSTELLUNG2(1, 1, 1);
            Laufzeit_auf = (float) 10.0 * (millis() - Startmillis)/1000;    
            Laufzeit_auf = abs(Laufzeit_auf) / 10.0;       
        }

        // Relais IMMER beide wieder abschalten
        digitalWrite(RELAIS_Strom, LOW);    // Strom aus    
        delay(100);
        digitalWrite(RELAIS_Richt, LOW);    // Richtungsrelais aus
        
    }// end if

    if (!Abbruch && Wahl==2) {
        Abbruch = false;
        if (!Abbruch) {
            MASKE1("Laufzeit ZU", "Bitte Laufzeit,","in Sekunden eingeben",50,"    s",0,"",0,"");
            x = 10 * EINSTELLUNG(0 , 30, Laufzeit_zu, 0.1, 50, 65, CYAN, BLACK); // x ist int
            if (!Abbruch) Laufzeit_zu = x / 10.0;
        }
        if (!Abbruch) {
            MASKE1("Laufzeit AUF", "Bitte Laufzeit,","in Sekunden eingeben",50,"    s",0,"",0,"");
            x = 10 * EINSTELLUNG(0 , 30, Laufzeit_auf, 0.1, 50, 65, CYAN, BLACK);
            if (!Abbruch) Laufzeit_auf = x / 10.0;
        }
    }// end if


    // Anzeige der neuen und alten Werte 
    if (!Abbruch && Wahl > 0) {
        tft.fillScreen(BLACK);
        tft.setTextColor(WHITE, BLACK);
        tft.setTextSize(2);
        tft.setCursor(3,0);
        tft.print("LAUFZEITEN");
        tft.drawLine(0,18,160,18,GRAY); 
        byte POSx = 10;
        tft.setCursor(POSx,30);
        tft.print("AUF: " + String(Laufzeit_auf,1) + "s ");
        tft.setCursor(POSx,80);
        tft.print("ZU:  " + String(Laufzeit_zu,1) + "s ");
        
        tft.setTextColor(GRAY, BLACK);
        tft.setCursor(POSx,53);
        tft.print("    (" + String(Laufzeit_auf_alt,1) + "s) ");
        tft.setCursor(POSx,103);
        tft.print("    (" + String(Laufzeit_zu_alt,1) + "s) ");  
        
        RollPos_k = 0;
        while (digitalRead(PIN_Taster_stop) == 1);
       
        EPROM_SCHREIBEN(); 
        MELDUNG("Im EEPROM", "gespeichert", BLACK, GREEN, 3000);    
    }

    /// Abschluss
    ROLLADEN_STOP;
     
} // Ende der Funktion



// Einstellungsfunktion I für Werte ***********************************************************
float EINSTELLUNG(int Wert_min, int Wert_max, float Wert_start, float Schritt, byte POS_x, byte POS_y, unsigned int Farbe_v, unsigned int Farbe_h)
{
    String Print;    
    float Wert = Wert_start;
    
    // Stop-Taster klären
    while (digitalRead(PIN_Taster_stop) == LOW) delay(10);

    // Schleife solange bis die Stop Taste gedrückt wurde. Stop = ok.
    Taster_stop = 1; Taster_auf = 1; Taster_ab = 1;
    tft.setTextColor(Farbe_h, Farbe_v);
    tft.setTextSize(2);
    while (Taster_stop == 1){
        if (Taster_auf == 0) Wert = Wert + Schritt;
        if (Taster_ab  == 0) Wert = Wert - Schritt;
        
        if (Wert > Wert_max) Wert = Wert_min;     
        if (Wert < Wert_min) Wert = Wert_max;

        Print = "";
        if (Wert_min < 0 && Wert >= 0) Print = "+";
        if (Wert_min < 0 && Wert  < 0) Print = "-"; 
        if (Wert_max >= 10 && abs(Wert) <  10) Print = Print + "0";
        if (Wert_max >=100 && abs(Wert) < 100) Print = Print + "0";
        
        if (Schritt >= 1) Print = Print + String(abs(Wert));
        if (Schritt <  1) Print = Print + String(Wert,1);

        tft.setCursor(POS_x,POS_y);
        tft.print(Print);

        do {
            Taster_auf  = digitalRead(PIN_Taster_auf);
            Taster_ab   = digitalRead(PIN_Taster_ab);
            Taster_stop = digitalRead(PIN_Taster_stop);
        } while (Taster_auf + Taster_ab + Taster_stop == 3);
        
        delay(150); // regelt, wie schnell die Zahlen auf dem Display durchlaufen.    
    } // End While Taster_stop == 1

    Abbruch = false;
    i = 1;
    while(digitalRead(PIN_Taster_stop) == LOW && Abbruch == false) {
        i = i + 1;
        delay(10);
        if (i > 250) {
            Abbruch = true;
            MELDUNG("Vorgang","abgebrochen", BLACK, SILVER, 3000);
        }   
    }
    tft.setTextColor(Farbe_v, Farbe_h);
    tft.setTextSize(2);
    tft.setCursor(POS_x,POS_y);
    tft.print(Print);
    while(digitalRead(PIN_Taster_stop) == LOW) delay(10);
 
    return Wert;
  
} // Funktion Ende

// EINSTELLUNGSFUNKTION2 FÜR MENÜS ***************************************************************
int EINSTELLUNG2(int Wert_min, int Wert_max, int Wert_get)
{
    Taster_stop = 1; Taster_ab = 1; Taster_auf = 1;
    tft.setTextSize(2);
    tft.setTextColor(CYAN, BLACK);
    Abbruch = false;

    /// Schleife solange bis die Stop Taste gedrückt wurde. Stop = ok.
    while (!Abbruch && Taster_stop == 1) {    
        tft.setCursor(3, 65);
        tft.print(Opt[Wert_get] + "      ");
            
        // Einlesen der Taster und Entprellen 
        delay(200); // Zum Entprellen
        Startmillis = millis();
        do {
            Taster_auf = digitalRead(PIN_Taster_auf);
            Taster_ab  = digitalRead(PIN_Taster_ab);
            Taster_stop= digitalRead(PIN_Taster_stop);
            if (millis() - Startmillis > 60000) Abbruch = true; // Nach 1min Rückspung um eine Ebene 
        } while (!Abbruch && Taster_auf + Taster_ab + Taster_stop == 3);
           
        // Scrollen entsprechend Tastendruck
        if (Taster_ab  == 0) Wert_get = constrain(Wert_get + 1, Wert_min, Wert_max);
        if (Taster_auf == 0) Wert_get = constrain(Wert_get - 1, Wert_min, Wert_max);

    } // end while

    /// Taster klären und Rückgabe
    i = 0;
    while(digitalRead(PIN_Taster_stop) == LOW && Abbruch == false) {
        i = i + 1;
        delay(10);
        if (i > 250) {
            Abbruch = true;
            MELDUNG("Vorgang","abgebrochen", BLACK, SILVER, 3000);
        }   
    }

    return Wert_get;

} // Ende der Funktion


//==================================================================================================================
// AB HIER KOMMT DER UV SENSOR
// =================================================================================================================

void UVSENSOR() {
    /* Reading  UV-Index 
    *   <10      0
    *    46      1
    *    65      2
    *    83      3
    *   103      4
    *   124      5
    *   142      6
    *   162      7
    *   180      8
    *   200      9
    *   221     10
    *   240     11+
    */

    // Maximal von mir gemessenes Signal bei 3,3V ist 700 bei voller Sonne hinter der Scheibe im Wohnzimmer.

    int UV_max = 700; // maximales Signal vom Sensor   
    unsigned long Zeit, UVmillis;
       
    UVSignal = analogRead(A0); // sensorVoltage = sensorValue/1024*3.3;
    int UV_norm = 100*UVSignal / UV_max;  // Eingangssignal normiert auf 0-100

    /// Wenn UV-Schutz nicht ausgelöst ist, wird bei jeder ÜBERschreitung des UV-Limits die Zeit aufaddiert
    if (UVStatus == false) {
        Zeit = constrain(Zeit, 0, UV_Delay_ein * 1000);
        if (UV_norm > UVLimit) Zeit = Zeit + (millis() - UVmillis);
        if (UV_norm < UVLimit) Zeit = Zeit - (millis() - UVmillis);
        UVmillis = millis();
        
        // Ist die aufaddierte Zeit positiver als das positive Zeitlimit (UV_Delay_ein), dann wird der Rolladen in die UVPos gefahren
        if (Zeit >= +UV_Delay_ein * 1000 && UVModus == true && RollStatus == 0 && RollPos_k < UVPos) {
            UVPosMerker = RollPos_k;
            UVStatus = true;
            ROLLADEN_START(UVPos); 
        }//end if
    } //end if;

    /// Wenn der UV-Schutz ausgelöst ist, wird bei jeder UNTERschreitung des UV-Limits die Zeit aufaddiert
    if (UVStatus == true) {
        Zeit = constrain(Zeit, 0, -(UV_Delay_aus * 1000));
        if (UV_norm < UVLimit) Zeit = Zeit + (millis() - UVmillis);
        if (UV_norm > UVLimit) Zeit = Zeit - (millis() - UVmillis);
        UVmillis = millis();
        
        // Ist die aufaddierte Zeit größer als das Zeitlimit (UV_Delay_aus), dann wird der Rolladen in die alte Position (UVPosMerker) zurückgefahren
        if (Zeit >= UV_Delay_aus * 1000 && UVModus == true && RollStatus == 0){
            UVStatus = false;
            ROLLADEN_START(UVPosMerker);;
        }//end if
    }//end if

    //UVLimit augeben zwischen 0 und 100 (byte)
    //UV_Delay_ein eingeben in Sekunden (unsigned long) 
    //UV_Delay_aus eingeben in Sekunden (unsigned long)

    /// Sensorfehler erkennen
    //if (abs(UVSignal-analogRead(A0))> 50) {
    //LCDPRINT(1,0,0, "! Fehler !",0);
    //LCDPRINT(0,0,1, "! UV Sensor !",5000);
    //}
      
}// Ende der Funktion

void UVSENSOR_STELLEN(){
    
    Abbruch = false;
    
    // UVLimit: UV Grenzwert, ab dem die UV Funktion aktiviert wird 
    if (!Abbruch) {
        MASKE1("Helligkeit", "Helligkeit, bei der sich der","Rolladen schliesst.",50,"    %",0,"",0,"");
        x = EINSTELLUNG(0, 100, UVLimit, 1, 50, 65, CYAN, BLACK);
        if (!Abbruch) UVLimit = x;
    }
    // UVPos: Position, die der Rollladen anfahren soll (0 bis 100%) 
    if (!Abbruch) {
        MASKE1("Position", "Position, die der ","Rollladen anfaehrt",50,"    %",0,"",0,"");
        x = EINSTELLUNG(0, 100, UVPos, 1, 50, 65, CYAN, BLACK);
        if (!Abbruch) UVPos = x;
    }
    // UV_Delay_ein: Verzögerung in Sekunden bis zur Auslösung des UV-Schutzes 
    if (!Abbruch) {
        MASKE1("Verzoegerung", "Verzoegerung beim Schließen","des Rollladens",40, "   Min.",0,"",0,"");
        x = 60 * EINSTELLUNG(0, 15, UV_Delay_ein / 60, 1, 40, 65, CYAN, BLACK);
        if (!Abbruch) UV_Delay_ein = x;
    }
    // UV_Delay_aus: Verzögerung in Sekunden bis zur Rückstellung des UV-Schutzes 
    if (!Abbruch) {
        MASKE1("Verzoegerung", "Verzoegerung beim Oeffnen","des Rollladens",40,"   Min. " ,0,"",0,"");
        x = 60 * EINSTELLUNG(0, 15, UV_Delay_aus / 60, 1, 40, 65, CYAN, BLACK);   
        if (!Abbruch) UV_Delay_aus = x;
    } 
    if (!Abbruch) {
        EPROM_SCHREIBEN(); 
        MELDUNG("Im EEPROM", "gespeichert", BLACK, GREEN, 3000);    
    }

}// Ende der Funktion



void MASKE1(String Text1, String Text2a, String Text2b, byte POS_x1, String Wert1, byte POS_x2, String Wert2, byte POS_x3, String Wert3)       
{
    tft.fillScreen(BLACK);
    tft.setTextColor(YELLOW, BLACK);
    tft.setTextSize(2);
    tft.setCursor(3,0);
    tft.print(Text1);

    tft.setTextSize(1);
    tft.setTextColor(SILVER, BLACK);
    tft.setCursor(0,25);
    tft.print(Text2a);
    tft.setCursor(0,38);    
    tft.print(Text2b);

    tft.setTextSize(2);
    tft.setTextColor(CYAN, BLACK);
    tft.setCursor(POS_x1,65);
    if (Wert1 != "") tft.print(Wert1);
    tft.setCursor(POS_x2,65);
    if (Wert2 != "") tft.print(Wert2);
    tft.setCursor(POS_x3,65);
    if (Wert3 != "") tft.print(Wert3);
    
    tft.drawLine(0,55,160,55, GRAY);
    tft.drawLine(0,89,160,89, GRAY);  
    
}
