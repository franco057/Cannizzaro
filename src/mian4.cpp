#include "vex.h"
#include <thread>

using namespace vex;
using namespace std::chrono;

// Dichiarazione dei componenti hardware
brain Brain;
optical SensoreOttico(PORT2); 
optical SensoreOttico2(PORT6);
motor braccio(PORT9, gearSetting::ratio18_1, false);
motor pinza(PORT4, gearSetting::ratio18_1, false);

motor leftA = motor(PORT11, gearSetting::ratio18_1, false);
motor leftB = motor(PORT1, gearSetting::ratio18_1, false);
motor_group leftMotors = motor_group(leftA, leftB);

motor rightA = motor(PORT10, gearSetting::ratio18_1, true);
motor rightB = motor(PORT12, gearSetting::ratio18_1, true);
motor_group rightMotors = motor_group(rightA, rightB);

inertial Inertial = inertial(PORT7);

smartdrive Smartdrive = smartdrive(leftMotors, rightMotors, Inertial, 100.0, 260.9, 127.3, mm);

// Costanti per le posizioni
const double PINZA_APERTA = -66.5;
const double PINZA_CHIUSA = 60.5;
const double BRACCIO_ALZATO = 35.5;
const double BRACCIO_ABBASSATO = 120.0;
const int VELOCITA_BRACCIO = 20;
const int VELOCITA_PINZA = 20;

// Costanti per il movimento - aggiornate in base ai dati TAVOLA 1 e 2
const double FATTORE_CORREZIONE_AVANTI = 0.3333;
const double FATTORE_CORREZIONE_INDIETRO = 0.3333;
const double FATTORE_CORREZIONE_ROTAZIONE = 1.0;
const int VELOCITA_ROTAZIONE = 25;
const int VELOCITA_AVANZAMENTO = 35;

// Dimensioni del campo da TAVOLA 1
const int LARGHEZZA_CAMPO = 152; // cm
const int LUNGHEZZA_CAMPO = 274; // cm
const int LATO_AREA_PARTENZA = 30; // cm - Area di partenza/arrivo
const int LARGHEZZA_AREA_STALLO = 40; // cm - Area blu stallo automobili
const int LUNGHEZZA_AREA_STALLO = 100; // cm - Area blu stallo automobili
const int LARGHEZZA_COLONNINA = 20; // cm - Colonnine rosse
const int LUNGHEZZA_COLONNINA = 50; // cm - Colonnine rosse
const int LARGHEZZA_OFFICINA = 20; // cm - Officine verde/gialla
const int LUNGHEZZA_OFFICINA = 50; // cm - Officine verde/gialla

// Distanze specifiche da TAVOLA 1
const int DISTANZA_AREA_STALLO_X = 41; // Distanza dall'area di partenza all'area stallo (asse X)
const int DISTANZA_AREA_STALLO_Y = 64; // Distanza dall'area di partenza all'area stallo (asse Y)
const int DISTANZA_COLONNINA1_X = 61; // Distanza approssimativa per la prima colonnina
const int DISTANZA_COLONNINA2_X = 132; // Distanza approssimativa per la seconda colonnina
const int DISTANZA_COLONNINA3_X = 203; // Distanza approssimativa per la terza colonnina
const int DISTANZA_OFFICINA_GIALLA_Y = 20; // Distanza Y dell'officina gialla
const int DISTANZA_OFFICINA_VERDE_Y = 50 + LARGHEZZA_CAMPO - 20; // Distanza Y dell'officina verde

// Costanti per i sensori
const int POTENZA_LED_NORMALE = 50;
const int POTENZA_LED_BASSA_LUMINOSITA = 100;
const int SOGLIA_BASSA_LUMINOSITA = 80;
const int NUMERO_CAMPIONI_COLORE = 5;
const int RITARDO_LETTURA_COLORE = 200; // millisecondi

// Variabili globali
bool osFront = false;
bool osLat = false;
char coloreRilevato = 'n'; // 'n' = nessuno, 'r' = rosso, 'g' = giallo, 'v' = verde, 'b' = blu
bool threadAttivo = true;
bool giallo = false;
bool verde = false;
int c1,c2,c3;
int distanzaTotale = 0; // Contatore distanza totale avanti

// Variabili per la strategia di percorso
int autoRosseRaccolte = 0;
int autoGialleRaccolte = 0;
int autoBluRaccolte = 0;
int autoInColonnina1 = 0;
int autoInColonnina2 = 0;
int autoInColonnina3 = 0;
bool officinaVerde = false; // false = officina gialla, true = officina verde
int posizioneX = 0; // Posizione X del robot (per mappatura)
int posizioneY = 0; // Posizione Y del robot (per mappatura)

// Espansione triport
triport expander = triport(PORT3);
potV2 potBraccio(expander.A);
potV2 potPinza(expander.B);

/**
 * Thread di controllo per il rilevamento di oggetti frontali
 */
void checkFront() {
    while(threadAttivo) {
        osFront = SensoreOttico2.isNearObject();
        this_thread::sleep_for(milliseconds(50));
    }
}

/**
 * Inizializza il robot e rileva il colore dell'area di partenza
 */
void inizio() {
    float distanza = 200; // millimetri
    int velocita = 55;
    int conteggioVerde = 0;
    int conteggioGiallo = 0;

    // Correzione della distanza
    float distanzaCorretta = distanza * FATTORE_CORREZIONE_AVANTI;
    Smartdrive.driveFor(forward, distanzaCorretta, mm, velocita, rpm);
    distanzaTotale += distanza; // Aggiorna contatore distanza

    // Controlla continuamente se un oggetto è rilevato frontalmente
    while(Smartdrive.isMoving()) {
        if(osFront) {
            Smartdrive.stop(hold);
            break;
        }
        this_thread::sleep_for(milliseconds(10));
    }

    // Configurazione del sensore ottico
    SensoreOttico.setLightPower(POTENZA_LED_NORMALE, percent);
    SensoreOttico.setLight(ledState::on);

    // Rileva il colore con più campioni per aumentare l'affidabilità
    for(int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
        if (SensoreOttico.isNearObject()) {
            int hue = SensoreOttico.hue(); 
            double brightness = SensoreOttico.brightness(); 
            int lightPower = POTENZA_LED_NORMALE;

            // Aumenta la potenza del LED se la luminosità è bassa
            if (brightness < SOGLIA_BASSA_LUMINOSITA) {  
                lightPower = POTENZA_LED_BASSA_LUMINOSITA;
                SensoreOttico.setLightPower(lightPower, percent);
            } 

            // Classifica il colore basato sull'hue con range migliorati
            // Range di verde più specifico: 110-150
            if (hue >= 30 && hue < 70) {
                conteggioGiallo++;
                Brain.Screen.setCursor(i+2, 1);
                Brain.Screen.print("Campione %d: Giallo (Hue: %d)", i+1, hue);
            } else if (hue >= 110 && hue < 150) {
                conteggioVerde++;
                Brain.Screen.setCursor(i+2, 1);
                Brain.Screen.print("Campione %d: Verde (Hue: %d)", i+1, hue);
            }
            
            // Aggiungi un breve delay per stabilizzare la lettura
            this_thread::sleep_for(milliseconds(50));
        }
        this_thread::sleep_for(milliseconds(RITARDO_LETTURA_COLORE));
    }

    // Determina il colore finale basato sul conteggio
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    
    if(conteggioVerde > conteggioGiallo) {
        Brain.Screen.print("Verde (Conteggio: %d)", conteggioVerde);
        verde = true;
        giallo = false;
        officinaVerde = true;
        Brain.Screen.newLine();
        Brain.Screen.print("Officina disponibile: VERDE");
    } else if(conteggioGiallo > conteggioVerde) {
        giallo = true;
        verde = false;
        officinaVerde = false;
        Brain.Screen.print("Giallo (Conteggio: %d)", conteggioGiallo);
        Brain.Screen.newLine();
        Brain.Screen.print("Officina disponibile: GIALLA");
    } else {
        // In caso di parità, effettua una seconda serie di letture con più potenza
        if (conteggioVerde == conteggioGiallo && conteggioVerde > 0) {
            SensoreOttico.setLightPower(POTENZA_LED_BASSA_LUMINOSITA, percent);
            conteggioVerde = 0;
            conteggioGiallo = 0;
            
            for(int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
                if (SensoreOttico.isNearObject()) {
                    int hue = SensoreOttico.hue();
                    
                    // Range più precisi per il secondo tentativo
                    if (hue >= 30 && hue < 70) {
                        conteggioGiallo++;
                    } else if (hue >= 110 && hue < 150) {
                        conteggioVerde++;
                    }
                }
                this_thread::sleep_for(milliseconds(RITARDO_LETTURA_COLORE));
            }
            
            if(conteggioVerde > conteggioGiallo) {
                Brain.Screen.print("Verde (2° lettura: %d)", conteggioVerde);
                verde = true;
                giallo = false;
                officinaVerde = true;
                Brain.Screen.newLine();
                Brain.Screen.print("Officina disponibile: VERDE");
            } else if(conteggioGiallo > conteggioVerde) {
                Brain.Screen.print("Giallo (2° lettura: %d)", conteggioGiallo);
                giallo = true;
                verde = false;
                officinaVerde = false;
                Brain.Screen.newLine();
                Brain.Screen.print("Officina disponibile: GIALLA");
            } else {
                Brain.Screen.print("Colore non determinato");
                // Default a giallo se non determinato
                giallo = true;
                verde = false;
                officinaVerde = false;
            }
        } else {
            Brain.Screen.print("Colore non determinato");
            // Default a giallo se non determinato
            giallo = true;
            verde = false;
            officinaVerde = false;
        }
    }
    
    // Spegni il LED dopo l'uso
    SensoreOttico.setLight(ledState::off);
}

/**
 * Muove il robot nella direzione specificata per la distanza data
 * @param direzione 'f' per avanti, 'b' per indietro
 * @param distanza Distanza in millimetri
 */
void move(char direzione, int distanza) {
    float fattoreCorrezione = (direzione == 'f') ? 
                             FATTORE_CORREZIONE_AVANTI : 
                             FATTORE_CORREZIONE_INDIETRO;
    float distanzaCorretta = distanza * fattoreCorrezione;

    if(direzione == 'f') {
        Smartdrive.driveFor(forward, distanzaCorretta, mm, VELOCITA_AVANZAMENTO, rpm);
        distanzaTotale += distanza; // Aggiorna contatore distanza
        
        // Aggiorna la posizione stimata (simulata)
        double angoloRadiani = Inertial.heading(degrees) * 3.14159 / 180.0;
        posizioneX += static_cast<int>(distanza * cos(angoloRadiani));
        posizioneY += static_cast<int>(distanza * sin(angoloRadiani));
    } else if (direzione == 'b') {
        Smartdrive.driveFor(reverse, distanzaCorretta, mm, VELOCITA_AVANZAMENTO, rpm);
        
        // Aggiorna la posizione stimata (simulata)
        double angoloRadiani = Inertial.heading(degrees) * 3.14159 / 180.0;
        posizioneX -= static_cast<int>(distanza * cos(angoloRadiani));
        posizioneY -= static_cast<int>(distanza * sin(angoloRadiani));
    }
    
    // Attendi fino a quando il movimento è completato
    while(Smartdrive.isMoving()) {
        // Controlla per ostacoli durante il movimento in avanti
        if(direzione == 'f' && osFront) {
            Smartdrive.stop(hold);
            Brain.Screen.print("Ostacolo rilevato!");
            break;
        }
        this_thread::sleep_for(milliseconds(10));
    }
}

/**
 * Ruota il robot verso l'angolo assoluto specificato
 * @param angolo Angolo assoluto in gradi
 */
void turn(int angolo) {
    Smartdrive.setTurnVelocity(VELOCITA_ROTAZIONE, percent);
    Smartdrive.turnToHeading(angolo, degrees);
    
    // Attendi fino a quando la rotazione è completata
    while(Smartdrive.isMoving()) {
        this_thread::sleep_for(milliseconds(10));
    }
}

/**
 * Controlla il braccio e la pinza
 * @param azione 'u' alza braccio, 'd' abbassa braccio, 'o' apri pinza, 'c' chiudi pinza
 */
void controllaRobotBraccio(char azione) {
    int velocitaMotore = VELOCITA_BRACCIO;
    double posizione = 0.0;
    motor* motoreTarget = nullptr;
    
    switch(azione) {
        case 'u':
            braccio.spinToPosition(BRACCIO_ALZATO, rotationUnits::deg, (VELOCITA_BRACCIO - 10), velocityUnits::pct);
            braccio.stop(hold);
            break;
        case 'd':
            braccio.spinToPosition(BRACCIO_ABBASSATO, rotationUnits::deg, VELOCITA_BRACCIO, velocityUnits::pct);
            braccio.stop(hold);
            break;
        case 'o':
            pinza.spinToPosition(PINZA_APERTA, degrees, (VELOCITA_PINZA + 10), velocityUnits::pct, true);
            pinza.stop(hold);
            break;
        case 'c':
            pinza.spinToPosition(PINZA_CHIUSA, degrees, VELOCITA_PINZA, velocityUnits::pct, false);
            this_thread::sleep_for(milliseconds(1000));
            pinza.stop(hold);
            break;
        default:
            break;
    }
    
    if (motoreTarget != nullptr) {
        motoreTarget->spinToPosition(posizione, rotationUnits::deg, velocitaMotore, velocityUnits::pct, false);
        motoreTarget->stop(hold);
    }
}

/**
 * Sequenza per prendere un oggetto
 */
void prendi() {
    controllaRobotBraccio('o'); // Apri pinza
    controllaRobotBraccio('d'); // Abbassa braccio
    controllaRobotBraccio('c'); // Chiudi pinza
    controllaRobotBraccio('u'); // Alza braccio
}

/**
 * Sequenza per lasciare un oggetto
 */
void lascia() {
    controllaRobotBraccio('d'); // Abbassa braccio
    controllaRobotBraccio('o'); // Apri pinza
    controllaRobotBraccio('u'); // Alza braccio
    controllaRobotBraccio('c'); // Chiudi pinza
}

/**
 * Verifica se un colore è affidabile in base alla luminosità
 * @param brightness Valore di luminosità dal sensore (0-100)
 * @return true se il colore è affidabile, false altrimenti
 */
bool coloreAffidabile(double brightness) {
    // Con luminosità molto bassa, i colori sono difficili da rilevare correttamente
    const double SOGLIA_LUMINOSITA_MINIMA = 10.0;
    
    // Se la luminosità è troppo bassa, il colore non è affidabile
    if (brightness < SOGLIA_LUMINOSITA_MINIMA) {
        return false;
    }
    
    return true;
}

/**
 * Riconosce il colore considerando Hue e Luminosità
 * @param hue Valore dell'hue dal sensore (0-359)
 * @param brightness Valore di luminosità dal sensore (0-100)
 * @return Carattere che rappresenta il colore: 'r' rosso, 'g' giallo, 'v' verde, 'b' blu, 'n' non determinato
 */
char riconosciColoreHV(int hue, double brightness) {
    // Se il colore non è affidabile, restituisci 'n' (nessun colore)
    if (!coloreAffidabile(brightness)) {
        return 'n';
    }
    
    // Classifica il colore in base all'hue con range migliorati
    if (hue < 15 || hue >= 330) { // rosso
        return 'r';
    } else if (hue >= 30 && hue < 60) { // giallo - range ristretto
        return 'g';
    } else if (hue >= 60 && hue < 150) { // verde - range specifico
        return 'v';
    } else if (hue >= 190 && hue < 250) { // blu
        return 'b';
    }
    
    // Se l'hue non rientra in nessun range definito
    return 'n';
}

/**
 * Legge il colore con il sensore frontale
 * @return Colore rilevato: 'r' rosso, 'g' giallo, 'v' verde, 'b' blu, 'n' non determinato
 */
char leggiFront() {
    SensoreOttico2.setLightPower(POTENZA_LED_NORMALE, percent);
    SensoreOttico2.setLight(ledState::on);

    int conteggiRosso = 0;
    int conteggioGiallo = 0;
    int conteggioVerde = 0;
    int conteggioBlu = 0;
    int conteggioNonDeterminato = 0;

    // Array per memorizzare i valori di ogni campione (per debug)
    int valoriHue[NUMERO_CAMPIONI_COLORE] = {0};
    double valoriBri[NUMERO_CAMPIONI_COLORE] = {0};

    for (int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
        if (SensoreOttico2.isNearObject()) {
            int hue = SensoreOttico2.hue(); 
            double brightness = SensoreOttico2.brightness();
            
            // Salva i valori per debug
            valoriHue[i] = hue;
            valoriBri[i] = brightness;
            
            // Aumenta la potenza del LED se la luminosità è bassa
            if (brightness < SOGLIA_BASSA_LUMINOSITA) {
                SensoreOttico2.setLightPower(POTENZA_LED_BASSA_LUMINOSITA, percent);
            }

            // Usa la nuova funzione di riconoscimento colore
            char colore = riconosciColoreHV(hue, brightness);
            
            // Aggiorna i conteggi in base al risultato
            switch(colore) {
                case 'r': conteggiRosso++; break;
                case 'g': conteggioGiallo++; break;
                case 'v': conteggioVerde++; break;
                case 'b': conteggioBlu++; break;
                case 'n': conteggioNonDeterminato++; break;
            }
        }
        // Aggiungi un breve delay per stabilizzare la lettura
        this_thread::sleep_for(milliseconds(50));
        this_thread::sleep_for(milliseconds(RITARDO_LETTURA_COLORE));
    }

    // Visualizza i risultati
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    // Visualizza i valori HV dei campioni (debug)
    Brain.Screen.print("HV: ");
    for (int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
        if (i < 2) { // Mostra solo i primi 2 per limitare lo spazio
            Brain.Screen.print("H:%d,B:%.1f ", valoriHue[i], valoriBri[i]);
        }
    }
    Brain.Screen.newLine();
    
    // Visualizza i conteggi di rilevamento
    Brain.Screen.print("R:%d G:%d V:%d B:%d N:%d", conteggiRosso, conteggioGiallo, conteggioVerde, conteggioBlu, conteggioNonDeterminato);
    Brain.Screen.newLine();

    char coloreRilevato = 'n'; // n = non determinato
    int maxConteggio = 0;
    
    // Trova il colore con il massimo conteggio
    if (conteggiRosso > maxConteggio) {
        maxConteggio = conteggiRosso;
        coloreRilevato = 'r';
    }
    if (conteggioGiallo > maxConteggio) {
        maxConteggio = conteggioGiallo;
        coloreRilevato = 'g';
    }
    if (conteggioVerde > maxConteggio) {
        maxConteggio = conteggioVerde;
        coloreRilevato = 'v';
    }
    if (conteggioBlu > maxConteggio) {
        maxConteggio = conteggioBlu;
        coloreRilevato = 'b';
    }
    
    // Controlla che ci sia abbastanza "certezza" nel risultato
    if (maxConteggio < NUMERO_CAMPIONI_COLORE / 3) {
        coloreRilevato = 'n'; // Non abbastanza certezza
        Brain.Screen.print("Colore incerto (conteggio basso)");
    } else {
        // Mostra il colore rilevato
        switch(coloreRilevato) {
            case 'r': Brain.Screen.print("Rosso"); break;
            case 'g': Brain.Screen.print("Giallo"); break;
            case 'v': Brain.Screen.print("Verde"); break;
            case 'b': Brain.Screen.print("Blu"); break;
            default: Brain.Screen.print("Non determinato"); break;
        }
    }

    // Spegni il LED dopo l'uso
    SensoreOttico2.setLight(ledState::off);
    return coloreRilevato;
}

/**
 * Percorso verso la colonnina di ricarica 1 (per auto rosse e gialle)
 * Aggiornato in base alla TAVOLA 1
 */
void percorsoColonnina1() {
    // Percorso verso colonnina 1
    move('b', 120);
    turn(270);
    move('f', 65); // Aggiornato in base alla TAVOLA 1
    lascia();
    
    // Ritorno
    move('b', 55);
    turn(90);
    move('f', 75);
    turn(0);
    move('f', 120);
}

/**
 * Percorso verso la colonnina di ricarica 2 (per auto rosse e gialle quando colonnina 1 è piena)
 * Aggiornato in base alla TAVOLA 1
 */
void percorsoColonnina2() {
    // Percorso verso colonnina 2
    move('b', 120);
    turn(270);
    move('f', 135); // Distanza maggiore per raggiungere la colonnina 2
    lascia();
    
    // Ritorno
    move('b', 55);
    turn(90);
    move('f', 145);
    turn(0);
    move('f', 120);
}

/**
 * Percorso verso la colonnina di ricarica 3 (per auto rosse e gialle quando colonnina 2 è piena)
 * Aggiunto in base alla TAVOLA 1 che mostra 3 colonnine
 */
void percorsoColonnina3() {
    // Percorso verso colonnina 3
    move('b', 120);
    turn(270);
    move('f', 205); // Distanza per raggiungere la colonnina 3
    lascia();
    
    // Ritorno
    move('b', 55);
    turn(90);
    move('f', 215);
    turn(0);
    move('f', 120);
}

/**
 * Percorso verso l'officina gialla per le auto blu
 * Aggiornato in base alla TAVOLA 1
 */
void percorsoOfficinaGialla() {
    // Percorso verso officina gialla - coordinati aggiornati
    move('b', 100);
    turn(180);
    move('f', 220); // Adattato in base ai dati della TAVOLA 1
    lascia();
    
    // Ritorno
    move('b', 220);
    turn(0);
    move('f', 100);
}

/**
 * Percorso verso l'officina verde per le auto blu
 * Aggiornato in base alla TAVOLA 1
 */
void percorsoOfficinaVerde() {
    // Percorso verso officina verde - coordinati aggiornati
    move('b', 100);
    turn(0);
    move('f', 220); // Adattato in base ai dati della TAVOLA 1
    lascia();
    
    // Ritorno
    move('b', 220);
    turn(180);
    move('f', 100);
}

/**
 * Funzione per saltare le auto verdi
 * Aggiornato in base alla TAVOLA 2 per la disposizione degli oggetti
 */
void saltaAutoVerde() {
    // Movimento ottimizzato per saltare un'auto verde in base alla TAVOLA 2
    move('b', 40);
    turn(0);
    move('f', 100);
    turn(90);
    move('f', 45); // Distanza aggiornata in base alla TAVOLA 2
}

/**
 * Navigazione iniziale verso l'area di stallo
 * Aggiornato in base alla TAVOLA 1 e 2
 */
void navigaAreaStallo() {
    // Naviga dall'area di partenza all'area di stallo (blu)
    move('f', 350);
    turn(90);
    move('f', 170);
    turn(0);
    move('f', 380);
    turn(90);
    move('f', 75);
}




void colori() {
    for (int i = 0; i < 18; i++) { // 18 auto totali nel parcheggio
        // Rileva il colore dell'oggetto
        char colore = leggiFront();
        
        // Visualizza il colore rilevato e implementa percorsi diversi
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Auto %d: ", i+1);
        
        switch(colore) {
            case 'r': // Se colore rosso
                Brain.Screen.print("ROSSA");
                Brain.Screen.newLine();
                
                // Verifica se le colonnine sono piene prima di procedere
                if (autoInColonnina1 < 5) {
                    Brain.Screen.print("Trasporto a Colonnina 1");
                    prendi();
                    percorsoColonnina1();
                    autoInColonnina1++;
                    autoRosseRaccolte++;
                } else if (autoInColonnina2 < 5) {
                    Brain.Screen.print("Trasporto a Colonnina 2");
                    prendi();
                    percorsoColonnina2();
                    autoInColonnina2++;
                    autoRosseRaccolte++;
                } else if (autoInColonnina3 < 5) {
                    Brain.Screen.print("Trasporto a Colonnina 3");
                    prendi();
                    percorsoColonnina3();
                    autoInColonnina3++;
                    autoRosseRaccolte++;
                } else {
                    Brain.Screen.print("Tutte le colonnine piene, salto auto");
                    saltaAutoVerde(); // Usa la funzione di salto per evitare l'auto
                }
                break;
                
            case 'g': // Se colore giallo
                Brain.Screen.print("GIALLA");
                Brain.Screen.newLine();
                
                // Verifica se le colonnine sono piene prima di procedere
                if (autoInColonnina1 < 5) {
                    Brain.Screen.print("Trasporto a Colonnina 1");
                    prendi();
                    percorsoColonnina1();
                    autoInColonnina1++;
                    autoGialleRaccolte++;
                } else if (autoInColonnina2 < 5) {
                    Brain.Screen.print("Trasporto a Colonnina 2");
                    prendi();
                    percorsoColonnina2();
                    autoInColonnina2++;
                    autoGialleRaccolte++;
                } else if (autoInColonnina3 < 5) {
                    Brain.Screen.print("Trasporto a Colonnina 3");
                    prendi();
                    percorsoColonnina3();
                    autoInColonnina3++;
                    autoGialleRaccolte++;
                } else {
                    Brain.Screen.print("Tutte le colonnine piene, salto auto");
                    saltaAutoVerde(); // Usa la funzione di salto per evitare l'auto
                }
                break;
                
            case 'b': // Se colore blu
                Brain.Screen.print("BLU");
                Brain.Screen.newLine();
                
                // Trasporta all'officina determinata all'inizio (verde o gialla)
                prendi();
                if (officinaVerde) {
                    Brain.Screen.print("Trasporto a Officina VERDE");
                    percorsoOfficinaVerde();
                } else {
                    Brain.Screen.print("Trasporto a Officina GIALLA");
                    percorsoOfficinaGialla();
                }
                autoBluRaccolte++;
                break;
                
            case 'v': // Se colore verde
                Brain.Screen.print("VERDE - Salto");
                Brain.Screen.newLine();
                // Le auto verdi sono da ignorare
                saltaAutoVerde();
                break;
                
            default: // Se colore non determinato
                Brain.Screen.print("NON DETERMINATO");
                Brain.Screen.newLine();
                
                // Tentativo di nuova lettura con maggiore potenza LED
                SensoreOttico2.setLightPower(POTENZA_LED_BASSA_LUMINOSITA, percent);
                colore = leggiFront();
                
                // Se ancora non determinato, assumiamo sia un'auto da saltare
                if (colore == 'n') {
                    Brain.Screen.print("Ancora non determinato - Salto");
                    saltaAutoVerde();
                } else {
                    // Torna all'inizio del ciclo per processare questo nuovo colore
                    i--; // Decrementa i per ripetere il ciclo con lo stesso indice
                }
                break;
        }
        
        // Stampa statistiche dopo ogni auto
        Brain.Screen.newLine();
        Brain.Screen.print("Stat: R:%d G:%d B:%d", autoRosseRaccolte, autoGialleRaccolte, autoBluRaccolte);
        Brain.Screen.newLine();
        Brain.Screen.print("Col: C1:%d C2:%d C3:%d", autoInColonnina1, autoInColonnina2, autoInColonnina3);
        
        // Breve pausa tra le auto
        this_thread::sleep_for(milliseconds(1000));
    }
    
    // Al termine di tutte le auto, torna all'area di partenza
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Completato! Ritorno all'area di partenza");
    
    // Calcola percorso di ritorno all'area di partenza
    // Questo è un esempio semplificato, andrebbe adattato in base alla posizione finale
    turn(180);
    move('f', 200);
    turn(270);
    move('f', 200);
    
    // Statistiche finali
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("GARA COMPLETATA!");
    Brain.Screen.newLine();
    Brain.Screen.print("Auto rosse: %d", autoRosseRaccolte);
    Brain.Screen.newLine();
    Brain.Screen.print("Auto gialle: %d", autoGialleRaccolte);
    Brain.Screen.newLine();
    Brain.Screen.print("Auto blu: %d", autoBluRaccolte);
    Brain.Screen.newLine();
    Brain.Screen.print("Totale colonnine: %d", autoInColonnina1 + autoInColonnina2 + autoInColonnina3);
}

/**
 * Funzione per inizializzazione del robot
 */
void inizializzaRobot() {
    // Inizializza il sensore inerziale
    Inertial.calibrate();
    // Attendi che la calibrazione sia completa
    while(Inertial.isCalibrating()) {
        this_thread::sleep_for(milliseconds(100));
    }
    
    // Inizializza le posizioni del braccio e della pinza
    braccio.setPosition(0, degrees);
    pinza.setPosition(0, degrees);
    
    // Posizione iniziale di partenza
    controllaRobotBraccio('u'); // Alza braccio
    controllaRobotBraccio('c'); // Chiudi pinza
    
    // Reset dei contatori
    autoRosseRaccolte = 0;
    autoGialleRaccolte = 0;
    autoBluRaccolte = 0;
    autoInColonnina1 = 0;
    autoInColonnina2 = 0;
    autoInColonnina3 = 0;
    
    // Inizializza posizione
    posizioneX = 0;
    posizioneY = 0;
    
    // Visualizza messaggio di pronto
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Robot pronto!");
    Brain.Screen.newLine();
    Brain.Screen.print("Premere un tasto per iniziare");
    
    // Attendi input utente per iniziare
    while(!Brain.Screen.pressing()) {
        this_thread::sleep_for(milliseconds(100));
    }
    Brain.Screen.clearScreen();
}

/**
 * Entry point del programma
 */
int main() {
    // Configura il controller del cervello
    Brain.Screen.setFont(mono12);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Avvio robot...");
    
    // Inizializza thread per rilevamento ostacoli
    thread checkFrontThread(checkFront);
    
    // Inizializza il robot
    inizializzaRobot();
    
    // Esegui la funzione di inizio per determinare il colore dell'area
    inizio();
    
    // Naviga fino all'area di stallo
    navigaAreaStallo();
    
    // Esegui la funzione di gestione colori per elaborare tutte le auto
    colori();
    
    // Termina il thread di rilevamento ostacoli
    threadAttivo = false;
    checkFrontThread.join();
    
    // Visualizza messaggio di completamento
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Programma completato!");
    
    return 0;
}