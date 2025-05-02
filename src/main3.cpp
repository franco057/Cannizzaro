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
const double PINZA_APERTA = -67.5;
const double PINZA_CHIUSA = 43.5;
const double BRACCIO_ALZATO = -35.5;
const double BRACCIO_ABBASSATO = 120.0;
const int VELOCITA_BRACCIO = 20;
const int VELOCITA_PINZA = 20;

// Costanti per il movimento
const double FATTORE_CORREZIONE_AVANTI = 0.3333;
const double FATTORE_CORREZIONE_INDIETRO = 0.3333;
const double FATTORE_CORREZIONE_ROTAZIONE = 1.0;
const int VELOCITA_ROTAZIONE = 25;
const int VELOCITA_AVANZAMENTO = 35;

// Costanti per i sensori
const int POTENZA_LED_NORMALE = 50;
const int POTENZA_LED_BASSA_LUMINOSITA = 100;
const int SOGLIA_BASSA_LUMINOSITA = 20;
const int NUMERO_CAMPIONI_COLORE = 5;
const int RITARDO_LETTURA_COLORE = 200; // millisecondi

// Variabili globali
bool osFront = false;
bool osLat = false;
char coloreRilevato = 'n'; // 'n' = nessuno, 'r' = rosso, 'g' = giallo, 'v' = verde, 'b' = blu
bool threadAttivo = true;
bool giallo = false;
bool verde = false;
int distanzaTotale = 0; // Contatore distanza totale avanti

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

            // Classifica il colore basato sull'hue
            if (hue >= 30 && hue < 90) {
                conteggioGiallo++;
            } else if (hue >= 90 && hue < 150) {
                conteggioVerde++;
            }
        }
        this_thread::sleep_for(milliseconds(RITARDO_LETTURA_COLORE));
    }

    // Determina il colore finale basato sul conteggio
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    
    if(conteggioVerde > conteggioGiallo) {
        Brain.Screen.print("Verde");
        verde = true;
    } else if(conteggioGiallo > conteggioVerde) {
        giallo = true;
        Brain.Screen.print("Giallo");
    } else {
        Brain.Screen.print("Colore non determinato");
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
    } else if (direzione == 'b') {
        Smartdrive.driveFor(reverse, distanzaCorretta, mm, VELOCITA_AVANZAMENTO, rpm);
    }
    
    // Attendi fino a quando il movimento è completato
    while(Smartdrive.isMoving()) {
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

    for (int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
        if (SensoreOttico2.isNearObject()) {
            int hue = SensoreOttico2.hue(); 
            double brightness = SensoreOttico2.brightness(); 
            
            // Aumenta la potenza del LED se la luminosità è bassa
            if (brightness < SOGLIA_BASSA_LUMINOSITA) {
                SensoreOttico2.setLightPower(POTENZA_LED_BASSA_LUMINOSITA, percent);
            }

            // Classifica il colore in base all'hue
            if (hue < 30 || hue >= 330) { // rosso
                conteggiRosso++;
            } else if (hue >= 30 && hue < 90) { // giallo
                conteggioGiallo++;
            } else if (hue >= 90 && hue < 150) { // verde
                conteggioVerde++;
            } else if (hue >= 180 && hue < 250) { // blu
                conteggioBlu++;
            }
        }
        this_thread::sleep_for(milliseconds(RITARDO_LETTURA_COLORE));
    }

    // Visualizza i risultati
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    char coloreRilevato = 'n'; // n = non determinato
    
    // Determina il colore prevalente
    if (conteggiRosso > conteggioGiallo && conteggiRosso > conteggioVerde && conteggiRosso > conteggioBlu) {
        Brain.Screen.print("Rosso");
        coloreRilevato = 'r';
    } else if (conteggioGiallo > conteggiRosso && conteggioGiallo > conteggioVerde && conteggioGiallo > conteggioBlu) {
        Brain.Screen.print("Giallo");
        coloreRilevato = 'g';
    } else if (conteggioVerde > conteggiRosso && conteggioVerde > conteggioGiallo && conteggioVerde > conteggioBlu) {
        Brain.Screen.print("Verde");
        coloreRilevato = 'v';
    } else if (conteggioBlu > conteggiRosso && conteggioBlu > conteggioGiallo && conteggioBlu > conteggioVerde) {
        Brain.Screen.print("Blu");
        coloreRilevato = 'b';
    } else {
        Brain.Screen.print("Colore incerto");
    }

    // Spegni il LED dopo l'uso
    SensoreOttico2.setLight(ledState::off);
    return coloreRilevato;
}

/**
 * Funzione che implementa percorsi diversi in base al colore rilevato
 * Gestisce percorsi specifici per rosso e giallo
 */
void colori() {
    // Rileva il colore dell'oggetto
    char colore = leggiFront();
    
    // Visualizza il colore rilevato e implementa percorsi diversi
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Colore rilevato: ");
    
    if (colore == 'r') { // Se colore rosso
        Brain.Screen.print("ROSSO");
        Brain.Screen.newLine();
        Brain.Screen.print("Eseguendo percorso ROSSO");
        
        // Percorso specifico per oggetto rosso
        prendi();
        move('b', 100);
        turn(270);
        move('f', 70);
        lascia();
        
        // Ritorno
        move('b', 50);
        turn(90);
        move('f', 80);
        turn(0);
        move('f', 100);



    } 
    else if (colore == 'g') { // Se colore giallo
        Brain.Screen.print("GIALLO");
        Brain.Screen.newLine();
        Brain.Screen.print("Eseguendo percorso GIALLO");
        
        // Percorso specifico per oggetto giallo
        prendi();
        turn(90);
        move('f', 140);
        turn(180);
        move('f', 60);
        lascia();
        
        // Ritorno
        move('b', 60);
        turn(270);
        move('f', 90);
    }
    else if (colore=='v') {
        Brain.Screen.print("VERDE ");
        Brain.Screen.newLine();
        Brain.Screen.print("Eseguendo percorso VERDE");
    }
    else {
        turn(87);
    }
    
    // Visualizza la distanza totale percorsa in avanti
    Brain.Screen.newLine();
    Brain.Screen.print("Distanza totale: %d mm", distanzaTotale);
}

/**
 * Programma principale
 */
int main() {
    // Reset iniziale dei motori
    braccio.resetPosition();
    pinza.resetPosition();

    // Calibrazione del sensore inerziale
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Calibrazione sensore inerziale...");
    
    Inertial.calibrate();
    while(Inertial.isCalibrating()) {
        this_thread::sleep_for(milliseconds(30));
    }
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Calibrazione completata!");
    this_thread::sleep_for(milliseconds(1));

    // Imposta l'orientamento iniziale
    Smartdrive.setHeading(0, degrees);

    // Avvia thread per il monitoraggio frontale
    thread check = thread(checkFront);
    
    // Inizializzazione e lettura colore iniziale
    inizio();

    // Sequenza di movimenti per il percorso
    move('f', 350);
    turn(90);
    move('f', 170);
    turn(0);
    move('f', 380);
    turn(90);
    move('f', 60);
    
    // Esegui la sequenza in base al colore rilevato
    colori();
    
    // Fase finale - torna alla posizione di partenza

    
    // Segnala completamento e mostra distanza totale
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Missione completata!");
    Brain.Screen.newLine();
    Brain.Screen.print("Distanza totale: %d mm", distanzaTotale);
    
    // Termina il thread di monitoraggio
    threadAttivo = false;
    check.join();
    
    return 0;
}