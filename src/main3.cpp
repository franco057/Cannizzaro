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
const double PINZA_CHIUSA = 60.5;
const double BRACCIO_ALZATO = 35.5;
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
            double saturation = SensoreOttico.saturation(); // Aggiunto il valore di saturazione
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
    } else if(conteggioGiallo > conteggioVerde) {
        giallo = true;
        verde = false;
        Brain.Screen.print("Giallo (Conteggio: %d)", conteggioGiallo);
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
            } else if(conteggioGiallo > conteggioVerde) {
                Brain.Screen.print("Giallo (2° lettura: %d)", conteggioGiallo);
                giallo = true;
                verde = false;
            } else {
                Brain.Screen.print("Colore non determinato");
            }
        } else {
            Brain.Screen.print("Colore non determinato");
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
    bool coloreAffidabile(double saturation, double brightness) {
        // I colori con bassa saturazione sono spesso grigio o bianco, non colori veri
        const double SOGLIA_SATURAZIONE_MINIMA = 20.0;
        // Con luminosità molto bassa, i colori sono difficili da rilevare correttamente
        const double SOGLIA_LUMINOSITA_MINIMA = 10.0;
        
        // Saturazione minima richiesta varia in base alla luminosità
        // Con luminosità molto alta o molto bassa, richiediamo più saturazione
        double sogliaEffettiva = SOGLIA_SATURAZIONE_MINIMA;
        
        // Richiedi più saturazione quando la luminosità è bassa o molto alta
        if (brightness < 30.0 || brightness > 90.0) {
            sogliaEffettiva = SOGLIA_SATURAZIONE_MINIMA * 1.5;
        }
        
        // Se sia la luminosità che la saturazione sono troppo basse, il colore non è affidabile
        if (brightness < SOGLIA_LUMINOSITA_MINIMA) {
            return false;
        }
        
        return saturation >= sogliaEffettiva;
    }
    
    /**
     * Riconosce il colore considerando Hue, Saturazione e Luminosità
     * @param hue Valore dell'hue dal sensore (0-359)
     * @param saturation Valore di saturazione dal sensore (0-100)
     * @param brightness Valore di luminosità dal sensore (0-100)
     * @return Carattere che rappresenta il colore: 'r' rosso, 'g' giallo, 'v' verde, 'b' blu, 'n' non determinato
     */
    char riconosciColoreHSV(int hue, double saturation, double brightness) {
        // Se il colore non è affidabile, restituisci 'n' (nessun colore)
        if (!coloreAffidabile(saturation, brightness)) {
            return 'n';
        }
        
        // Classifica il colore in base all'hue con range migliorati
        if (hue < 15 || hue >= 330) { // rosso
            return 'r';
        } else if (hue >= 30 && hue < 70) { // giallo - range ristretto
            return 'g';
        } else if (hue > 72 && hue < 150) { // verde - range specifico
            return 'v';
        } else if (hue >= 190 && hue < 250) { // blu
            return 'b';
        }
        
        // Se l'hue non rientra in nessun range definito
        return 'n';
    }
    
    /**
     * Versione migliorata della funzione leggiFront che utilizza HSV
     * @return Colore rilevato: 'r' rosso, 'g' giallo, 'v' verde, 'b' blu, 'n' non determinato
     */
    char leggiFrontHSV() {
        SensoreOttico2.setLightPower(POTENZA_LED_NORMALE, percent);
        SensoreOttico2.setLight(ledState::on);
    
        int conteggiRosso = 0;
        int conteggioGiallo = 0;
        int conteggioVerde = 0;
        int conteggioBlu = 0;
        int conteggioNonDeterminato = 0;
    
        // Array per memorizzare i valori di ogni campione (per debug)
        int valoriHue[NUMERO_CAMPIONI_COLORE] = {0};
        double valoriSat[NUMERO_CAMPIONI_COLORE] = {0};
        double valoriBri[NUMERO_CAMPIONI_COLORE] = {0};
    
        for (int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
            if (SensoreOttico2.isNearObject()) {
                int hue = SensoreOttico2.hue(); 
                double brightness = SensoreOttico2.brightness();
                double saturation = SensoreOttico2.saturation();
                
                // Salva i valori per debug
                valoriHue[i] = hue;
                valoriSat[i] = saturation;
                valoriBri[i] = brightness;
                
                // Aumenta la potenza del LED se la luminosità è bassa
                if (brightness < SOGLIA_BASSA_LUMINOSITA) {
                    SensoreOttico2.setLightPower(POTENZA_LED_BASSA_LUMINOSITA, percent);
                }
    
                // Usa la nuova funzione di riconoscimento colore
                char colore = riconosciColoreHSV(hue, saturation, brightness);
                
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
    
        // Visualizza i valori HSV dei campioni (debug)
        Brain.Screen.print("HSV: ");
        for (int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
            if (i < 2) { // Mostra solo i primi 2 per limitare lo spazio
                Brain.Screen.print("H:%d,S:%.1f,B:%.1f ", valoriHue[i], valoriSat[i], valoriBri[i]);
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
}

/**
 * Funzione che implementa percorsi diversi in base al colore rilevato
 * Gestisce percorsi specifici per rosso e giallo
 */
void colori() {
    for (int i = 0; i < 17; i++)
    {
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
            move('b', 115);
            turn(270);
            move('f', 70);
            lascia();
            
            // Ritorno
            move('b', 50);
            turn(90);
            move('f', 80);
            turn(0);
            move('f', 115);



        } 
        else if (colore == 'g') { // Se colore giallo
            Brain.Screen.print("GIALLO");
            Brain.Screen.newLine();
            Brain.Screen.print("Eseguendo percorso GIALLO");
            
            // Percorso specifico per oggetto giallo

        }
        else if (colore=='v') {
            Brain.Screen.print("VERDE ");
            Brain.Screen.newLine();
            Brain.Screen.print("NON TOCCARE");
            move('b', 40);
            turn(0);
            move('f', 100);
            turn(90);
            move('f', 40);
        }
        else {
            turn(87);
        }
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
    move('f', 75);

    
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