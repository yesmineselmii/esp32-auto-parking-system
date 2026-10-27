// ---------------- PINS MOTEURS (Pont-H / L298N) ----------------
const int IN1 = 25;
const int IN2 = 26;
const int IN3 = 27;
const int IN4 = 14;

// ---------------- PINS CAPTEURS ULTRASONS ----------------
// Capteur Lateral Gauche
const int trigLateralG = 23;
const int echoLateralG = 22;

// Capteur Arriere
const int trigArriere = 18;
const int echoArriere = 5;

// ---------------- SEUILS DE DISTANCE (en cm) ----------------
const int SEUIL_PLACE = 30;     // Espace détecté à gauche (> 30 cm)
const int SEUIL_ARRIERE = 10;   // Arrêt en marche arrière (<= 10 cm)

// ---------------- PARAMÈTRES DE TEMPS ----------------
const unsigned long TEMPS_ROTATION = 600;        // Durée de la rotation à droite (ms)
const unsigned long TEMPS_MAX_RECHERCHE = 20000;  // Limite de recherche de place (20s)
const unsigned long TEMPS_MAX_RECUL = 10000;     // Limite de marche arrière (10s)
const int NB_DETECTIONS_PLACE = 3;               // Validation de la place

// Prototypes de fonctions
long lireDistance(int trigPin, int echoPin);
void avancerLentement();
void reculerLentement();
void tournerDroiteLentement();
void arreter();
void afficherDistance(const char* nom, long distance);

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);

  // Configuration des broches moteurs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configuration des broches capteurs (Gauche et Arriere)
  pinMode(trigLateralG, OUTPUT);
  pinMode(echoLateralG, INPUT);

  pinMode(trigArriere, OUTPUT);
  pinMode(echoArriere, INPUT);

  digitalWrite(trigLateralG, LOW);
  digitalWrite(trigArriere, LOW);

  arreter();

  Serial.println("===============================================");
  Serial.println("       ROBOT DE PARKING AUTONOME READY         ");
  Serial.println("===============================================");
  Serial.println("[SYSTEME] Initialisation terminee.");
  Serial.println("[SYSTEME] Demarrage dans 3 secondes...");
  delay(3000);
}

// ---------------- LOOP ----------------

void loop() {
  // ==========================================================
  // 1. RECHERCHE DE PLACE DE PARKING (GAUCHE)
  // ==========================================================
  Serial.println("\n-----------------------------------------------");
  Serial.println("[ETAPE 1] Mouvement : AVANCER LENTEMENT");
  Serial.println("[ETAPE 1] Objectif  : Obtenir " + String(NB_DETECTIONS_PLACE) + " lectures > " + String(SEUIL_PLACE) + " cm a gauche");
  Serial.println("-----------------------------------------------");
  
  unsigned long debutRecherche = millis();
  int detectionsPlace = 0;
  bool placeTrouvee = false;

  while (millis() - debutRecherche < TEMPS_MAX_RECHERCHE) {
    long dGauche = lireDistance(trigLateralG, echoLateralG);

    // Affichage de la distance pour suivi
    afficherDistance("Capteur GAUCHE", dGauche);

    // Confirmation de la présence d'une place (> 30 cm)
    if (dGauche > SEUIL_PLACE && dGauche != 999) {
      detectionsPlace++;
      Serial.println(" -> [DETECTION PLACE] Validation " + String(detectionsPlace) + "/" + String(NB_DETECTIONS_PLACE) + " (Espace libéré : " + String(dGauche) + " cm)");
    } else {
      if (detectionsPlace > 0) {
        Serial.println(" -> [DETECTION PLACE] Reinitialisation : Obstacle redetecte.");
      }
      detectionsPlace = 0;
    }

    if (detectionsPlace >= NB_DETECTIONS_PLACE) {
      placeTrouvee = true;
      Serial.println("\n[SUCCES ETAPE 1] Place de parking validee !");
      break;
    }

    // Avancer d'un pas lent
    avancerLentement();
  }

  arreter();

  if (!placeTrouvee) {
    Serial.println("\n[ECHEC ETAPE 1] Temps limite de recherche atteint (" + String(TEMPS_MAX_RECHERCHE/1000) + "s).");
    Serial.println("[MOUVEMENT] ARRET DEFINITIF.");
    while (1);
  }

  Serial.println("[MOUVEMENT] Pause de 1 seconde avant la manœuvre de rotation...");
  delay(1000);

  // ==========================================================
  // 2. ROTATION À DROITE
  // ==========================================================
  Serial.println("\n-----------------------------------------------");
  Serial.println("[ETAPE 2] Mouvement : ROTATION A DROITE (LENTEMENT)");
  Serial.println("[ETAPE 2] Duree estimee : " + String(TEMPS_ROTATION) + " ms");
  Serial.println("-----------------------------------------------");
  
  unsigned long debutRotation = millis();
  while (millis() - debutRotation < TEMPS_ROTATION) {
    tournerDroiteLentement();
  }
  
  arreter();
  Serial.println("[SUCCES ETAPE 2] Rotation terminee.");
  Serial.println("[MOUVEMENT] Pause de 1 seconde avant la marche arriere...");
  delay(1000);

  // ==========================================================
  // 3. MARCHE ARRIÈRE DANS LA PLACE
  // ==========================================================
  Serial.println("\n-----------------------------------------------");
  Serial.println("[ETAPE 3] Mouvement : RECULER LENTEMENT");
  Serial.println("[ETAPE 3] Objectif  : Reculer jusqu'a " + String(SEUIL_ARRIERE) + " cm de l'obstacle arriere");
  Serial.println("-----------------------------------------------");
  
  unsigned long debutRecul = millis();

  while (millis() - debutRecul < TEMPS_MAX_RECUL) {
    long dArriere = lireDistance(trigArriere, echoArriere);
    afficherDistance("Capteur ARRIERE", dArriere);

    // Sécurité si la lecture échoue
    if (dArriere == 999) {
      arreter();
      Serial.println(" -> [AVERTISSEMENT] Mesure arriere invalide (hors portee). Pause temporaire.");
      delay(200);
      continue;
    }

    // Condition d'arrêt final (10 cm de l'obstacle)
    if (dArriere <= SEUIL_ARRIERE) {
      arreter();
      Serial.println("\n===============================================");
      Serial.println("    PARKING REUSSI ! DISTANCE FINAL = " + String(dArriere) + " cm");
      Serial.println("===============================================");
      Serial.println("[MOUVEMENT] ARRET FINAL DU ROBOT.");
      while (1); // Fin du programme
    }

    // Reculer d'un pas lent
    reculerLentement();
  }

  arreter();
  Serial.println("\n[ECHEC ETAPE 3] Temps maximal de recul atteint.");
  Serial.println("[MOUVEMENT] ARRET DEFINITIF.");
  while (1); // Fin du programme
}

// ---------------- MESURE HC-SR04 ----------------

long lireDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duree = pulseIn(echoPin, HIGH, 25000); // Timeout ~4 mètres

  if (duree == 0) {
    return 999; // Code erreur "pas d'écho"
  }

  return (long)(duree * 0.0343 / 2.0);
}

void afficherDistance(const char* nom, long distance) {
  Serial.print("[MESURE] ");
  Serial.print(nom);
  Serial.print(" : ");
  if (distance == 999) {
    Serial.println("Hors portee / Aucun echo");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }
}

// ---------------- COMMANDES MOTEURS (AVEC LOGS ET LENTEUR) ----------------

// Avance par petites impulsions (30ms activé / 40ms désactivé)
void avancerLentement() {
  Serial.println("   [MOTEURS] IMPULSION : Avancer (30ms ON / 40ms OFF)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(30);
  
  arreter();
  delay(40);
}

// Recule par petites impulsions (30ms activé / 40ms désactivé)
void reculerLentement() {
  Serial.println("   [MOTEURS] IMPULSION : Reculer (30ms ON / 40ms OFF)");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(30);
  
  arreter();
  delay(40);
}

// Tourne à droite par petites impulsions (25ms activé / 30ms désactivé)
void tournerDroiteLentement() {
  Serial.println("   [MOTEURS] IMPULSION : Tourner a Droite (25ms ON / 30ms OFF)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(25);
  
  arreter();
  delay(30);
}

void arreter() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}