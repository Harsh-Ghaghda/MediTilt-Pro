// --- UI COLOR SETTINGS ---
int hoverColor = 1055;     // Custom Blue highlight
int defaultColor = 65535;  // Default normal button color
// -------------------------

// State Tracking for Navigation
int currentPage = 0;   
int currentIndex = 0;  

// Exact button IDs from your Nextion Layout
String page0_btns[] = {"b1", "b6", "b2", "b0"}; 
String page1_btns[] = {"b1", "b2", "b3", "b4", "b0"}; 
String page2_btns[] = {"b9", "b10", "b0"}; 

const int page0_size = 4;
const int page1_size = 5;
const int page2_size = 3;

// Forward declaration of functions
void endNextionCmd();
void clearAllButtonsOnPage();
void highlightCurrentButton(int colorCode);

void setup() {
  // Serial to PC (Python connects here)
  Serial.begin(115200); 
  
  // Serial to Nextion Display
  Serial2.begin(9600, SERIAL_8N1, 16, 17); 

  delay(2000); 

  // Force wipe the screen to clear any stuck buttons, then highlight the first
  clearAllButtonsOnPage();
  highlightCurrentButton(hoverColor); 
}

void loop() {
  // 1. CHECK FOR BLINKS FROM PYTHON SCRIPT (Over USB)
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    
    if (incomingChar == '0') {
      handleNext();
    } 
    else if (incomingChar == '1') {
      handleSelect();
    }
  }

  // 2. CHECK FOR INCOMING BED COMMANDS FROM NEXTION
  if (Serial2.available() > 0) {
    String incomingCmd = Serial2.readStringUntil('\n');
    incomingCmd.trim(); 

    if (incomingCmd.length() > 0) {
      interpretBedCommand(incomingCmd);
    }
  }
}

// --------------------------------------------------------
// INCOMING NEXTION COMMAND INTERPRETER
// --------------------------------------------------------
void interpretBedCommand(String cmd) {
  // Prints back to Python for debugging
  Serial.println("\n[HARDWARE COMMAND RECEIVED]: " + cmd);

  if (cmd == "H0")      Serial.println("Moving Torso Actuator to 0 DEG (FLAT)");
  else if (cmd == "H1") Serial.println("Moving Torso Actuator to 15 DEG");
  else if (cmd == "H2") Serial.println("Moving Torso Actuator to 30 DEG");
  else if (cmd == "H3") Serial.println("Moving Torso Actuator to 60 DEG");
  else if (cmd == "L0") Serial.println("Moving Lower Body Actuator to 0 DEG (FLAT)");
  else if (cmd == "L1") Serial.println("Moving Lower Body Actuator to 15 DEG");
  else if (cmd == "L2") Serial.println("Moving Lower Body Actuator to 30 DEG");
  else if (cmd == "L3") Serial.println("Moving Lower Body Actuator to 60 DEG");
  else if (cmd == "T0") Serial.println("Resetting Lateral Tilt to FLAT");
  else if (cmd == "T1") Serial.println("Executing Left Side Lateral Tilt to 15 DEG");
  else if (cmd == "R0") Serial.println("Resetting Lateral Tilt to FLAT");
  else if (cmd == "R1") Serial.println("Executing Right Side Lateral Tilt to 15 DEG");
}

// --------------------------------------------------------
// CORE NAVIGATION HELPER FUNCTIONS
// --------------------------------------------------------
String getCurrentButtonID() {
  if (currentPage == 0) return page0_btns[currentIndex];
  if (currentPage == 1) return page1_btns[currentIndex];
  if (currentPage == 2) return page2_btns[currentIndex];
  return "";
}

void endNextionCmd() {
  Serial2.write(0xFF); Serial2.write(0xFF); Serial2.write(0xFF);
}

void highlightCurrentButton(int colorCode) {
  String target = getCurrentButtonID();
  Serial2.print(target + ".bco=" + String(colorCode));
  endNextionCmd();
}

void clearAllButtonsOnPage() {
  if (currentPage == 0) {
    for (int i = 0; i < page0_size; i++) {
      Serial2.print(page0_btns[i] + ".bco=" + String(defaultColor));
      endNextionCmd();
    }
  } 
  else if (currentPage == 1) {
    for (int i = 0; i < page1_size; i++) {
      Serial2.print(page1_btns[i] + ".bco=" + String(defaultColor));
      endNextionCmd();
    }
  } 
  else if (currentPage == 2) {
    for (int i = 0; i < page2_size; i++) {
      Serial2.print(page2_btns[i] + ".bco=" + String(defaultColor));
      endNextionCmd();
    }
  }
}

void handleNext() {
  // Un-highlight current button
  highlightCurrentButton(defaultColor); 

  // Advance the index
  currentIndex++;
  if (currentPage == 0 && currentIndex >= page0_size) currentIndex = 0;
  if (currentPage == 1 && currentIndex >= page1_size) currentIndex = 0;
  if (currentPage == 2 && currentIndex >= page2_size) currentIndex = 0;

  // Highlight the new button
  highlightCurrentButton(hoverColor); 
}

void handleSelect() {
  String target = getCurrentButtonID();

  // Clear current highlight
  highlightCurrentButton(defaultColor); 

  // Simulate Nextion click
  Serial2.print("click " + target + ",1"); endNextionCmd();
  delay(100); 
  Serial2.print("click " + target + ",0"); endNextionCmd();

  // Track page state
  bool pageChanged = false;
  if (currentPage == 0) {
    if (target == "b1" || target == "b2") { currentPage = 1; pageChanged = true; }
    else if (target == "b0" || target == "b6") { currentPage = 2; pageChanged = true; }
  } 
  else if (currentPage == 1 || currentPage == 2) {
    if (target == "b0") { currentPage = 0; pageChanged = true; }
  }

  // Handle page transitions
  if (pageChanged) {
    delay(250); 
    currentIndex = 0;
    clearAllButtonsOnPage(); // Wipe the new page clean
    highlightCurrentButton(hoverColor);
  } else {
    highlightCurrentButton(hoverColor);
  }
}