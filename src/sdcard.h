#include <mySD.h>

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
File log_file;

void sdcard_init() {
  Serial.println("-->[MISD] Initializing SD card..");
  // pinMode(SS, OUTPUT);
  if(!card.init(SPI_HALF_SPEED, SD_CS, SD_MOSI, SD_MISO, SD_CLK)) {
    Serial.println("[E][MISD] initialization failed. Things to check:");
    Serial.println("-->[MISD] * is a card is inserted?");
    Serial.println("-->[MISD] * Is your wiring correct?");
    Serial.println("-->[MISD] * did you change the chipSelect pin to match your shield or module?");
  }

  // print the type of card
  Serial.print("-->[MISD] Card type: ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("[E][MISD] Could not find FAT16/FAT32 partition.");
    return;
  }

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("-->[MISD] Volume type is FAT");
  Serial.println(volume.fatType(), DEC);
  volumesize = volume.blocksPerCluster();  // clusters are collections of blocks
  volumesize *= volume.clusterCount();     // we'll have a lot of clusters
  volumesize *= 512;                       // SD card blocks are always 512 bytes
  volumesize /= 1024 * 1024;
  Serial.print("-->[MISD] Volume size (Mbytes): ");
  Serial.println(volumesize);
  
  if (!SD.begin(SD_CS, SD_MOSI, SD_MISO, SD_CLK)) {
    Serial.println("[E][MISD] initialization failed!");
    return;
  }
}

void printRootFiles() { 
  Serial.println("\n-->[MISD] Files found on the card (name, date and size in bytes):\n");
  root.openRoot(volume);
  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
}



// void writeLog(const char* msg) {
//   // open the file. note that only one file can be open at a time,
//   // so you have to close this one before opening another.
//   myFile = SD.open("log.txt", FILE_WRITE);

//   // if the file opened okay, write to it:
//   if (myFile) {
//     myFile.println(msg);
//     myFile.close();
//   } else {
//     // if the file didn't open, print an error:
//     Serial.println("[E][MISD] error opening test.txt");
//   }
// }

  // // re-open the file for reading:
  // myFile = SD.open("test.txt");
  // if (myFile) {
  //   Serial.println("test.txt:");

  //   // read from the file until there's nothing else in it:
  //   while (myFile.available()) {
  //   	Serial.write(myFile.read());
  //   }
  //   // close the file:
  //   myFile.close();
  // } else {
  // 	// if the file didn't open, print an error:
  //   Serial.println("error opening test.txt");
  // }

 