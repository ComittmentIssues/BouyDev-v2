//DS18B20 1-Wire Device  Version 2
//Kevin Darrah
//www.kevindarrah.com
//  4-17-14

/*
Release Notes:

-Fixed a few bugs with writes to a specific pin number, rather than the variable 'pin'
-Improved timing.  I found that in cold temperatures the CRC checker was finding errors every once and a while
-Added a convert checker. Now when you convert, you don't have to wait out the 750ms.  The code checks for a complete bit
-  NOTE! This only works when when you're using 3 wires, not in parasitic mode
-Added a CRC count, so you can keep an eye on total errors.

...Library coming soon
*/

byte ROM[100][8];
//This is used to store the 64 bit addresses of the devices. They start at ROM[1]
byte num_devices=1, alarm_devices, CRC_Error;
//num of devices is the number of devices found.  A value of 10 means 10 found, and they start at 1
int CRC_count=0; //Just to keep a count of how many errors
float tempC_decimal=0.0000, tempF_decimal=0.0000;
//These temps will hold the last value read from the sensor
byte LSB, MSB, TH, TL, Config;
//These are the RAW bytes read out of the sensor, and are last read
byte TempSensor1[8]={0x28,0xBD,0xA2,0xA0,0x5,0x0,0x0,0x7F};
//if you want to assign your addresses, and not use the search function

void setup(){//   setup  start   setup  start   setup  start   setup  start   setup  start   setup  start
  
  //for(byte i=0; i<8; i++)//this will assign your defined address to the ROM[0] location
  //ROM[0][i] = TempSensor1[i];
  
  Serial.begin(115200);//note! you will always need to include serial communication even if you don't use it
  //go through the code and remove serial.print commands if you don't need it
  
  DSB_search_all(7, 1);//search for all devices on digital pin 8, and serial.print all addresses
//(pin, print) pin is the digital pin, and print is 0 or 1, depending on if you want to serialprint what was found


//for(byte i=1; i<=num_devices; i++)//write to every found device
//DSB_scratch_write(8, i, 25, 0, 0xFF);//writes to the scratchpad of the DS18B20
//(digital pin, ROM address, high alarm temp, low alarm temp, configuration byte)
//this also copies the information the EEPROM of the devices

}//   setup  end   setup  end   setup  end   setup  end   setup  end   setup  end   setup  end   


void loop(){//    LOOP START    LOOP START    LOOP START    LOOP START    LOOP START    LOOP START  
  //DSB_search_all(7, 1);//pin, print
  //Serial.println(CRC_count);//print the serial count
  //(pin 8, 0=no print)  This is to fill the ROM[][] with the addresses found, and to
  //have a value for num_devices
  
 // DSB_Convert_All(8);//pin
  //this is to be called before a search_alarm function, so it executes a 0x44 (convert) 
  //command to each device found on the bus in DSB_search_all
  
  //DSB_search_alarm(8, 1);// pin, print
  //this searches for all of the devices that are in a high temp or low temp condition as
  //defined by the values written in DSB_scratch_write
  //alarm_devices will have the number of alarmed devices, and the ROM[][] array will be 
  //filled up with the new devices found that are alarming

  DSB_get_temp_one_device(7,1);//pin, print
  //if only one device is connected, this will go out and grab the temperature of the single 
  //device (pin 8, will serial.print the temp)

  //DSB_get_one_address(8,1);//pin, print
  //will go out an grab the address of the single device connected and store it in ROM[0][0..7]
  //can only use this if only one device is on the bus
  //for(int h=1; h<=num_devices; h++)//use this to get all temps on the devices found
  //DSB_get_temp_address(7,h,1);//pin, address, printer
  //will get the temp of the device at an address on the bus. Address is the index of the ROM[x] array
   delay(500);
  
  
}//      LOOP  END      LOOP  END      LOOP  END      LOOP  END      LOOP  END      LOOP  END


void DSB_byte_write(byte command, byte pin){//    BYTE WRITE START START
int i;// don't hate on my use of i everywhere!  I know, I know bad programming Kevin!

//We need to write a BYTE "command" to the digital pin "pin"


  for(i=0; i<8; i++){//8 bits in a byte, do this 8 times, least significant bit
pinMode(pin, OUTPUT);//get ready to write
digitalWrite(pin, LOW);//go LOW for the entire 60ish microseconds
delayMicroseconds(1);
  
   if(bitRead(command, i)){//if the first bit is a '1'
     //might get rid of this, probably not needed
    digitalWrite(pin,HIGH);//since this is a '1', change into an input, and let the pull-up resistor yank the line HIGH
    delayMicroseconds(59);//chill out for 60micro secs, to let the DS18B20 get a read of the high signal
   }//true
   else{//nope it was a '0'
    delayMicroseconds(59);//the DS18B20 will come in and read the low line as a 0 here
    digitalWrite(pin,LOW);//back to input, line gets yanked high, and ready for the next bit
   }//else
   
  }//for i=0
}////    BYTE WRITE END END

byte DSB_byte_read(byte pin){//                  BYTE READ START START
  int i;
  byte byte_in;
  
  //reads a byte from the DS18B20, this is not a 'void' function so it returns the byte read in
  
 for(i=0; i<8; i++){//8 bits in a byte
    pinMode(pin, OUTPUT);//get ready
    digitalWrite(pin, LOW);//drive low to tell the DS18B20 we're ready
    //used to have a delay of 1us here, turned out it was giving corrupt data
   delayMicroseconds(1);
   pinMode(pin, INPUT);// change to an input to allow line to yank HIGH
   delayMicroseconds(5);//The DS18B20 now has control over the line
   //if pulled low, then we read in a '0', if high, then the bit is a '1'
   if(digitalRead(pin)==1)//grab the line status
   bitSet(byte_in,i);//set it or...
   else
   bitClear(byte_in,i);//clear it
   
   delayMicroseconds(54);//wait out the rest of the timing window
 }//for

  return(byte_in); 

}////    BYTE READ END END


void DSB_Reset(byte pin){//                       RESET  START START
//this is needed before every transaction with the DS18B20

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(500);//drive the line low for 500us
  pinMode(pin, INPUT);//then let it pull-up for 500us
  delayMicroseconds(500);//PRESENCE pulse occurs here, but we don't care about it
  
}//                       RESET  END END



void DSB_search_all(byte pin, boolean printer){//      SEARCH ALL START START
  DSB_Search(0xF0, pin, printer);//send a 'search all' command to the main search function
}
void DSB_search_alarm(byte pin, boolean printer){//    SEARCH ALARM START START
  DSB_Search(0xEC, pin, printer);//send a search for alarm dvices only
  alarm_devices=num_devices;//set the alarm-dvices variable with whatever was found in the last search
  DSB_Search(0xF0, pin, 0);//search again for everything, to refresh the ROM addresses and the num_devices
}
void DSB_get_temp_one_device(byte pin, byte printer){//   GET TEMP ONE START START
  int i;
  
  //this function gets the temperature of a device if only one device is on the bus
  
  LSB=0;
  MSB=0;
  tempC_decimal =0;
  DSB_Reset(pin);//reset
  DSB_byte_write(0xCC, pin);//skip the ROM address, since there is only one device
  DSB_byte_write(0x44, pin);//convert the temp
  while(1){
  if(search_read(pin)==1)
  break; 
  }//while
  DSB_Reset(pin);//reset again
  DSB_byte_write(0xCC, pin);//skip again
  DSB_scratch_read(pin);//then read out the scratch pad, which includes the temp

if(printer==1){//will print out here if desired
Serial.print(tempC_decimal, 4);
Serial.print("C ");
Serial.print(tempF_decimal, 4);
Serial.println("F ");
}  
 
}//GET TEMP ONE END END


void DSB_get_one_address(byte pin, byte printer){//   GET ADDR ONE START START
  int i;
  
  //gets the address of one device on the bus
  
  DSB_Reset(pin);//reset
  DSB_byte_write(0x33, pin);//read ROM address
  for(i=0; i<8; i++){//get the 8 bytes
  ROM[0][i]=DSB_byte_read(pin);//everything is stored in ROM[0][0..7]
  }
  
  if(printer==1){//print it off if desired
     for(i=0; i<8; i++){
  Serial.print(ROM[0][i],HEX); 
    Serial.print(":"); }
    Serial.println("");
    
  }//printer
  
  
  
}//   GET ADDR ONE END END

void DSB_get_temp_address(byte pin, byte address, byte printer){// GET TEMP ADDR START START
  int i;
  LSB=0;
  MSB=0;
  tempC_decimal =0;
  
  //this gets the temp at a specific address on the bus 0.....
  
  DSB_Reset(pin);//reset
  DSB_byte_write(0x55, pin);// match the 8byte ROM
  for(i=0; i<8; i++)
  DSB_byte_write(ROM[address][i], pin);//send out the address
  DSB_byte_write(0x44, pin);//convert
  //delay(750);//chill for a sec
  while(1){
  if(search_read(pin)==1)
   break; 
  }//while
  DSB_Reset(pin);//start over with a reset
  DSB_byte_write(0x55, pin);//match again
  for(i=0; i<8; i++)
  DSB_byte_write(ROM[address][i], pin);//8 byte address out again
  DSB_scratch_read(pin);//read the scratch pad



if(printer==1){//print if desired
Serial.print(tempC_decimal, 4);
Serial.print("C ");
Serial.print(tempF_decimal, 4);
Serial.println("F ");
}    
}// GET TEMP ADDR END END

void DSB_Convert_All(byte pin){//                CONVERT ALL START START
int i, j;

//needed to convert everything, to save time
//this is used when searching for alarmed devices

  for(i=1; i<=num_devices; i++){//same as temp reads, except we stop at convert
  DSB_Reset(pin);
  DSB_byte_write(0x55, pin);
  for(j=0; j<8; j++)
  DSB_byte_write(ROM[i][j], pin);
  DSB_byte_write(0x44, pin);
  //delay(10);
  }//i
  
}//                CONVERT ALL END END

void DSB_scratch_write(byte pin, byte address, byte high, byte low, byte cnfig){//SCRATCH WRITE START START
  int j;
  
  //writes to the scratch pad and copies to the EEPROM of the device
  
  DSB_Reset(pin);//reset
  DSB_byte_write(0x55, pin);//match the ROM
  for(j=0; j<8; j++)
  DSB_byte_write(ROM[address][j], pin);//8byte address out
  
  DSB_byte_write(0x4E, pin);//write scratch
  DSB_byte_write(high, pin);//high alarm
  DSB_byte_write(low, pin);//low alarm
  DSB_byte_write(cnfig, pin);//config
  
  //back in to write that to the EEPROM
  DSB_byte_write(0x55, pin);
  for(j=0; j<8; j++)
  DSB_byte_write(ROM[address][j], pin);
  DSB_byte_write(0x48, pin);//eeprom copy
}//SCRATCH WRITE END END
void DSB_scratch_read(byte pin){//                    SCRATCH READ START START
  int i, j;
  byte data_in[9], CRC=0;
  boolean CRCxor;
  tempC_decimal=0;
  
  //things are about to get a little crazy here
  //this is where the scratchpad is read out and checked against the CRC
  //the LSB MSB of the temperature is converted into a float value
  
  
  DSB_byte_write(0xBE, pin);//read scratch
for(i=0; i<9; i++){//9 bytes are read in
  data_in[i]=DSB_byte_read(pin);//keep it all in data_in[0..8]
  }
  
 for(i=0; i<8; i++){//CRC checker, 8 bytes wide, 9th byte is the CRC 
   for(j=0; j<8; j++){//8 bits each
   CRCxor = bitRead(data_in[i], j) ^ bitRead(CRC, 0);//XOR CRC bit 0 with the incoming bit
   bitWrite(CRC, 3, (CRCxor^bitRead(CRC, 3))); //take that XOR result and XOR it with the 3rd bit in the CRC
   bitWrite(CRC, 4, (CRCxor^bitRead(CRC, 4)));//do the same with the 4th bit
   CRC = CRC>>1;//shift the whole CRC byte over to the right once
   bitWrite(CRC, 7, CRCxor);//now store the XOR bit into the last position of the CRC byte
   }//this goes on for the entire 8 bytes
  }
 //Serial.println(CRC, HEX);//print stuff if you want to see the results
 //Serial.println(data_in[8], HEX);
  if(CRC==data_in[8]){//okay, now check the CRC you made with the 9th byte that came in
  //if we're good, go ahead and set everybody up
    LSB=data_in[0];
    MSB=data_in[1];
    TH=data_in[2];
    TL=data_in[3];
    Config=data_in[4];
   CRC_Error=0;   
  }
  else//if not, set everybody to 0
  {
    CRC_Error=1;//ALSO - set a flag, you can use this to see if there was a fail
    CRC_count++;//increment the count
    LSB=0;
    MSB=0;
    TH=0;
    TL=0;
    Config=0;  
  }

  
  if(bitRead(MSB, 7)==0){//check to see if the temp is negative
  //positive temperature
  
  for(i=0; i<4; i++){//first set the fractional part of the number
  if(bitRead(LSB,i)==0)//only the bottom 4 bits represent 0.5, 0.25, 0.125, 0.0625
   tempC_decimal = tempC_decimal+pow(2.0000, -4.0000+i);//yes, this is weird, but it makes sense,
  // 2^-1 = 0.5, 2^-2=0.25, 2^-3=0.125, and 2^-4=0.0625
  }//for
  
  
//tempC is the fractional part right now, so we add that to the integer part
//the top 4 of LSB are 2^0, 2^1... so we shift that to the right 4 times to get it lined up
//then the MSB only contains 3 contributing bits, so we AND off everything else
//then shift that to the left 4 times to line it up with the LSB we just shifted over
tempC_decimal = tempC_decimal + (LSB>>4) + ((B00000111 & MSB)<<4); 
//convert to F, for those of us who can't use Celsius :)
tempF_decimal = (tempC_decimal)*9.0000/5.0000+32.00000;}
else
{//or..... if the temp is NEGATIVE

//the negative temp comes in as a 2's comliment number, so we need to convert it back

//I did this kinda weird, but I combined the LSB with MSB into a word, then inverted everything, 
// by using a ^ against 0xFFFF, then added '1'.  This converts back to what we were dealing with 
//if the number was positive
word full16 = ((((B00000111 & MSB)<<8) + LSB) ^ 0xFFFF)+1;
MSB = full16>>8;//but, I already had my code ready to work with MSB and LSB, so I pulled them back out
LSB=  full16 & 0x00FF;
 
  //now we're back to normal, so I pulled the same trick as before
  for(i=0; i<4; i++){
  if(bitRead(LSB,i))
   tempC_decimal = tempC_decimal+pow(2.0000, -4.0000+i);
  }//for 
  
//and this is the same as before, now with a '-1 multiplier'
tempC_decimal = (tempC_decimal + (LSB>>4) + ((B00000111 & MSB)<<4))*-1; 
tempF_decimal = (tempC_decimal)*9.0000/5.0000+32.00000;
  
}//else neg

}//                    SCRATCH READ END END


void DSB_Search(byte search_command, byte pin, boolean printer){// SEARCH SEARCH SEARCH START START
  int i, j, k, n;
  
  //this contains the function to search out all devices on the bus
  
  byte true_check[8]={0,0,0,0,0,0,0,0};//the first read is stored here
  byte comp_check[8]={0,0,0,0,0,0,0,0};//compliment of first read
  byte last_disc[8]={0,0,0,0,0,0,0,0};//last discrepency position
  byte disc_check[8]={0,0,0,0,0,0,0,0};//discrepency action here, or which way to drive the line
  num_devices=1;//well, there should be at least one to start with
  
  //all starts here
  for(i=0; i<num_devices; i++){//start with <1, but num_devices increases as we find new devices
      DSB_Reset(pin);//reset the bus
      DSB_byte_write(search_command, pin);//depends on if we are searching all or just for alarms
      
    for(k=0; k<8; k++){//8bytes
    for(j=0; j<8; j++){//8bits each
    bitWrite(true_check[k], j, search_read(pin));//go read the first bit
    bitWrite(comp_check[k], j, search_read(pin));//go read the compliment
    
    //now check for discrepency, or 0-0.  Meaning 1 device has a 0 and one has a 1 in the current bit position
    if(bitRead(true_check[k], j)==0 && bitRead(comp_check[k], j)==0){
    
      //whatever is in disc_check for this position will determine what to do next, but by default,
      //it will be 0, at least on the first pass
    if(bitRead(disc_check[k], j)==0){
    for(n=0; n<8; n++){//we first wipe out last_disc, because we are about to write a new value in
    last_disc[n]=0;
    }
    bitSet(last_disc[k], j);//we set this, so we know where the last discrepeny in the chain is at
    bitWrite(ROM[num_devices][k], j, 0);//set the bit in the address to 0
    search_write(0, pin);//write a '0' to the line, drive LOW
    }//==0
    else{//if disc_check for this position was a 1, then we do this
    bitWrite(ROM[num_devices][k], j, 1);//set the bit position for the address to a 1
    search_write(1, pin);//write a '1' to the line, or drive HIGH
      }//else  

    }//if disc found
    else//OR if we had both a '1' and a '1', then nobody is on the line, so kill the num_devices to 0
    if(bitRead(true_check[k], j)==1 && bitRead(comp_check[k], j)==1)
    {
    num_devices=0;
    }
    else//otherwise, there was no discrepency found, and all devices on teh line are at the first read
    {//this is a 0-1 or 1-0 read-read
    bitWrite(ROM[num_devices][k], j, bitRead(true_check[k], j));//set the address location
    search_write(bitRead(true_check[k], j), pin);//and write the line with what we checked
    }
    
    //do this for the 64 bits.....
    }//j
    }//k
    
    //okay, now an entire address is written, so now we check to see if a discrepency was found,
    //if so, then we need to go back, and change the way we handle the discrepency, by writing a '1' to the line
    //instead of a '0', or vice versa
    
    for(k=7; k>=0; k--){//we work our way backwards, because we only care about the last disc found
      for(j=7; j>=0; j--){
 
        //two checks here, becuase, we don't want to want to change the way we handled
       // something if we already checked it the last time around
        if(bitRead(last_disc[k],j)  && bitRead(disc_check[k],j)==0){
        bitSet(disc_check[k],j);//set disc_check to a one, now it will write a '1' to the line in that position
        num_devices++;//new device found!
        k=0;//get out of the for loops
        j=0;
      }
      else
      bitClear(disc_check[k],j);//if no discrepency found at teh current position, the we can clear the bit
      }}//j k
      
      //now go and check the bus again with the new disc_check rules
  }//i
  
  //done finding devices, now print if desired
  if(printer==1){
  Serial.print("Found ");
  Serial.println(num_devices);
  for(j=1; j<=num_devices; j++){
  for(i=0; i<8; i++){
    Serial.print(ROM[j][i], HEX);
    Serial.print(":");
  }//for
  Serial.print("  ");
  DSB_get_temp_address(pin, j, 1);//print off the temps as well

}//j
  }//if printer
  
  
}// SEARCH SEARCH SEARCH END END END

boolean search_read(byte pin){// SEARCH READ START
   //delayMicroseconds(1);
//used by the Search routine
   pinMode(pin, OUTPUT);//trigger a reading
   digitalWrite(pin, LOW);//go low
   delayMicroseconds(1);//might get rid of
   pinMode(pin, INPUT);//release the line and wait
   delayMicroseconds(5);
   
   //now see if teh line is high or low, indicating bit value
   if(digitalRead(pin)==HIGH){
     delayMicroseconds(54);
   return 1;
   }
   else{
     delayMicroseconds(54);
   return 0;

   }
}// SEARCH READ END
void search_write(boolean val, byte pin){// SEARCH WRITE START

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(1);
//used by the Search routine
     if(val==0){  
    delayMicroseconds(59);
    pinMode(pin, INPUT);
    //delayMicroseconds(1);  
}
    else{//if 1, then go low, then high immediately for the rest of the time
    pinMode(pin, INPUT);
    delayMicroseconds(59);
 }
}// SEARCH WRITE END


