#include <Arduino.h>
#include <SFE_BMP180.h>
#include <TinyGPS++.h>
#include <RTClib.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define CS_PIN 49
#define MOSI_PIN 51
#define MISO_PIN 50
#define CLK_PIN 52

#define BMP_RES 2 //BMP sensor pressure resolution
#define DHT_PIN 22

#define FAULT_PIN 30 //1st pin for fault-indicating led
#define N_INSTR 6 //n° of instruments installed
#define GPS_WAIT 5 //fallback minutes the GPS has to get a (valid) fix
#define LCD_CAPACITY 80 //char displayable by LCD (now is a 20x4)
#define SAVE_INTERVAL 10 //fallback every hour's X-th minute report saved on SD
#define NAME_SIZE 4 //char size of instrument name
#define LNT_LEN 60 //lenght of lantern instructions array
#define LCDaddress 0x27 //I2C address of LCD

//index for the fault-storage byte
#define GPS_INDEX 0
#define RTC_INDEX 1
#define BMP_INDEX 2
#define DHT_INDEX 3
#define LCD_INDEX 4
#define SD_INDEX 5

//index for setting storage on SD file ALS.DAT
#define GPS_TIMEOUT_INDEX 0
#define SD_SAVE_INDEX 1
#define NAME_INDEX 2
#define LANTERN_INDEX 3

#define T_INDEX 0
#define p_INDEX 1
#define H_INDEX 2
#define WD_INDEX 3 //wind direction
#define WS_INDEX 4 //wind speed

#define RED_PIN 41
#define GREEN_PIN 42
#define BLUE_PIN 45
#define LNT_OFF 03

//delaration of instruments
TinyGPSPlus GPS;
RTC_DS3231 RTC;
SFE_BMP180 BMP180;
DHT DHT1 (DHT_PIN,DHT22);
LiquidCrystal_I2C LCD1(LCDaddress,20,4);
//LiquidCrystal_I2C LCD1(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

//file object declaration
File myFile; //file pointer to the only one file that can be opened at once

//System Strings
PROGMEM const char Splash[] =
		"Automatic LighthouseSystem              RossWorks 2020";
PROGMEM const char ShutDownMessage[]="Ready for shutdown";
PROGMEM const char RebootString[]="Rebooting System...";
PROGMEM const char ReadingSettings[]="Attempting to read settings file...\n";
//GPS & RTC messages
PROGMEM const char GPSStart[] = "Acquiring GPS signalplease wait";
PROGMEM const char GPSOK[] = "GPS signal acquired syncing RTC";
PROGMEM const char GPSbadRTCok[] = "Poor GPS signal     Using RTC time";
PROGMEM const char GPSbadRTCbad[] =
	"No GPS signal       RTC lost power      Date and time       unreliable";
PROGMEM const char GPSupdate[]="Updating GPS fix...";
//instrument reporting & naming
PROGMEM const char Instruments[] = "GPS RTC BMP DHT LCD SD  ";//4 chars/instr
PROGMEM const char NotWorkingEquip[] = "FAULT";
PROGMEM const char WorkingEquip[] = "OK";
PROGMEM const char FallBackName[]="ALSunit";
//Time & logging reporting
PROGMEM const char TimeString[] = "Today is DDD DD MMM YYYY hh:mm:ss ";
PROGMEM const char LogFileHeaderProto[]=
							"%s startup on %02i/%02i/%02i at %02i:%02i:%2i";
PROGMEM const char LogFileHeaderProto2[] ="T\tp\tH\tdir\tspeed\tdate\ttime";

//program mode messages
PROGMEM const char BadCommand[]="Received invalid command\n";
PROGMEM const char EnterProgMode[]="Entering program mode...\n";
PROGMEM const char File2prog[]="stored file\n";
PROGMEM const char Serial2LNT[]="serial input\n";
PROGMEM const char UpdateDone[]="updated\n";
PROGMEM const char NoFile[]="No valid file detected\n";

//lantern managing
PROGMEM const char UpdateLNT[]="Updating lantern routine from ";
PROGMEM const char LantFileUpdated[]="New lantern file is ";
PROGMEM const char BadLant[]="Missing lantern string termination char\n";
// DST rule managing
PROGMEM const char UpdateDST[]="Updating DST rule from";
PROGMEM const char BadDST[]="DST rule not compliant to standard\n";
//mass memory managing
PROGMEM const char EnterMemory[]="Accessing mass storage management\n";
PROGMEM const char BadIO[]="Unable to create requested file\n";
PROGMEM const char RestartAdvise[]="Applying new settings require ALS reboot\n";

//default DST rule (ZULU time)
PROGMEM const byte DefTZ[10]={0,0,3,1,0,0,0,10,2,0};

/*typical log file row:
T	p	H	dir	speed	date time
+22	1012	57	200	12	28-9-1994	00:40
*/
PROGMEM const char LogFileRowProto[]="%+i\t%i\t%i\t%i\t%i\t%i-%i-%i\t%i:%i";
/*PROGMEM const char *DaysNames[]={"Sunday", "Monday", "Tuesday","Wednesday",
									 "Thursday", "Friday", "Saturday"};*/
//default lantern file, just in case we need a reliable filename
PROGMEM const char DefaultLanternFile[13]="DEFAULT.LNT";

//default DST file, just in case we need a reliable filename
PROGMEM const char DefaultDSTFile[13]="ZULU.DST";

//global variables
int GPSfix[7]={-360,61,61/*latitude*/,+360,61,61/*longitude*/,-1000/*height*/};
unsigned int InstrStatus=0;
byte SDinterval=SAVE_INTERVAL;
byte GPStimeout=GPS_WAIT;
unsigned long LastLCDWrite=0;
char MyName[8+1] = "ALSunit";
const char LogfileName[13]="LOGFILE.DAT"; /**current logfile (8.3 name convention)*/
char LanternFile[13]="DEFAULT.LNT"; /**current lantern file (8.3 filename)*/
byte Lantern[LNT_LEN]={3}; /**current lantern pattern (1 byte/second)*/
/*
n-th week of month,n-th day of week,n-th month,n-th hour of day,UTC offset
first the start time of DST,then the end of DST (default @ start is CEST)
*/
byte DST[10]={0,0,3,1,120,0,0,10,2,60};
char StdTime[5]="CET", DstTime[5]="CEST";

int BMP180read(float *outT, float *outp){
	/**
	 * @brief Reads presure and temperature data from BMP180
	 * @param outT variable containing temperature output
	 * @param outp variable containing pressure output
	 * @return 1 if data are collected correctly, 0 otherwise
	 * @author RossWorks
	 */
	byte status; //byte of millis to wait
	double T=0,p=0; //temp variables
	status = BMP180.startTemperature(); //start T reading, will have to wait
	if (status!=0){//status is decided by the sensor. if 0, something went bad
		delay(status);//wait for reading to be ready, delay is OK
		status=BMP180.getTemperature(T);//actual T reading
		if (status!=0){//a status == 0 is sign of bad news
			*outT=T;//assign T to output parameter
			status=BMP180.startPressure(BMP_RES);//now same drill seen for Temp
			if (status!=0){
				delay(status);
				status=BMP180.getPressure(p,T);
				if (status!=0){*outp=p;}
				else{return 0;}//every else reflect status == 0
			}else{return 0;}//return zero means the reading went bad
		}else{return 0;}
	}else{return 0;}
	return 1;//if we avoided every "else" we can return 1 to flag the success
}

byte ChangeDST(const char *InputString){
	/*DST rules are updated from the incoming Bytes. I will implement error 
	tolerance teqniques later*/
	byte i=0, aux=0;
	char DSTFileName[13]={'\0'}; //DST File Name. its content goes to DST rule
	switch (*(InputString+2)){
		case 'F':
			WritePGM2Serial(File2prog,&Serial2);
			for (i=0;i<13;i++){
			/*keep reading chars after the 3 command chars until the file name
			is full OR we hit a semicolon. the semicolon tells to stop. And
			instead of the semicolon we load the string termination char*/
				if(*(InputString+3+i) == ';'){*(DSTFileName+i) = '\0'; break;}
				*(DSTFileName+i) = *(InputString+3+i);
			}
			*(DSTFileName+12)='\0';
			myFile=SD.open(DSTFileName,FILE_READ);
			if (myFile == false){
				WritePGM2Serial(NoFile,&Serial2);return -1;break;}
			for (i=0;i<18;i++){
				if (i<9){*(DST+i)=myFile.read(); continue;}
				if (aux==0){
					*(StdTime+i-9)=myFile.read();
					if (*(StdTime+i-9)==';'){*(StdTime+i-9)=='\0'; aux=1;}
				}
				else{*(DstTime+i-9)=myFile.read();
					if (*(DstTime+i-9)==';'){*(DstTime+i-9)=='\0';}
				}
			}
			break;
		case 'N':
			/*Should final semicolon be missing, ALS fears that the message
			could be corrupted or involuntary. Semicolon acts as a mark
			of intentionality*/
			if (*(InputString+21)!=';'){WritePGM2Serial(BadDST,&Serial2);break;}
			for (i=0;i<18;i++){
				if (i<9){*(DST+i)=*(InputString+i+3); continue;}
				if (aux==0){
					*(StdTime+i-9)=*(InputString+i);
					if (*(StdTime+i-9)==';'){*(StdTime+i-9)=='\0'; aux=1;}
				}
				else{*(DstTime+i-9)=*(InputString+i);
					if (*(DstTime+i-9)==';'){*(DstTime+i-9)=='\0';}
				}
			}
			break;
		default:
			break;
	}
	return 0;
}

byte ChangeLANT(const char *InputChars){
	byte i=0;
	switch (*(InputChars+2)){
		case 'F': //We are setting the lantern according to a known file
			WritePGM2Serial(File2prog,&Serial2); //state what you're doing
			for (i=0;i<13;i++){
			/*keep reading chars after the 3 command chars until the file name
			is full OR we hit a semicolon. the semicolon tells to stop. And
			instead of the semicolon we load the string termination char*/
				if(*(InputChars+3+i) == ';'){*(LanternFile+i) = '\0'; break;}
				*(LanternFile+i) = *(InputChars+3+i);
			}
			*(LanternFile+12)='\0'; //we put a terminator anyway, you never know
			/*It's worth checking if the incoming filename is valid.
			things go south we can revert to default lantern file. If EVEN that
			filename is compromised, ReadLightSetting() will just shut the thing
			off*/
			myFile=SD.open(LanternFile,FILE_READ);
			if (myFile==false){ReadFromPGM(DefaultLanternFile,LanternFile);}
			else{myFile.close();}
			WritePGM2Serial(LantFileUpdated,&Serial2); //state what file is up
			Serial2.println(LanternFile);
			ReadLightSetting();
			break;
		case 'N': //create a new sequence 3+60+1 chars = buffer full!!!!
			WritePGM2Serial(Serial2LNT,&Serial2);
			if (*(InputChars+63)!= ';'){
				/*Should final semicolon be missing, ALS fears that the message
				could be corrupted or involuntary. Semicolon acts as a mark
				of intentionality*/
				WritePGM2Serial(BadLant,&Serial2);
				break;
			}
			for (i=0;i<60;i++){
				*(Lantern+i)=*(InputChars+3+i);
			}
			WritePGM2Serial(UpdateDone,&Serial2);//All done
			break;
		default:
			return -1;
			break;
	}
	return 0;
}

void ClearSerial(Stream *SerialPort){
	char dummy='\0';
	while(SerialPort->available()>0){
		dummy=SerialPort->read();
	}
}

void deg2coords(const double INdegr, int *deg,int *min,int *sec){
	/**
	 * this functions translates a ###.#######° information in a ###°##' ##''
	 * information
	 * @brief transform degrees-only into degree-minute-second coordinate 
	 * @param INdegr Input for degree only coord
	 * @param deg collects the integer part of input
	 * @param min collects minutes of arc from input
	 * @param sec collects seconds of arc from input
	 * @author RossWorks
	 */
	float Aux=0;
	*deg=trunc(INdegr); //isolating degree infrmation as the integer part 
	Aux=INdegr-*deg; //isolating decimal part of the degree original value
	Aux*=60;
	*min=trunc(Aux); //extracting the minutes
	Aux-=*min;
	Aux*=60;
	*sec=trunc(Aux); //extracting seconds
}

byte DHTread(float *outH){
	float H=DHT1.readHumidity();
	if (H!=H){*outH=-1; return 0;}//a bad humidity read generates NaN (NaN!=NaN)
	else{*outH=H; return 1;}//a NaN is never equal to itself
}

bool DSTadj(DateTime *CurrentDate){// all ok
	DateTime StartDST(2000,1,1,0,0,0); // initialize as January 1st, 2000
	DateTime EndDST=StartDST;
	int NowYear = (*CurrentDate).year();
	DSTperiod(NowYear,&StartDST,&EndDST);
	/* quite simple: is the current date lies between start of DST and its end
	it gets corrected for DST, otherwise for non-DST
	both DST and non-DST times are an offset from UTC time, which is provided by
	GPS. UTC time is always the solar time over London*/
	if ((*CurrentDate>StartDST) && (*CurrentDate<EndDST)){
		*CurrentDate = *CurrentDate + TimeSpan(0,0,DST[4],0);
		return true;
	}
	else{
		*CurrentDate = *CurrentDate + TimeSpan(0,0,DST[9],0);
		return false;
	}
}

void DSTperiod(const int year,DateTime *Start,DateTime *End){//ALL OK
	DateTime N1(year,DST[2],1,0,0,0);
	DateTime N2(year,DST[7],1,0,0,0);
	//first we go the the first day of month that is equal to the one we're
	//looking for
	while(N1.dayOfTheWeek()!=DST[1]){
		N1=N1+TimeSpan(1,0,0,0);//otherwise, go for next day
	}
	// then we scroll to defined week of said month (0 = last)
	switch (DST[0]){
		case 0:
		//search last weekday in said month
			while(N1.month()==(N1+TimeSpan(7,0,0,0)).month()){
			//if the next weekday is in another month, we've found last weekday
			N1=N1+TimeSpan(7,0,0,0);
			}
			break;
		default:
			N1=N1+TimeSpan(7*DST[0]);
			break;
	}
	N1=N1+TimeSpan(0,DST[3],0,0);
	*Start=N1;
	//now same drill for the end date of DST
	while(N2.dayOfTheWeek()!=DST[6]){
		N2=N2+TimeSpan(1,0,0,0);
	}
	switch (DST[5]){
		case 0:
			while(N2.month()==(N2+TimeSpan(7,0,0,0)).month()){
			//if the next weekday is in another month, we've found last weekday
			N2=N2+TimeSpan(7,0,0,0);
			}
			break;
		default:
			N2=N2+TimeSpan(7*DST[5]);
			break;
	}
	*End=N2;
}

int GetGPSFix(int *position,unsigned int *time,unsigned int GPStimeOut){//ALL OK
/**
 * @brief this functions interpellates the GPS to get location fix & ZULU time
 * @return 1 if Fix is obtained, 0 if not
 * @param position array containing latitude & longitude in float point degrees
 * @param time array containing time info from year to seconds
 * @author RossWorks
 */
	char c;
	const unsigned long StartTime = millis();
	float latf=0,lonf=0;
	while ((millis()-StartTime)<=(GPStimeOut*6e4)){
	//Serial.println(GPS.sentencesWithFix()); //THE GPS WORKS!!!!!
	/*I've decided to simply let the encode function work without updating any
	warning to user, (LCD, Serial print, etc...) to ensure that the GPS routine 
	won't miss anything. If it misses the GPS fix, that's not my fault lol*/
		if (Serial1.available()){
			c=Serial1.read();
			GPS.encode(c);
		}
		if(GPS.sentencesWithFix()>5 && GPS.location.age()<1000){
			/*I'm going to refuse old (more than 1sec) data.
			when updating GPS I could get old data and the RTC would be set to
			the last time a fix was obtained
			*/
			latf = GPS.location.lat();
			lonf = GPS.location.lng();
			deg2coords(latf,(position+0),(position+1),(position+2));
			deg2coords(lonf,(position+3),(position+4),(position+5));
			*(position+6) = (int)GPS.altitude.meters();
			*(time+0) = GPS.date.year();
			*(time+1) = GPS.date.month();
			*(time+2) = GPS.date.day();
			*(time+3) = GPS.time.hour();
			*(time+4) = GPS.time.minute();
			*(time+5) = GPS.time.second();
			return 1;
		}
	}
	return 0;
}

byte HandleStorage(char *Command){
	char FileName[13]={'\0'},buff='\0';
	byte I=0;
	switch (*(Command+2)){
		case 'D': //Download a file over Serial
			for (I=0;I<13;I++){//reading filename requested
				*(FileName+I)=*(Command+3+I);
				if (*(FileName+I)== ';'){*(FileName+I)='\0'; break;}
			}
			*(FileName+12)='\0'; // putting a EOS char at the end of the array
			myFile=SD.open(FileName,FILE_READ);
			// if file pointer is null, print a warning and exit function
			if (myFile==NULL){WritePGM2Serial(NoFile,&Serial2); return -1;}
			while(myFile.available()>0){
				Serial2.print((char)myFile.read());
			}
			Serial2.print("\n");
			myFile.close();
			break;
		case 'U': //Upload a file via Serial
			for (I=0;I<13;I++){//reading filename to be created
				*(FileName+I)=*(Command+3+I);
				if (*(FileName+I)== ';'){*(FileName+I)='\0'; break;}
			}
			*(FileName+12)='\0'; //EOS char at the end of the array for safety
			myFile=SD.open(FileName,FILE_WRITE);
			// if file pointer is null, print a warning and exit function
			if (myFile==NULL){WritePGM2Serial(BadIO,&Serial2); return -1;}
			while(Serial2.available()>0){
				myFile.print(Serial2.read());
			}
			myFile.println('\0');
			myFile.close();
			// remind to restart ALS if a new ALS.DAT has been uploaded
			WritePGM2Serial(RestartAdvise,&Serial2);
			break;
		default:
			WritePGM2Serial(BadCommand,&Serial2);
			break;
	}
	return 0;
}

byte LCDtest(){
	/**
	 * @brief Tests the LCD
	 * @returns 1 is LCD tests OK, 0 otherwise
	 * @author RossWorks
	 */
	byte error=0;
	Wire.beginTransmission(LCDaddress);
	error=Wire.endTransmission();
	switch(error){
		case 0:
			return 1; //success
			break;
		default:
			return 0;
			break;
	}
	return 0;
}

int ProgramMode(const char *Command){
	/*this function reads the 2nd byte of the command to determine which 
	parameter is to be changed*/
	switch (*(Command+1)){
		case 'D': // chabge DST settings
			WritePGM2Serial(UpdateDST, &Serial2);
			ChangeDST(Command);
			break;
		case 'L'://change Lanter settings
			WritePGM2Serial(UpdateLNT, &Serial2);
			ChangeLANT(Command);
			break;
		case 'M': // mass memory operation
			WritePGM2Serial(EnterMemory,&Serial2);
			HandleStorage(Command);
			break;
		default: //invalid command
			return -1;
			break;
	}
	return 0;
}

byte ReadLightSetting(){
/* lantern instructions work on 60 sec basis: every entry of the array is a sec
of a specified color lightining. this function check if the pointed file exists
before trying to read something from it*/
	int i=0;
	myFile=SD.open(LanternFile,FILE_READ);
	if (myFile==false){return 0;}
	for (i=0;i<LNT_LEN;i++){
		*(Lantern+i)=myFile.read();
	}
	myFile.close();
	return 1;
}

byte ReadGPStimeout(){
	if (bitRead(InstrStatus,SD_INDEX)==1){
		myFile=SD.open(F("ALS.DAT"));
		if (myFile != NULL){
			myFile.seek(GPS_TIMEOUT_INDEX);
			return myFile.read();
			myFile.close();
		}
	}
	return GPS_WAIT;
}

byte ReadSettings(){
	char buff='A', Namebuff[9]={'\0'};
	byte C=0,GPSbuff=0,SDbuff=0,i=0;
	WritePGM2Serial(ReadingSettings,&Serial2);
	myFile=SD.open(F("ALS.DAT"));
	if (myFile==NULL){WritePGM2Serial(NoFile,&Serial2);return -1;}
	do{
		buff=myFile.read();
		if (buff==','){C++;continue;}
		if (buff==';'){break;}
		if (buff==EOF){break;}
		switch (C){
			case GPS_TIMEOUT_INDEX: //reading GPS wait settings
				GPSbuff*=10;
				GPSbuff+=(buff-'0');
				break;
			case SD_SAVE_INDEX: //reading SD saving frequency
				SDbuff*=10;
				SDbuff+=(buff-'0');
				break;
			case NAME_INDEX: //reading unit name
				*(Namebuff+i)=(char)buff;
				i++;
				break;
			default:
				WritePGM2Serial(NotWorkingEquip,&Serial2);
				break;
		}
	}while(buff!=EOF);
	*(Namebuff+8)='\0';
	myFile.close();
	GPStimeout=GPSbuff;
	SDinterval=SDbuff;
	for (i=0;i<9;i++){*(MyName+i)=*(Namebuff+i);}
}

void ReadFromPGM(char *pgm_pointer, char *outString){
	char a=':'; byte i=0;
	while (a!='\0'){
		a=(char)pgm_read_byte(pgm_pointer+i);
		*(outString+i)=a;i++;
	}
}

void(* resetFunc) (void) = 0;  // declare reset fuction at address 0

void SendInstrStatus(){
	byte i=0,j=0,k=0,h=0;
	char report[11]="Arduino";
	for (i=0;i<N_INSTR;i++){
		h=0;
		for (j=NAME_SIZE*i;j<(NAME_SIZE*(i+1));j++){
			*(report+h)=(char)pgm_read_byte(Instruments+j);h++;
			}
		*(report+h)=':';h++;
		if (bitRead(InstrStatus,i)==1){
			for (j=0;j<2;j++){
				*(report+h)=(char)pgm_read_byte(WorkingEquip+j);h++;
			}
			//*(report+h)='1';h++;
		}
		else{
			for (j=0;j<5;j++){
				*(report+h)=(char)pgm_read_byte(NotWorkingEquip+j);h++;
			}
			//*(report+h)='0';h++;
		}
		*(report+h)='\0';
		Serial2.println(report);
		*report='\0';
	}
}

void SetPin(){ //Setting every pin that needs to be set
	byte i=0;
	pinMode(CS_PIN,OUTPUT); //CS pin init for SD comm
	pinMode(RED_PIN,OUTPUT);
	pinMode(GREEN_PIN,OUTPUT);
	pinMode(BLUE_PIN,OUTPUT);
	for (i=0;i<N_INSTR;i++){
		pinMode(FAULT_PIN+i,OUTPUT);
	}
}

void ShutDown(){
	char Answer[20];
	myFile.close();
	digitalWrite(RED_PIN,LOW);
	digitalWrite(GREEN_PIN,LOW);
	digitalWrite(BLUE_PIN,LOW);
	WritePGM2LCD(ShutDownMessage);
	ReadFromPGM(ShutDownMessage,Answer);
	Serial2.println(Answer);
	while (true){}
}

byte UpdateGPS(){
	byte esito=0;
	unsigned int DateAndTime[6]={0};
	LCD1.clear();
	WritePGM2LCD(GPSStart);
	esito=GetGPSFix(GPSfix,DateAndTime,GPStimeout);
	if (esito==1){
		RTC.adjust(DateTime(DateAndTime[0],DateAndTime[1],DateAndTime[2],
						DateAndTime[3],DateAndTime[4],DateAndTime[5]));
		bitWrite(InstrStatus,GPS_INDEX,1);}
	return esito;
}

void UpdateLantern(const byte Seconds){
	char R=0,G=0,B=0;
	R=bitRead(Lantern[Seconds],7)*2+bitRead(Lantern[Seconds],6);
	analogWrite(RED_PIN,255*R/3);
	G=bitRead(Lantern[Seconds],5)*2+bitRead(Lantern[Seconds],4);
	analogWrite(GREEN_PIN,255*G/3);
	B=bitRead(Lantern[Seconds],3)*2+bitRead(Lantern[Seconds],2);
	analogWrite(BLUE_PIN,255*B/3);
}

void WeatherGPS2LCD(const float *weather, const DateTime *Ctime){
	LCD1.clear();
	char LCD_text[21];
	char charlat='N'; char charlon='E';
	if (GPSfix[0]<0){charlat='S';}
	if (GPSfix[3]<0){charlon='W';}
	sprintf(LCD_text,"T=%+2i%cC p=%4ihPa",
					(int)*(weather+T_INDEX),(char)223,
					(int)*(weather+p_INDEX));
	LCD1.setCursor(0,0);
	LCD1.print(LCD_text);
	sprintf(LCD_text,"H=%3i%% W---%c|---kmh",(int)*(weather+H_INDEX),(char)223);
	LCD1.setCursor(0,1);
	LCD1.print(LCD_text);
	sprintf(LCD_text,// the DateTime pointer obj has to be inside()it's a class
			"%c%3i%c%02i'%02i%c %02i:%02i:%02i",
			charlat,abs(GPSfix[0]),(char)223,GPSfix[1],GPSfix[2],'"',
			(*Ctime).hour(),(*Ctime).minute(),(*Ctime).second());
	LCD1.setCursor(0,2);
	LCD1.print(LCD_text);
	sprintf(LCD_text,
			"%c%3i%c%02i'%02i%c  %4im",
			charlon,abs(GPSfix[3]),(char)223,GPSfix[4],GPSfix[5],'"',GPSfix[6]);
	LCD1.setCursor(0,3);
	LCD1.print(LCD_text);
}

unsigned long Write2SD(const float *WeatherData, const DateTime *Nowadays){
	/*A char array (Dumb) is created and filled with the
	prototype of a log file line. Not the most elegant use of memory,
	I will further optimize it
	*/
	char Dumb[40];
	char FileString[40];
	ReadFromPGM(LogFileRowProto,Dumb);
	myFile=SD.open(LogfileName,FILE_WRITE);//work as appending
	sprintf(FileString,Dumb,
	(int)*(WeatherData+T_INDEX),(int)*(WeatherData+p_INDEX),
	(int)*(WeatherData+H_INDEX),
	(int)*(WeatherData+WD_INDEX),(int)*(WeatherData+WS_INDEX),
	(*Nowadays).day(),(*Nowadays).month(),(*Nowadays).year(),
	(*Nowadays).hour(),(*Nowadays).minute());
	myFile.println(FileString);
	myFile.close();
	return millis();
}

void WriteAWR(const float *Wdata, const DateTime *Today,char *report){
	char test;
	byte i=0;
	sprintf(report,
			"%s %02i%02i%02i W%iS%i %+02i/%02i %i",
			MyName,(*Today).day(),(*Today).hour(),(*Today).minute(),
			(int)*(Wdata+WD_INDEX),(int)*(Wdata+WS_INDEX),
			(int)*(Wdata+T_INDEX),(int)*(Wdata+H_INDEX),(int)*(Wdata+p_INDEX));
}

void WritePGM2LCD(char *pgm_address){
	LCD1.clear();
	char cout='\0';
	byte row=0,column=0;
	byte i=0;
	for (i=0;i<81;i++){
		cout=(char)pgm_read_byte(pgm_address+i);
		if (cout == '\0'){break;}
		else{
			column = i % 20;
			row = i / 20;
			LCD1.setCursor(column,row);
			LCD1.print(cout);
		}
	}
	delay(2e3);
}

void WritePGM2Serial(char *pgm_address, Stream *SerPort){
	char cout='\0';
	byte i=0;
	cout=(char)pgm_read_byte(pgm_address);
	while (cout != '\0'){
		SerPort->print(cout);
		i++;
		cout=(char)pgm_read_byte(pgm_address+i);
	}
}

void WritePositionReport(char *report){
	char charlat='N'; char charlon='E';
	if (GPSfix[0]<0){charlat='S';}
	if (GPSfix[3]<0){charlon='W';}
	sprintf(report,"%c%3i%c%02i'%02i%c %c%3i%c%02i'%02i%c  %4im",
			charlat,abs(GPSfix[0]),'°',GPSfix[1],GPSfix[2],'"',
			charlon,abs(GPSfix[3]),'°',GPSfix[4],GPSfix[5],'"',
			GPSfix[6]);
}

void setup(){
	/*
	1)Initialise/test every instruments/pin/serial/I2C/etc. + 
	  display splashscreen + Lantern setting + 
	  reading, if posible, settings file ALS.DAT and apply said settings
	2)Time & position phase
	|-> 1)Read GPS location&time, flag if result valid within time limit.
	|   GPS function saves position in global array. after use, GPS is shut off
	|-> 2)Set RTC clock in UTC, will be corrected for timezone/DST every time
	|   the current time is needed
	3)Setup now reads all the weather sensors
	|->	1)BMP180 for temperature & pressure
	|->	2)DHT22 read & test
	4) and writing to LCD
	5) Writes the booting process results in a log file (if SD is available)
	   the log file is unique and updated every time
	*/
	unsigned int PresentDay[6]={0};
	byte esito=0;
	bool IsDST=0;
	DateTime NowToday(2000,1,1,0,0,0);
	float WeatherData[3]={0};
	char LogFileText[64],RowProto[40], name[8];
	
	//phase 1
	Serial1.begin(9600); //GPS uses Serial1. 9600 baud as default
	Serial2.begin(9600); //Bluetooth uses Serial2 9600 baud as default
	Serial2.setTimeout(360e3);
	Wire.begin();
	SetPin();//setting pins
	bitWrite(InstrStatus,LCD_INDEX,LCDtest()); //LCDtest tries to comm with LCD
	if (bitRead(InstrStatus,LCD_INDEX)==1){
		LCD1.init();// init of lcd display
		LCD1.backlight(); //turn on the backlight
		WritePGM2LCD(Splash); //Write the splash,just to say the thing is alive
	}
	bitWrite(InstrStatus,RTC_INDEX,RTC.begin());//check if RTC is available
	bitWrite(InstrStatus,BMP_INDEX,BMP180.begin());//BMP status
	DHT1.begin();//init of DHT. set a pin, and a timer, but returns nothing.
	//SD init, false if failure
	if (SD.begin()){
		bitWrite(InstrStatus,SD_INDEX,1);
		ReadSettings();
	}
	else{bitWrite(InstrStatus,SD_INDEX,0);}
	
	//phase 2
	// I warn that we're waiting for GPS fix
	if (bitRead(InstrStatus,LCD_INDEX)==1){WritePGM2LCD(GPSStart);}
	WritePGM2Serial(GPSStart,&Serial2);
	esito=GetGPSFix(GPSfix,PresentDay,GPStimeout); //saving fix and time
	bitWrite(InstrStatus,GPS_INDEX,esito);//write if GPS is OK
	if (esito==1){
		/*GPS ok, we set RTC time*/
		WritePGM2LCD(GPSOK);
		RTC.adjust(DateTime(PresentDay[0],PresentDay[1],PresentDay[2],
							PresentDay[3],PresentDay[4],PresentDay[5]));
		bitWrite(InstrStatus,RTC_INDEX,1);}
	else if(RTC.lostPower()){
		WritePGM2LCD(GPSbadRTCbad);/*RTC has stopped and we can't set it*/
		bitWrite(InstrStatus,RTC_INDEX,0);}
	else{/*GPS failed, but RTC has a time set. we'll do with that*/
		WritePGM2LCD(GPSbadRTCok);
		bitWrite(InstrStatus,RTC_INDEX,1);}
	delay(1500);
	
	//phase 3
	NowToday=RTC.now();//getting current UTC time
	IsDST=DSTadj(&NowToday);//adj UTC date for current time zone and DST
	esito=BMP180read(WeatherData+0,WeatherData+1);//reading BMP180
	bitWrite(InstrStatus,BMP_INDEX,esito);
	esito=DHTread(WeatherData+2);//DHT22 read
	bitWrite(InstrStatus,DHT_INDEX,esito);
	
	//phase 4
	WeatherGPS2LCD(WeatherData,&NowToday);
	LastLCDWrite=millis();
	
	//phase 5
	esito=bitRead(InstrStatus,SD_INDEX);
	if (esito==1){
		myFile=SD.open(LogfileName,FILE_WRITE);
		ReadFromPGM(LogFileHeaderProto,RowProto);
		sprintf(LogFileText,RowProto,
		MyName,NowToday.day(),NowToday.month(),NowToday.year(),
		NowToday.hour(),NowToday.minute(),NowToday.second());
		myFile.println(LogFileText);
		ReadFromPGM(LogFileHeaderProto2,RowProto);
		myFile.println(RowProto);
		myFile.close();
	}
}

void loop(){
	static unsigned long LastSDWrite=0;
	byte i=0;
	bool IsDST=false;
	char BTinput[64]="5Arduino"; //BT input string storage
	char Answer[64]="Arduino"; //the space of serial buffer for writing answers
	float WeatherData[5]={0}; //Temp, pres & Humidity + Wind speed/direction
	DateTime NowToday(2000,1,1,0,0,0); // this is today
/*-------------------------weather data read and LCD writing------------------*/
	NowToday=RTC.now();//getting current UTC time
	IsDST=DSTadj(&NowToday);//adj UTC date for current time zone and DST
	bitWrite(InstrStatus,BMP_INDEX,//reading BMP180 + testing
			 BMP180read(WeatherData+T_INDEX,WeatherData+p_INDEX));
	//DHT22 read + test
	bitWrite(InstrStatus,DHT_INDEX,DHTread(WeatherData+H_INDEX));
	if (bitRead(InstrStatus,LCD_INDEX)==1){
		if ((millis()-LastLCDWrite)>=1000){ //if more than a sec has passed
			WeatherGPS2LCD(WeatherData,&NowToday); //write to LCD
			LastLCDWrite=millis();//and save last time we updated LCD
		} 
	}
/*------------------------SD saving log file----------------------------------*/
	if (SD.begin()){//first: is SD OK?
		bitWrite(InstrStatus,SD_INDEX,1); //update SD fault flag
		//then timing is checked
		if ((NowToday.minute()==0) || ((NowToday.minute() % SAVE_INTERVAL)==0)){
			if ((millis()-LastSDWrite)>6e4){//have we writed in the last minute?
				LastSDWrite=Write2SD(WeatherData,&NowToday);
			}
		}
	}
	else{bitWrite(InstrStatus,SD_INDEX,0);}//update SD fault flag
/*-----------------------------Lantern Updating-------------------------------*/
	UpdateLantern(NowToday.second());
/*-----------------------------BT command reception---------------------------*/		
	//if BT serial interface has data let'see what has to be done
	Answer[0]='\0'; //resetting answer string
	//reading new bytes in serial buffer
	while (Serial2.available()>0){*(BTinput+i)=Serial2.read();i++;}
		switch (*BTinput){
			case 'A': //Ask for an Automatic Weather Report (AWR)
				WriteAWR(WeatherData,&NowToday,Answer);
				Serial2.println(Answer);
				break;
			case 'D': //D for Date&time
				ReadFromPGM(TimeString,Answer);
				Serial2.print(NowToday.toString(Answer));
				if(IsDST){Serial2.println(DstTime);}
				else{Serial2.println(StdTime);}
				break;
			case 'F': //as Fix position reporting
				WritePositionReport(Answer);
				Serial2.println(Answer);
				break;
			case 'G': // as GPS update
				WritePGM2LCD(GPSStart);
				ReadFromPGM(GPSupdate,Answer);
				Serial2.print(Answer);
				bitWrite(InstrStatus,GPS_INDEX,UpdateGPS());
				Serial2.println(bitRead(InstrStatus,GPS_INDEX));
				break;
			case 'P': // Program mode to set the board w/o reprogramming
				WritePGM2Serial(EnterProgMode, &Serial2);
				ProgramMode(BTinput);
				break;
			case 'Q': //Q acts as a handshake between ALS and python GUI
				break;
			case 'R':
				WritePGM2LCD(RebootString);
				WritePGM2Serial(RebootString,&Serial2);
				Serial2.println(' ');
				delay(1000);
				resetFunc();
			case 'S': //S for Status: how is the system doing?
				SendInstrStatus();
				break;
			case 'T': //test branch
				break;
			case 'Z':
				ShutDown();
				break;
			default: //fallback case
				break;
		}
	ClearSerial(&Serial2);
}