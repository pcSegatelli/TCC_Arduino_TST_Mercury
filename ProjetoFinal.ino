/* Nome do arquivo: ProjetoFinal.pde
* Modificado por: Paula Cristina Segatelli Costa
* Data das modificacoes: 2014.03.10 
* Descricao: Tag RFID é liga pelo Mifare no qual o CI DS1307 gera a data e hora da leitura e armazena
* isso mais os tempos parciais e totais calculados pelo arduino no cartao SD em formato texto
//-------------------------------------------
* Nome original do arquivo: BigProject.pde
* Criador: Evan(WWW.B2CQSHOP.COM)
* Data da Criacao: 2011.09.19
*/
//+++++++++++++++++++++++++++++++++++++++++++

//Bibliotecas
#include <SPI.h>           // O sensor comunica-se usando SPI, de modo a incluir a biblioteca.
#include <SdFat.h>        //  Biblioteca para utilizacao do SD
#include <SdFatUtil.h>   //   Biblioteca para utilizacao do SD
#include <Wire.h>       //    Biblioteca do CI DS1307  
#include <DS1307.h>    //     Biblioteca do CI DS1307
//++++++++++++++++++++++++++++++++++++++++++

#define uchar unsigned char
#define uint unsigned int

#define MAX_LEN 16 //Comprimento maximo da matriz
#define DS1307_I2C_ADDRESS 0x68 //DS1307

//++++++++++++++++++++++++++++++++++++++++++
//Setar os pinos
const int selectSd = 3;
const int selectRfid = 4;
const int NRSTPD = 5;
//++++++++++++++++++++++++++++++++++++++++++

//COMANDOS MIFARE
#define PCD_IDLE 0x00 //NO action;Cancelar o comando atual
#define PCD_AUTHENT 0x0E //Chave de Autentificacao
#define PCD_RECEIVE 0x08 //Receber Dados
#define PCD_TRANSMIT 0x04 //Enviar Dados
#define PCD_TRANSCEIVE 0x0C //Enviar e Receber Dados
#define PCD_RESETPHASE 0x0F //Reiniciar
#define PCD_CALCCRC 0x03 //CRC Calcular

//Mifare_One COMANDOS DO CARTAO MIFARE
#define PICC_REQIDL 0x26 //A zona nÃ£o encontrou a antena em modo de hibernacao
#define PICC_REQALL 0x52 //Da antena a area para encontrar todos os cartoes
#define PICC_ANTICOLL 0x93 //Anti-colisao
#define PICC_SElECTTAG 0x93 //TAG
#define PICC_AUTHENT1A 0x60 //Verificacao da chave A
#define PICC_AUTHENT1B 0x61 //Verificacao da chave B
#define PICC_READ 0x30 //Bloco de leitura
#define PICC_WRITE 0xA0 //Bloco de Gravacao
#define PICC_DECREMENT 0xC0 //Debito
#define PICC_INCREMENT 0xC1 //Recarregar
#define PICC_RESTORE 0xC2 //Transferir blocos de dados para o buffer
#define PICC_TRANSFER 0xB0 //Armazenar no buffer de dados
#define PICC_HALT 0x50 //Dormencia

//Retorno de codigo de erro  ao comunicar com MF522
#define MI_OK 0
#define MI_NOTAGERR 1
#define MI_ERR 2

//++++++++++++++++++++++++++++++++++++++++++
//-----------Registro MFRC522---------------
//Page 0:Comandos e estado
#define Reserved00 0x00
#define CommandReg 0x01
#define CommIEnReg 0x02
#define DivlEnReg 0x03
#define CommIrqReg 0x04
#define DivIrqReg 0x05
#define ErrorReg 0x06
#define Status1Reg 0x07
#define Status2Reg 0x08
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define WaterLevelReg 0x0B
#define ControlReg 0x0C
#define BitFramingReg 0x0D
#define CollReg 0x0E
#define Reserved01 0x0F

//Page 1:Comandos
#define Reserved10 0x10
#define ModeReg 0x11
#define TxModeReg 0x12
#define RxModeReg 0x13
#define TxControlReg 0x14
#define TxAutoReg 0x15
#define TxSelReg 0x16
#define RxSelReg 0x17
#define RxThresholdReg 0x18
#define DemodReg 0x19
#define Reserved11 0x1A
#define Reserved12 0x1B
#define MifareReg 0x1C
#define Reserved13 0x1D
#define Reserved14 0x1E
#define SerialSpeedReg 0x1F

//Page 2:CFG
#define Reserved20 0x20
#define CRCResultRegM 0x21
#define CRCResultRegL 0x22
#define Reserved21 0x23
#define ModWidthReg 0x24
#define Reserved22 0x25
#define RFCfgReg 0x26
#define GsNReg 0x27
#define CWGsPReg 0x28
#define ModGsPReg 0x29
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D
#define TCounterValueRegH 0x2E
#define TCounterValueRegL 0x2F

//Page 3:TestRegister
#define Reserved30 0x30
#define TestSel1Reg 0x31
#define TestSel2Reg 0x32
#define TestPinEnReg 0x33
#define TestPinValueReg 0x34
#define TestBusReg 0x35
#define AutoTestReg 0x36
#define VersionReg 0x37
#define AnalogTestReg 0x38
#define TestDAC1Reg 0x39
#define TestDAC2Reg 0x3A
#define TestADCReg 0x3B
#define Reserved31 0x3C
#define Reserved32 0x3D
#define Reserved33 0x3E
#define Reserved34 0x3F

//////////////////////////////////////
// store error strings in flash to save RAM
#define error(s) Error_P(PSTR(s))
///////////////////////////////////////////
// for RTC work, DS1307
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
/////////////////////////////////////////////
uchar serNum[5]; 
//4 bytes cartao de numero de serie, o quinto byte byte de checksum

// Salva os dados lidos a partir da RFID
uchar str_tem[MAX_LEN];
uchar str_name[MAX_LEN];
///////////////////////////////////////////
// Inicializar variÃ¡veis
// Primeiro de quatro necessario para o trabalho de cartao SD

Sd2Card card;
SdVolume volume;
SdFile root;
SdFile file;
////////////////////////////////////////////////////
//variaveis de tempo parcial
long tpV = 0, tpVmin = 0; 
long tpVsec = 0, tpVmil = 0, tpVhoras=0;
long tpV1=0, tpVdec=0;

//Variaveis de tempo total
long ttV = 0,ttVmin = 0; 
long ttVsec = 0, ttVmil = 0, ttVhoras=0;
long ttV1=0,ttVdec=0;

long tAntig = 0;
int count=0;

//variavel para definir o pino do led
int ledVermelho = 2;
int ledVerde = 8;
int ledAmarelo = 9;
//*****************************************************
void setup() {
  pinMode(selectSd,OUTPUT); //  Configura pino digital 3 como OUTPUT para conecta-lo ao pino RFID / ATIVAR
  pinMode(selectRfid,OUTPUT); // Definir pino digital 4 como OUTPUT para conecta-lo ao pino RFID / ATIVAR
  pinMode(NRSTPD,OUTPUT); // Definir pino digital 5 , Not Reset and Power-down
  pinMode(ledVermelho, OUTPUT);  //Definir pino digital do led vermelho 
  pinMode(ledAmarelo, OUTPUT); // Definir pino digital do led amarelo
  pinMode(ledVerde, OUTPUT); //Definir pino digital do led verde

  digitalWrite(ledAmarelo, LOW);
  digitalWrite(ledVerde, LOW);
  digitalWrite(ledVermelho, LOW);

  Serial.begin(9600); // para depuracao
  Serial.flush(); // precisa do flush serial buffer, caso contrario, leu pela primeira vez a partir de reset / power on pode nao estar correto

 digitalWrite(selectSd, HIGH);
 digitalWrite(selectRfid, HIGH);
 digitalWrite(NRSTPD, HIGH);

 // iniciar a biblioteca SPI:
 SPI.begin();

 // inicializar o i2c bus para DS1307 RTC
 Wire.begin();

   SelectOne();
   SD_Init();
   
   file.writeError = 0; 
         
   SelectOne();
  MFRC522_Init();
  SelectOne();
 
  digitalWrite(ledAmarelo, HIGH);   //o led acende
  delay(1000); //piscar durante esse tempo
  digitalWrite(ledAmarelo, LOW); //o led apaga
  
//second = 0;
//minute = 45;
//hour = 14;
//dayOfMonth = 31;
//month = 5;
//year = 14;
//setDateDs1307(second, minute, hour, dayOfMonth, month, year);
  
Start();
  }
//**********************************************************
void loop() {
uchar i,tmp;
uchar status;
uchar RC_size;
uchar blockAddr; //Selecione o endereco de bloco de operacao 0 a 63

SelectOne();
  
status = MFRC522_Request(PICC_REQIDL, str_tem); //Encontrar cartoes, devolva o tipo de cartao

//Anti-colisÃ£o, retornar o numero de serie 4 bytes do cartao
status = MFRC522_Anticoll(str_tem);
memcpy(serNum, str_tem, 5);

SelectOne();
 
if (count<3){  
  if (status == MI_OK) {    
     tpV = ( millis () - tAntig- ttV); //tempo parcial recebe a diferenca entre a funcao milis e tempo antigo menos o tempo total
     ttV = ttV + tpV; // tempo total acumula tempo total mais tempo parcial
     
     ttV1=ttV; //o tempo calculado e inserido nessa nova variavel
     tpV1=tpV; //o tempo calculado e inserido nessa nova variavel
      
      Data();//funcao que obtem a data e hora da leitura   
      writeString(file,","); //funcao que escreve uma string 
      CalcularParcial(); //funcao que calcula o tempo parcial
      writeString(file,","); //funcao que escreve uma string
      CalcularTotal(); //funcao que calcula o tempo total
      writeCRLF(file); //funcao que pula uma linha na escrita no cartÃ£o SD
      count=count+1; //soma mais um no valor count

      digitalWrite(ledVerde, HIGH);//o led acende
      delay(500); //piscar durante esse tempo
      digitalWrite(ledVerde, LOW); //o led apaga

     Serial.println(count);
    }
}
  
 else{
     Fechar(); //funcao para fechar o arquivo
      digitalWrite(ledVermelho, HIGH);   //o led acende
      delay(2000); //piscar durante esse tempo
      digitalWrite(ledVermelho, LOW); //o led apaga
 }     
}
//******************************************************
//FUNCOES DO SOFTWARE

/*
* Nome da funcao: Error_P
* SD mensagem de erro.
* Valor de retorno: Nenhum
*/
void Error_P(const char *str) {
	PgmPrint("error: ");
	SerialPrintln_P(str);
	if (card.errorCode()) {
		PgmPrint("SD error: ");
		Serial.print(card.errorCode(), HEX);
		Serial.print(',');
		Serial.println(card.errorData(), HEX);
	}
	while(1);
 }
//-----------------------
//Ativa o cartao SD e modulo RFID de uma so vez
void SelectOne() {
	digitalWrite(selectSd, HIGH);
	digitalWrite(selectRfid, HIGH);
}
//------------------------
//Inicializa o leitor de cartao SD para ler e escrever

void SD_Init() {
 // Inicializar o cartao SD no SPI_HALF_SPEED para evitar erros de bus com breadboards.
// Use SPI_FULL_SPEED para um melhor desempenho.

if (!card.init(SPI_HALF_SPEED,selectSd)) error("card.init failed"); //definir o selectSd como SS, de modo que voce seleciona o pino certo de SD
    // initialize o volume FAT
    if (!volume.init(&card)) error("volume.init failed");
        // Abrir diretorio do root
          if (!root.openRoot(&volume)) error("openRoot failed");
            // criar um novo arquivo
               char name[] = "LOGGER00.TXT";
               for (uint8_t i = 0; i < 100; i++) {
                   name[6] = i/10 + '0';
                   name[7] = i%10 + '0';
                   if (file.open(&root, name, O_CREAT | O_EXCL | O_WRITE)) break;
                }
       Serial.print("Logging to: ");
       Serial.println(name); //mostrar nome do arquivo
	}
//------------------------------
/*
* Escrever CR LF to um arquivo
*/
 void writeCRLF(SdFile& f) {
    f.write((uint8_t*)"\r\n", 2);
}
//----------------------------------
/*
* Escrever um (unsigned) numero para um arquivo
*/
void writeNumber(SdFile& f, uint32_t n) {
	uint8_t buf[10];
	uint8_t i = 0;
	do {
		i++;
		buf[sizeof(buf) - i] = n%10 + '0';
		n /= 10;
		} while (n);
			f.write(&buf[sizeof(buf) - i], i);
}
//-------------------------------------
/*
* Escreva uma string para o arquivo
*/
void writeString(SdFile& f, char *str) {
	uint8_t n;
	for (n = 0; str[n]; n++);
		f.write((uint8_t *)str, n);
}

//----------------------------------------
/*
* Nome da funÃ§Ã£o: Write_MFRC5200
* DescriÃ§Ã£o: MFRC522 um registro para escrever um byte de dados
* Entrada: - registro de endereÃ§o addr; val - valor a ser escrito
* Valor de retorno: Nenhum
*/
void Write_MFRC522(uchar addr, uchar val){
	digitalWrite(selectRfid, LOW);
	//Endereco Formatado: 0XXXXXX0
		SPI.transfer((addr<<1)&0x7E);
		SPI.transfer(val);
		digitalWrite(selectRfid, HIGH);
}
//-------------------------------------------------------
/*
* Nome da funÃ§Ã£o: Read_MFRC522
* DescriÃ§Ã£o: Leia um byte de dados a partir de um registo MFRC522
* ParÃ¢metros de entrada: addr - registro de endereÃ§o
* Valor de retorno: Retorna um byte de leitura de dados
*/

uchar Read_MFRC522(uchar addr){
	uchar val;
	digitalWrite(selectRfid, LOW);
	//Endereco Formatado: 1XXXXXX0
	SPI.transfer(((addr<<1)&0x7E) | 0x80);
	val =SPI.transfer(0x00);
	digitalWrite(selectRfid, HIGH);
	return val;
}
//-------------------------------------------------------
/*
* Nome da funÃ§Ã£o: SetBitMask
* DescriÃ§Ã£o: bits do registro Set RC522
* ParÃ¢metros de entrada: reg - registro de endereÃ§o; mÃ¡scara - valor definido
* Valor de retorno: Nenhum
*/

void SetBitMask(uchar reg, uchar mask){
uchar tmp;
tmp = Read_MFRC522(reg);
Write_MFRC522(reg, tmp | mask); // set bit mask
}
//--------------------------------------------------
/*
* Nome da funÃ§Ã£o: ClearBitMask
* DescriÃ§Ã£o: Limpar registo RC522 dos bits
* ParÃ¢metros de entrada: reg - registro de endereÃ§o; mÃ¡scara - valor evidente dos bits
* Valor de retorno: Nenhum
*/

void ClearBitMask(uchar reg, uchar mask){
	uchar tmp;
	tmp = Read_MFRC522(reg);
	Write_MFRC522(reg, tmp & (~mask)); // clear bit mask
}
//---------------------------------------------------
/*
* Nome da funÃ§Ã£o: AntennaOn
* DescriÃ§Ã£o: antena abertas, cada vez que vocÃª iniciar ou desligar a barreira natural entre o transmissor deve ser de pelo menos 1 ms intervalo
* Entrada: Nenhuma
* Valor de retorno: Nenhum
*/
void AntennaOn(void){
uchar temp;
temp = Read_MFRC522(TxControlReg);
if (!(temp & 0x03))
{
   SetBitMask(TxControlReg, 0x03);
}
}
//---------------------------------------------------
/*
* Nome da funÃ§Ã£o: AntennaOff
* DescriÃ§Ã£o: Fechar antena, cada vez que vocÃª iniciar ou desligar a barreira natural entre o transmissor deve ser de pelo menos 1 ms intervalo
* Entrada: Nenhuma
* Valor de retorno: Nenhum
*/

void AntennaOff(void){
ClearBitMask(TxControlReg, 0x03);
}
//-----------------------------------------------------
/*
* Nome da funÃ§Ã£o: ResetMFRC522
* DescriÃ§Ã£o: Reiniciar RC522
* Entrada: Nenhuma
* Valor de retorno: Nenhum
*/

void MFRC522_Reset(void){
  Write_MFRC522(CommandReg, PCD_RESETPHASE);
}
//-----------------------------------------------------
/*
* Nome da funÃ§Ã£o: InitMFRC522
* DescriÃ§Ã£o: Inicializar RC522
* Entrada: Nenhuma
* Valor de retorno: Nenhum
*/

void MFRC522_Init(void){
 digitalWrite(NRSTPD,HIGH);
 MFRC522_Reset();
//Timer: TPrescaler*TreloadVal/6.78MHz = 24ms

Write_MFRC522(TModeReg, 0x8D); //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
Write_MFRC522(TPrescalerReg, 0x3E); //TModeReg[3..0] + TPrescalerReg
Write_MFRC522(TReloadRegL, 30);
Write_MFRC522(TReloadRegH, 0);
Write_MFRC522(TxAutoReg, 0x40); //100%ASK
Write_MFRC522(ModeReg, 0x3D); //Valor inicial CRC de 0x6363??

//ClearBitMask(Status2Reg, 0x08); //MFCrypto1On=0
//Write_MFRC522(RxSelReg, 0x86); //RxWait = RxSelReg[5..0]
//Write_MFRC522(RFCfgReg, 0x7F); //RxGain = 48dB

AntennaOn(); //Antena aberta
}
//----------------------------------------
/*
* Nome da funÃ§Ã£o: MFRC522_Request
* DescriÃ§Ã£o: Encontre cartÃµes, ler o nÃºmero do tipo de cartÃ£o
* ParÃ¢metros de entrada: reqMode - encontrar cartÃµes de forma
* TagType - Tipo de CartÃ£o de Retorno
* 0x4400 = Mifare_UltraLight
* = 0x0400 Mifare_One (S50)
* 0x0200 = Mifare_One (S70)
* = 0x0800 Mifare_Pro (X)
* 0x4403 = Mifare_DESFire
* Valor de retorno: o retorno bem sucedido MI_OK
*/

uchar MFRC522_Request(uchar reqMode, uchar *TagType){
uchar status;
uint backBits; //Os bits dos dados recebidos
Write_MFRC522(BitFramingReg, 0x07); //TxLastBists = BitFramingReg[2..0] ???
TagType[0] = reqMode;
status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);
if ((status != MI_OK) || (backBits != 0x10))
{
status = MI_ERR;
}
return status;
}
//--------------------------------------------------
/*
* Nome da funÃ§Ã£o: MFRC522_ToCard
* DescriÃ§Ã£o: RC522 e cartÃ£o ISO14443 comunicaÃ§Ã£o
* ParÃ¢metros de entrada: comando - palavra de comando MF522,
* SendData - dados enviados ao cartÃ£o via RC522,
* SendLen - comprimento de dados enviados
* Dados retrospectivos - receber dados de retorno para o cartÃ£o,
* BackLen - retorno comprimento de bits de dados
* Valor de retorno: o retorno bem sucedido MI_OK
*/

uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData,uint *backLen){
uchar status = MI_ERR;
uchar irqEn = 0x00;
uchar waitIRq = 0x00;
uchar lastBits;
uchar n;
uint i;
switch (command)
{
case PCD_AUTHENT: //CartÃµes de CertificaÃ§Ã£o Fechar
{
irqEn = 0x12;
waitIRq = 0x10;
break;
}
case PCD_TRANSCEIVE: //FIFO de transmissÃ£o de dados
{
irqEn = 0x77;
waitIRq = 0x30;
break;
}
default:
break;
}
Write_MFRC522(CommIEnReg, irqEn|0x80); //Pedido de interrupÃ§Ã£o
ClearBitMask(CommIrqReg, 0x80); //Limpar bit pedido de interrupÃ§Ã£o
SetBitMask(FIFOLevelReg, 0x80); //FlushBuffer=1, FIFO InicializaÃ§Ã£o
Write_MFRC522(CommandReg, PCD_IDLE); //NO action;Cancelar o comando atual ???
//Escrevendo dados para o FIFO
for (i=0; i<sendLen; i++)
{
Write_MFRC522(FIFODataReg, sendData[i]);
}
//Executa o comando
Write_MFRC522(CommandReg, command);
if (command == PCD_TRANSCEIVE)
{
SetBitMask(BitFramingReg, 0x80); //StartSend=1,transmission of data starts
}
// Aguardar a conclusÃ£o de receber dados
i = 2000; // i de acordo com o ajuste de freqÃ¼Ãªncia do relÃ³gio, cartÃ£o mÃ¡ximo operaÃ§Ã£o M1 espera 25ms de tempo??
do
{
//CommIrqReg[7..0]
//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
n = Read_MFRC522(CommIrqReg);
i--;
}
while ((i!=0) && !(n&0x01) && !(n&waitIRq));
ClearBitMask(BitFramingReg, 0x80); //StartSend=0
if (i != 0)
{
if(!(Read_MFRC522(ErrorReg) & 0x1B)) //BufferOvfl Collerr CRCErr ProtecolErr
{
status = MI_OK;
if (n & irqEn & 0x01)
{
status = MI_NOTAGERR; //??
}
if (command == PCD_TRANSCEIVE)
{
n = Read_MFRC522(FIFOLevelReg);
lastBits = Read_MFRC522(ControlReg) & 0x07;
if (lastBits)
{
*backLen = (n-1)*8 + lastBits;
}
else
{
*backLen = n*8;
}
if (n == 0)
{
n = 1;
}
if (n > MAX_LEN)
{
n = MAX_LEN;
}
// LÃª os dados recebidos do FIFO
for (i=0; i<n; i++)
{
backData[i] = Read_MFRC522(FIFODataReg);
}
}
}
else
{
status = MI_ERR;
}
}
//SetBitMask(ControlReg,0x80); //timer stops
//Write_MFRC522(CommandReg, PCD_IDLE);
return status;
}
//------------------------------------------------
/*
* Nome da funÃ§Ã£o: MFRC522_Anticoll
* DescriÃ§Ã£o: DetecÃ§Ã£o de Anti-colisÃ£o, selecione o cartÃ£o lÃª cartÃ£o de nÃºmero de sÃ©rie
* ParÃ¢metros de entrada: serNum - Retorna um nÃºmero de sÃ©rie do cartÃ£o de 4 bytes, o quinto byte byte de checksum
* Valor de retorno: o retorno bem sucedido MI_OK
*/
uchar MFRC522_Anticoll(uchar *serNum)
{
uchar status;
uchar i;
uchar serNumCheck=0;
uint unLen;
//ClearBitMask(Status2Reg, 0x08); //TempSensclear
//ClearBitMask(CollReg,0x80); //ValuesAfterColl
Write_MFRC522(BitFramingReg, 0x00); //TxLastBists = BitFramingReg[2..0]
serNum[0] = PICC_ANTICOLL;
serNum[1] = 0x20;
status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);
if (status == MI_OK)
{
//Verifique o nÃºmero de sÃ©rie do cartÃ£o
for (i=0; i<4; i++)
{
serNumCheck ^= serNum[i];
}
if (serNumCheck != serNum[i])
{
status = MI_ERR;
}
}
//SetBitMask(CollReg, 0x80); //ValuesAfterColl=1
return status;
}
//----------------------------------------------------------------------------------
void Fechar(){  
if(file.writeError) {error("The file is Close");}

// Fecha o arquivo e forÃ§a de gravaÃ§Ã£o de todos os dados para o cartÃ£o SD
 file.close();
 Serial.println("Write to SD card , Done");
 SelectOne();
 }
//-----------------------------------------------------------------------------------------
void CalcularTotal(){
  //calculo do tempo total
  ttVmil = ttV1 % 1000; //ExtraÃƒÂ­mos os milisegundos do total 
  ttV1 = ttV1 / 1000; //Convertemos o total para segundos  
  ttVsec = ttV1 % 60; //ExtraÃƒÂ­mos os segundos do total  
  ttV1 /= 60;   //Convertemos o total para minutos  
  ttVmin = ttV1 % 60;  //ExtraÃƒÂ­mos os minutos em 1 hora  
  ttV1 /= 60;   //Convertemos o total para horas 
  ttVhoras = ttV1; 
 
  writeNumber(file,(ttVhoras));
  writeString(file,":");
  writeNumber(file,(ttVmin)); 
  writeString(file,":");
  writeNumber(file,(ttVsec));
}
//-----------------------------------------------------------------------------------------
void CalcularParcial(){
   //calculo do tempo parcial
    tpVmil = tpV1 % 1000; //ExtraÃƒÂ­mos os milisegundos do total 
    tpV1 = tpV1 / 1000; //Convertemos o total para segundos  
    tpVsec = tpV1 % 60; //ExtraÃƒÂ­mos os segundos do total  
    tpV1 /= 60;   //Convertemos o total para minutos  
    tpVmin = tpV1 % 60;  //ExtraÃƒÂ­mos os minutos em 1 hora  
    tpV1 /= 60;   //Convertemos o total para horas 
    tpVhoras = tpV1; 
                     
     writeNumber(file, (tpVhoras)); 
     writeString(file,":");
     writeNumber(file,(tpVmin)); 
     writeString(file,":");
     writeNumber(file,(tpVsec));
   }
//--------------------------------------------------------------------------------------------
void Start(){
 tAntig = millis(); //iniciar milis
}
//---------------------------------------------------------------
void setDateDs1307(byte second, // 0-59
byte minute, // 0-59
byte hour, // 1-23
byte dayOfMonth, // 1-28/29/30/31
byte month, // 1-12
byte year) // 0-99
{
Wire.beginTransmission(DS1307_I2C_ADDRESS);
Wire.write(0);
Wire.write(decToBcd(second)); // 0 to bit 7 starts the clock
Wire.write(decToBcd(minute));
Wire.write(decToBcd(hour)); // If you want 12 hour am/pm you need to set
// bit 6 (also need to change readDateDs1307)
Wire.write(decToBcd(dayOfMonth));
Wire.write(decToBcd(month));
Wire.write(decToBcd(year));
Wire.endTransmission();
}
// Gets the date and time from the ds1307
void getDateDs1307(byte *second,
byte *minute,
byte *hour,
byte *dayOfMonth,
byte *month,
byte *year)
{
// Reset the register pointer
Wire.beginTransmission(DS1307_I2C_ADDRESS);
Wire.write(0);
Wire.endTransmission();
Wire.requestFrom(DS1307_I2C_ADDRESS, 7);
// A few of these need masks because certain bits are control bits
*second = bcdToDec(Wire.read() & 0x7f);
*minute = bcdToDec(Wire.read());
*hour = bcdToDec(Wire.read() & 0x3f); // Need to change this if 12 hour am/pm
*dayOfMonth = bcdToDec(Wire.read());
*month = bcdToDec(Wire.read());
*year = bcdToDec(Wire.read());
}
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
return ( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
return ( (val/16*10) + (val%16) );
}
//------------------------------------------------------
void Data(){
      getDateDs1307(&second, &minute, &hour, &dayOfMonth, &month, &year);
      writeNumber(file, (dayOfMonth,DEC));  
      writeString(file, "-");
      writeNumber(file, (month,DEC));
      writeString(file, "-");
      writeNumber(file, (year,DEC));
      writeString(file, ",");
      writeNumber(file,(hour,DEC));
      writeString(file, ":");
      writeNumber(file, (minute,DEC));
      writeString(file, ":");
      writeNumber(file, (second,DEC));
   }


