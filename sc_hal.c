/*
 * SC.c
 * SC���ж��Э�����
 * ����: 2022.3.29
 * ����:
 * Ǩ�Ƶ� STM32L4xx HAL ��
 */

#include <stdlib.h>
#include "stm32l4xx_hal.h" // Include the HAL library header for STM32L4 series
#include "INST.h"          // Include the instruction set header
#include "SC.h"            // Include SC library header
#include "SCSerail.h"      // Include the modified serial communication header

static uint8_t Level =1;//������صȼ�
static uint8_t End = 0;//��������С�˽ṹ
//static uint8_t Error = 0;//���״̬
uint8_t syncReadRxPacketIndex;
uint8_t syncReadRxPacketLen;
uint8_t *syncReadRxPacket;
uint8_t *syncReadRxBuff;
uint16_t syncReadRxBuffLen;
uint16_t syncReadRxBuffMax;
uint32_t syncTimeOut;

//1��16λ�����Ϊ2��8λ��
//DataLΪ��λ��DataHΪ��λ
void Host2SC(uint8_t *DataL, uint8_t* DataH, int Data)
{
	if(End){
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

//2��8λ�����Ϊ1��16λ��
//DataLΪ��λ��DataHΪ��λ
int SC2Host(uint8_t DataL, uint8_t DataH)
{
	int Data;
	if(End){
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}

void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
	uint8_t i;
	uint8_t msgLen = 2;
	uint8_t bBuf[6];
	uint8_t CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if(nDat){
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		writeSC(bBuf, 6);

	}else{
		bBuf[3] = msgLen;
		writeSC(bBuf, 5);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	if(nDat){
		for(i=0; i<nLen; i++){
			CheckSum += nDat[i];
		}
		writeSC(nDat, nLen);
	}
	CheckSum = ~CheckSum;
	writeSC(&CheckSum, 1);
}

//��ͨдָ��
//���ID��MemAddr�ڴ���ַ��д�����ݣ�д�볤��
int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSC();
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
	wFlushSC();
	return Ack(ID);
}

//�첽дָ��
//���ID��MemAddr�ڴ���ַ��д�����ݣ�д�볤��
int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSC();
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	wFlushSC();
	return Ack(ID);
}

//�첽дִ����
int regAction(uint8_t ID)
{
	rFlushSC();
	writeBuf(ID, 0, NULL, 0, INST_REG_ACTION);
	wFlushSC();
	return Ack(ID);
}

//ͬ��дָ��
//���ID[]���飬IDN���鳤�ȣ�MemAddr�ڴ���ַ��д�����ݣ�д�볤��
void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	uint8_t mesLen = ((nLen+1)*IDN+4);
	uint8_t Sum = 0;
	uint8_t bBuf[7];
	uint8_t i, j;

	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;

	rFlushSC();
	writeSC(bBuf, 7);

	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;

	for(i=0; i<IDN; i++){
		writeSC(&ID[i], 1);
		writeSC(nDat+i*nLen, nLen);
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	Sum = ~Sum;
	writeSC(&Sum, 1);
	wFlushSC();
}

int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
	rFlushSC();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	wFlushSC();
	return Ack(ID);
}

int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat)
{
	uint8_t buf[2];
	Host2SC(buf+0, buf+1, wDat);
	rFlushSC();
	writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
	wFlushSC();
	return Ack(ID);
}

//��ָ��
//���ID��MemAddr�ڴ���ַ����������nData�����ݳ���nLen
int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen)
{
	int Size;
	uint8_t bBuf[4];
	uint8_t calSum;
	uint8_t i;
	rFlushSC();
	writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
	wFlushSC();
	if(!checkHead()){
		return 0;
	}
	//Error = 0; // Error handling is done by return values
	Size = readSC(bBuf, 3);
    if(Size != 3){
        return 0;
    }
	Size = readSC(nData, nLen);
	if(Size!=nLen){
		return 0;
	}
	Size = readSC(bBuf+3, 1);
    if(Size != 1){
        return 0;
    }
	calSum = bBuf[0]+bBuf[1]+bBuf[2];
	for(i=0; i<nLen; i++){ // Use nLen for loop limit
		calSum += nData[i];
	}
	calSum = ~calSum;
	if(calSum!=bBuf[3]){
		return 0;
	}
	//Error = bBuf[2]; // Error status can be handled externally if needed
	return nLen; // Return the expected length if successful
}

//��1�ֽڣ���ʱ����-1
int readByte(uint8_t ID, uint8_t MemAddr)
{
	uint8_t bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

//��2�ֽڣ���ʱ����-1
int readWord(uint8_t ID, uint8_t MemAddr)
{
	uint8_t nDat[2];
	int Size;
	uint16_t wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
	wDat = SC2Host(nDat[0], nDat[1]);
	return wDat;
}

//Pingָ����ض��ID����ʱ����-1
int	Ping(uint8_t ID)
{
	uint8_t bBuf[4];
	uint8_t calSum;
	rFlushSC();
	writeBuf(ID, 0, NULL, 0, INST_PING);
	wFlushSC();
	//Error = 0; // Error handling is done by return values
	if(!checkHead()){
		return -1;
	}

	if(readSC(bBuf, 4)!=4){
		return -1;
	}
	if(bBuf[0]!=ID && ID!=0xfe){
		return -1;
	}
	if(bBuf[1]!=2){
		return -1;
	}
	calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		return -1;
	}
	//Error = bBuf[2]; // Error status can be handled externally if needed
	return bBuf[0];
}

int checkHead(void)
{
	uint8_t bDat;
	uint8_t bBuf[2] = {0, 0};
	uint8_t Cnt = 0;
    // Use a timeout for waiting for the header
    uint32_t start_time = HAL_GetTick();
    uint32_t timeout_ms = 100; // Adjust timeout as needed

	while(1){
		// Try to read one byte with a small timeout
		if(readSC(&bDat, 1) != 1){
            // Check if overall timeout has occurred
            if((HAL_GetTick() - start_time) > timeout_ms){
                return 0; // Timeout waiting for header
            }
            continue; // No byte received, continue waiting
		}
		bBuf[1] = bBuf[0];
		bBuf[0] = bDat;
		if(bBuf[0]==0xff && bBuf[1]==0xff){
			break;
		}
		Cnt++;
		if(Cnt>10){ // Limit consecutive non-header bytes
			return 0;
		}
        start_time = HAL_GetTick(); // Reset timeout on receiving a byte
	}
	return 1;
}

//ָ��Ӧ��
int	Ack(uint8_t ID)
{
	uint8_t bBuf[4];
	uint8_t calSum;
	//Error = 0; // Error handling is done by return values
	if(ID!=0xfe && Level){
		if(!checkHead()){
			return 0;
		}
		if(readSC(bBuf, 4)!=4){
			return 0;
		}
		if(bBuf[0]!=ID){
			return 0;
		}
		if(bBuf[1]!=2){
			return 0;
		}
		calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
		if(calSum!=bBuf[3]){
			return 0;
		}
		//Error = bBuf[2]; // Error status can be handled externally if needed
	}
	return 1;
}

int	syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen)
{
	uint8_t checkSum;
	uint8_t i;
	rFlushSC();
	syncReadRxPacketLen = nLen;
	checkSum = (4+0xfe)+IDN+MemAddr+nLen+INST_SYNC_READ;
	writeByteSC(0xff);
	writeByteSC(0xff);
	writeByteSC(0xfe);
	writeByteSC(IDN+4);
	writeByteSC(INST_SYNC_READ);
	writeByteSC(MemAddr);
	writeByteSC(nLen);
	for(i=0; i<IDN; i++){
		writeByteSC(ID[i]);
		checkSum += ID[i];
	}
	checkSum = ~checkSum;
	writeByteSC(checkSum);
	wFlushSC();

	// Use readSCTimeOut with the configured syncTimeOut
	syncReadRxBuffLen = readSCTimeOut(syncReadRxBuff, syncReadRxBuffMax, syncTimeOut);
	return syncReadRxBuffLen;
}

void syncReadBegin(uint8_t IDN, uint8_t rxLen, uint32_t TimeOut)
{
	syncReadRxBuffMax = IDN*(rxLen+6);
	syncReadRxBuff = malloc(syncReadRxBuffMax);
	syncTimeOut = TimeOut; // Timeout is now in milliseconds for HAL_GetTick compatibility
}

void syncReadEnd(void)
{
	if(syncReadRxBuff){
		free(syncReadRxBuff);
		syncReadRxBuff = NULL;
	}
}

int syncReadPacketRx(uint8_t ID, uint8_t *nDat)
{
	uint16_t syncReadRxBuffIndex = 0;
	syncReadRxPacket = nDat;
	syncReadRxPacketIndex = 0;
	while((syncReadRxBuffIndex+6+syncReadRxPacketLen)<=syncReadRxBuffLen){
		uint8_t bBuf[] = {0, 0, 0};
		uint8_t calSum = 0;
		while(syncReadRxBuffIndex<syncReadRxBuffLen){
			bBuf[0] = bBuf[1];
			bBuf[1] = bBuf[2];
			bBuf[2] = syncReadRxBuff[syncReadRxBuffIndex++];
			if(bBuf[0]==0xff && bBuf[1]==0xff && bBuf[2]!=0xff){
				break;
			}
		}
		if(bBuf[2]!=ID){
			continue;
		}
		if(syncReadRxBuff[syncReadRxBuffIndex++]!=(syncReadRxPacketLen+2)){
			continue;
		}
		//Error = syncReadRxBuff[syncReadRxBuffIndex++]; // Error status can be handled externally
		calSum = ID+(syncReadRxPacketLen+2)+syncReadRxBuff[syncReadRxBuffIndex++];
		for(uint8_t i=0; i<syncReadRxPacketLen; i++){
			syncReadRxPacket[i] = syncReadRxBuff[syncReadRxBuffIndex++];
			calSum += syncReadRxPacket[i];
		}
		calSum = ~calSum;
		if(calSum!=syncReadRxBuff[syncReadRxBuffIndex++]){
			return 0;
		}
		return syncReadRxPacketLen;
	}
	return 0;
}

int syncReadRxPacketToByte(void)
{
	if(syncReadRxPacketIndex>=syncReadRxPacketLen){
		return -1;
	}
	return syncReadRxPacket[syncReadRxPacketIndex++];
}

int syncReadRxPacketToWrod(uint8_t negBit)
{
	if((syncReadRxPacketIndex+1)>=syncReadRxPacketLen){
		return -1;
	}
	int Word = SC2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex+1]);
	syncReadRxPacketIndex += 2;
	if(negBit){
		if(Word&(1<<negBit)){
			Word = -(Word & ~(1<<negBit));
		}
	}
	return Word;
}

