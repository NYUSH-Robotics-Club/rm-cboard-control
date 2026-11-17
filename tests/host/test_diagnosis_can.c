#include "diagnosis_can.h"
#include "can.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static MockRegisters regs[2];
CAN_HandleTypeDef hcan1={.Instance=&regs[0]}, hcan2={.Instance=&regs[1]};
static uint32_t hw_state[2], now, available, hw_error, init_error, aborts, selected_mailbox=1;
static CAN_FilterTypeDef filters[2];
static CAN_TxHeaderTypeDef sent;
static uint8_t sent_data[8];
static unsigned index_of(CAN_HandleTypeDef *h) { return h==&hcan1?0:1; }
uint32_t HAL_CAN_GetState(CAN_HandleTypeDef *h) { return hw_state[index_of(h)]; }
uint32_t HAL_CAN_Stop(CAN_HandleTypeDef *h) { hw_state[index_of(h)]=HAL_CAN_STATE_READY; return HAL_OK; }
uint32_t HAL_CAN_Init(CAN_HandleTypeDef *h) { (void)h; return init_error; }
uint32_t HAL_CAN_Start(CAN_HandleTypeDef *h) { hw_state[index_of(h)]=HAL_CAN_STATE_LISTENING; return HAL_OK; }
uint32_t HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f) { filters[index_of(h)]=*f; return HAL_OK; }
uint32_t HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t flags) { (void)h; (void)flags; return HAL_OK; }
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo) { (void)h; (void)fifo; return available; }
uint32_t HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo, CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; (void)fifo;
    available--;
    *header=(CAN_RxHeaderTypeDef){.ExtId=0x12345,.IDE=CAN_ID_EXT,.DLC=8};
    memset(data, 0xa5, 8);
    return HAL_OK;
}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *h) { (void)h; return hw_error; }
uint32_t HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *header, uint8_t *data, uint32_t *mailbox)
{
    (void)h; sent=*header; memcpy(sent_data,data,8); *mailbox=selected_mailbox; return HAL_OK;
}
uint32_t HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mailbox) { (void)h; (void)mailbox; aborts++; return HAL_OK; }
void HAL_NVIC_DisableIRQ(int irq) { (void)irq; }
void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }
uint32_t HAL_GetTick(void) { return now; }
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h);
extern void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h);
int main(void)
{
    DiagnosisBus bus;
    DiagnosisEvent event;
    assert(DiagnosisCan_Configure(1,1000000,DIAG_SILENT));
    assert(hcan1.Init.Prescaler==3 && hcan1.Init.AutoRetransmission==DISABLE);
    assert(filters[0].FilterBank==0 && filters[0].SlaveStartFilterBank==14);
    assert(DiagnosisCan_Configure(2,500000,DIAG_LOOPBACK));
    assert(hcan2.Init.Prescaler==6 && hcan2.Init.Mode==CAN_MODE_SILENT_LOOPBACK);
    assert(filters[1].FilterBank==14);
    assert(!DiagnosisCan_Configure(0,1000000,DIAG_NORMAL));
    assert(!DiagnosisCan_Configure(1,2000000,DIAG_NORMAL));
    DiagnosisRequest request={.seq=17,.channel=1,.id=0x123,.length=4,.data={0x08330001}};
    assert(DiagnosisCan_Send(&request,1)==DIAG_LISTEN_ONLY);
    assert(DiagnosisCan_Configure(1,1000000,DIAG_NORMAL));
    assert(DiagnosisCan_Send(&request,1)==DIAG_OK);
    assert(sent.StdId==0x123 && sent.DLC==4 && sent_data[2]==0x33);
    assert(!DiagnosisCan_Poll(&event,2)); /* Queued is not completed. */
    assert(DiagnosisCan_Send(&request,2)==DIAG_BUSY);
    assert(!DiagnosisCan_Configure(1,500000,DIAG_NORMAL));
    regs[0].TSR=CAN_TSR_RQCP0|CAN_TSR_TXOK0;
    assert(DiagnosisCan_Poll(&event,3) && event.kind==DIAG_TX_OK && event.token==17);
    DiagnosisCan_Snapshot(1,&bus);
    assert(bus.tx_ok==1 && bus.pending==0 && bus.last_tx_tsr==3);
    regs[0].TSR=0; /* Model hardware's W1C completion semantics. */
    selected_mailbox=CAN_TX_MAILBOX2;
    assert(DiagnosisCan_Send(&request,4)==DIAG_OK);
    regs[0].TSR=CAN_TSR_RQCP0<<16; /* Failed frame on mailbox 2. */
    assert(DiagnosisCan_Poll(&event,5) && event.kind==DIAG_TX_ERROR);
    regs[0].TSR=0;
    assert(DiagnosisCan_Send(&request,0xfffffff0U)==DIAG_OK);
    assert(!DiagnosisCan_Poll(&event,0x20));
    assert(DiagnosisCan_Poll(&event,0x22) && event.kind==DIAG_TX_TIMEOUT);
    assert(aborts==1 && !DiagnosisCan_Poll(&event,0x23));
    assert(DiagnosisCan_Send(&request,0x24)==DIAG_BUSY);
    regs[0].TSR=(CAN_TSR_RQCP0|CAN_TSR_TXOK0)<<16; /* Abort raced TX success. */
    assert(DiagnosisCan_Poll(&event,0x25) && event.kind==DIAG_TX_OK);
    regs[0].TSR=0;
    regs[0].ESR=CAN_ESR_BOFF;
    assert(DiagnosisCan_Send(&request,100)==DIAG_HARDWARE);
    regs[0].ESR=0;
    now=200; available=70;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
    DiagnosisCan_Snapshot(2,&bus);
    assert(bus.rx_frames==70 && bus.rx_dropped==6 && bus.rx_callbacks==1 && bus.last_rx_ms==200);
    for(unsigned i=0;i<64;i++) {
        assert(DiagnosisCan_Poll(&event,201));
        assert(event.channel==2 && event.kind==DIAG_RX && event.flags==DIAG_EXTENDED);
        assert(event.id==0x12345 && event.data[1]==0xa5a5a5a5 && event.ms==200);
    }
    assert(!DiagnosisCan_Poll(&event,201));
    hw_error=HAL_CAN_ERROR_RX_FOV0;
    HAL_CAN_ErrorCallback(&hcan2);
    DiagnosisCan_Snapshot(2,&bus); assert(bus.rx_dropped==7);
    init_error=1;
    assert(!DiagnosisCan_Configure(2,125000,DIAG_NORMAL));
    DiagnosisCan_Snapshot(2,&bus); assert(!bus.ready);
    puts("diagnosis CAN: PASS");
}
