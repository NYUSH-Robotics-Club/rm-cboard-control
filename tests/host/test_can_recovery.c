/* Exercise fault entry, mailbox cancellation, bounded retries and operator rearming. */
#include "main.h"
#include "bsp_can.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static CAN_TypeDef regs[2];
CAN_HandleTypeDef hcan1={&regs[0]}, hcan2={&regs[1]};
static uint32_t now_ms, aborts, writes;
static int abort_completes=1;
static int tx_fails, rx_ready;
static unsigned chosen_mailbox;
static bool pending_model;
static uint32_t pending_masks[2];
static unsigned bus_index(const CAN_HandleTypeDef *h) { return h == &hcan1 ? 0U : 1U; }
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *h, uint32_t m)
{ return pending_model && (pending_masks[bus_index(h)] & m) != 0U; }
static void finish_mailbox(CAN_HandleTypeDef *h, unsigned i)
{
 pending_masks[bus_index(h)] &= ~(1U << i);
 h->Instance->TSR |= (3U << (8U*i)) | (1U << (26U+i));
 h->Instance->free_slots++;
}

uint32_t HAL_GetTick(void) { return now_ms; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h,CAN_FilterTypeDef *f)
{ (void)h; assert(f->SlaveStartFilterBank==14U); return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h) {h->Instance->free_slots=3;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h,uint32_t n) {(void)h;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h,uint32_t n,CAN_RxHeaderTypeDef *r,uint8_t *d)
{(void)h;(void)n;
 if (!rx_ready) return HAL_ERROR;
 rx_ready=0; *r=(CAN_RxHeaderTypeDef){.StdId=0x209,.DLC=8};
 const uint8_t feedback[8]={0,1,0xff,0xf4,0xfe,0xbf,0,0};
 memcpy(d,feedback,8);return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,CAN_TxHeaderTypeDef *t,uint8_t *d,uint32_t *m)
{(void)t;(void)d;
 if(tx_fails)return HAL_ERROR;
 *m=1U<<chosen_mailbox;writes++;
 if (pending_model) {
  assert(!(pending_masks[bus_index(h)] & *m));
  pending_masks[bus_index(h)] |= *m; h->Instance->free_slots--;
 }
 h->Instance->TSR &= ~((15U << (8U*chosen_mailbox)) | (1U << (26U+chosen_mailbox)));
 return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h,uint32_t m)
{
 if (!pending_model) assert(m==7U);
 aborts++;
 if(abort_completes) {
  if(pending_model) {
   for(unsigned i=0;i<3;++i) if((m & (1U<<i)) && (pending_masks[bus_index(h)] & (1U<<i)))
    finish_mailbox(h,i);
  } else h->Instance->free_slots=3;
 }
 return HAL_OK;
}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h){return h->Instance->free_slots;}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *h){(void)h;return 0;}
static void test_trace(void) {
 assert(BspCan_Start(BSP_CAN_CHANNEL_1)&&BspCan_Start(BSP_CAN_CHANNEL_2));
 BspCan_Service(0); assert(BspCan_TryArm(500));
 BspCan_TraceConfigure(BSP_CAN_CHANNEL_1,0x2fe,0x209,0);
 uint8_t command[8]={0xd1,0x20}; /* -12000 */
 MotorTraceBatch batch;
 for (unsigned i=0;i<3;++i) {
  chosen_mailbox=i; now_ms=1000+i*10;
  BspCan_TraceCommand(BSP_CAN_CHANNEL_1,0x2fe,0,-12000);
  assert(BspCan_Write(BSP_CAN_CHANNEL_1,0x2fe,command,8));
  now_ms+=4;
  /* Success, arbitration loss, transmit error, respectively. */
  regs[0].TSR=(1U|(2U<<i))<<(8*i);
  BspCan_TraceRead(&batch);
  assert(batch.channel==1 && batch.tx_id==0x2fe && batch.rx_id==0x209);
  assert(batch.count==3 && batch.lost==0);
  assert(batch.events[0].kind==MOTOR_TRACE_COMMAND && batch.events[0].raw==-12000);
  assert((batch.events[1].kind & 255U)==MOTOR_TRACE_SUBMIT && batch.events[1].detail==batch.events[0].seq);
  assert((batch.events[2].kind & 255U)==MOTOR_TRACE_TX_OK+i && batch.events[2].detail==batch.events[1].seq);
  assert(batch.events[2].ms-batch.events[1].ms==4);
  assert((batch.events[1].kind >> 8)==i+1 && (batch.events[2].kind >> 8)==i+1);
 }
 tx_fails=1;
 assert(!BspCan_Write(BSP_CAN_CHANNEL_1,0x2fe,command,8));
 BspCan_TraceRead(&batch); assert(batch.count==1 && batch.events[0].kind==MOTOR_TRACE_REJECT);
 tx_fails=0;
 /* An emptied mailbox with no result must not be reported as TXOK. */
 assert(BspCan_Write(BSP_CAN_CHANNEL_1,0x2fe,command,8));
 regs[0].TSR=1U<<(26U+chosen_mailbox);
 BspCan_TraceRead(&batch); assert(batch.count==2 && (batch.events[1].kind & 255U)==MOTOR_TRACE_TX_UNKNOWN);
 /* Reuse without a retained completion, including by a non-yaw frame, must
  * end the old association rather than attribute unrelated TXOK to yaw. */
 assert(BspCan_Write(BSP_CAN_CHANNEL_1,0x2fe,command,8));
 assert(BspCan_Write(BSP_CAN_CHANNEL_1,0x200,command,8));
 regs[0].TSR=3U<<(8U*chosen_mailbox);
 BspCan_TraceRead(&batch);
 assert(batch.count==2 && (batch.events[1].kind & 255U)==MOTOR_TRACE_TX_UNKNOWN);
 assert(batch.events[1].detail==batch.events[0].seq);
 BspCan_TraceRead(&batch); assert(batch.count==0);
 rx_ready=1; BspCanFrame frame;
 assert(BspCan_Read(BSP_CAN_CHANNEL_1,&frame));
 BspCan_TraceRead(&batch); assert(batch.count==0); /* BSP does not decode motor protocols. */
 BspCan_TraceFeedback(BSP_CAN_CHANNEL_1,0x209,1234,-321,(uint32_t)(int32_t)-12);
 BspCan_TraceRead(&batch);
 assert(batch.count==1 && batch.events[0].kind==MOTOR_TRACE_FEEDBACK);
 assert(batch.events[0].ms==1234);
 assert(batch.events[0].raw==-321 && (int32_t)batch.events[0].detail==-12);
 /* Wrong bus/slot does not contaminate yaw; bounded ring reports overwrite. */
 BspCan_TraceCommand(BSP_CAN_CHANNEL_2,0x2fe,0,123);
 BspCan_TraceCommand(BSP_CAN_CHANNEL_1,0x2fe,1,123);
 BspCan_TraceRead(&batch); assert(batch.count==0);
 for(unsigned i=0;i<300;++i) BspCan_TraceCommand(BSP_CAN_CHANNEL_1,0x2fe,0,(int16_t)i);
 BspCan_TraceRead(&batch); assert(batch.count==48 && batch.lost==44 && batch.events[0].raw==44);
 memset(regs,0,sizeof(regs)); now_ms=aborts=writes=chosen_mailbox=0;
 puts("CAN trace: PASS (all mailboxes, result linkage, reject, unknown, signed feedback, overflow)");
}

static void test_single_flight(void) {
 assert(BspCan_Start(BSP_CAN_CHANNEL_1)&&BspCan_Start(BSP_CAN_CHANNEL_2));
 BspCan_Service(0); assert(BspCan_TryArm(500));
 pending_model=true; memset(pending_masks,0,sizeof(pending_masks));
 BspCan_TraceConfigure(BSP_CAN_CHANNEL_1,0x2fe,0x209,0);
 uint8_t data[8]={1,2}, zero[8]={0};
 chosen_mailbox=0;
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_1,0x2fe,data,8)==BSP_CAN_TX_ACCEPTED);
 chosen_mailbox=1;
 unsigned before=writes;
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_1,0x2fe,data,8)==BSP_CAN_TX_BUSY);
 assert(writes==before && pending_masks[0]==1);
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_1,0x200,data,8)==BSP_CAN_TX_ACCEPTED);
 chosen_mailbox=2;
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_1,0x1ff,data,8)==BSP_CAN_TX_ACCEPTED);
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_1,0x2ff,data,8)==BSP_CAN_TX_BUSY);
 /* Same ID on another bus is independent. */
 chosen_mailbox=0;
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_2,0x2fe,data,8)==BSP_CAN_TX_ACCEPTED);
 /* CAN2 uses the same in-flight/retry stop handling after enabling retransmission. */
 before=writes;
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_2,0x2fe,data,8)==BSP_CAN_TX_BUSY);
 assert(writes==before);
 abort_completes=0; before=aborts;
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_2,0x2fe,zero,8)==BSP_CAN_TX_BUSY);
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_2,0x2fe,zero,8)==BSP_CAN_TX_BUSY);
 assert(aborts==before+1);
 finish_mailbox(&hcan2,0); abort_completes=1;
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_2,0x2fe,zero,8)==BSP_CAN_TX_ACCEPTED);
 finish_mailbox(&hcan1,0);
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_1,0x2fe,data,8)==BSP_CAN_TX_ACCEPTED);
 /* A stop aborts once, never overwrites an in-flight mailbox directly. */
 abort_completes=0; before=aborts;
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_1,0x2fe,zero,8)==BSP_CAN_TX_BUSY);
 assert(aborts==before+1);
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_1,0x2fe,zero,8)==BSP_CAN_TX_BUSY);
 assert(aborts==before+1);
 finish_mailbox(&hcan1,0); abort_completes=1;
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_1,0x2fe,zero,8)==BSP_CAN_TX_ACCEPTED);
 regs[0].ESR=CAN_ESR_BOFF;
 assert(BspCan_TryWrite(BSP_CAN_CHANNEL_1,0x2fe,data,8)==BSP_CAN_TX_ERROR);
 assert(!BspCan_OutputsArmed() && pending_masks[0]==0 && pending_masks[1]==0);
 MotorTraceBatch batch; BspCan_TraceRead(&batch);
 bool deferred=false;
 for(unsigned i=0;i<batch.count;++i) deferred |= (batch.events[i].kind & 255U)==MOTOR_TRACE_DEFER;
 assert(deferred);
 pending_model=false; memset(regs,0,sizeof(regs)); now_ms=aborts=writes=chosen_mailbox=0;
 puts("CAN single flight: PASS (ID/bus isolation, full mailboxes, completion, stop abort, fault cancellation)");
}

int main(void) {
 test_trace();
 test_single_flight();
 assert(BspCan_Start(BSP_CAN_CHANNEL_1)&&BspCan_Start(BSP_CAN_CHANNEL_2));
 uint8_t zero[8]={0}, command[8]={0,100};
 BspCan_Service(0);
 assert(!BspCan_TryArm(499));
 assert(!BspCan_Write(BSP_CAN_CHANNEL_1,0x200,command,8));
 assert(BspCan_Write(BSP_CAN_CHANNEL_1,0x200,zero,8));
 assert(BspCan_TryArm(500));
 assert(BspCan_Write(BSP_CAN_CHANNEL_1,0x200,command,8));
 regs[0].ESR=0x00FF0037; regs[0].free_slots=0; regs[1].free_slots=0;
 now_ms=600;
 assert(!BspCan_Write(BSP_CAN_CHANNEL_1,0x200,command,8));
 assert(!BspCan_OutputsArmed() && aborts==2 && regs[1].free_slots==3);
 const BspCanRecovery *s=BspCan_GetRecovery(BSP_CAN_CHANNEL_1);
 assert(s->fault_count==1 && s->first_fault_esr==0x00FF0037);
 BspCan_Service(601); assert(regs[0].MCR & CAN_MCR_INRQ);
 regs[0].MSR=CAN_MSR_INAK;
 BspCan_Service(602); assert(!(regs[0].MCR & CAN_MCR_INRQ));
 regs[0].MSR=0; BspCan_Service(603);
 assert(!BspCan_TryArm(2000)); /* elapsed time alone is never a recovery */
 regs[0].ESR=0; BspCan_Service(604);
 assert(s->recovery_count==1 && !BspCan_OutputsArmed());
 assert(!BspCan_TryArm(1103)); assert(BspCan_TryArm(1104));
 /* Abort stuck: never request initialization with old mailboxes pending. */
 abort_completes=0;regs[0].ESR=4;regs[0].free_slots=0;
 BspCan_Service(1200);BspCan_Service(1220);
 assert(s->phase==5 && s->timeout_count==1 && !(regs[0].MCR&1));
 assert(!BspCan_Write(BSP_CAN_CHANNEL_2,0x200,command,8));
 BspCan_Service(2219);assert(s->phase==5);
 abort_completes=1;BspCan_Service(2220);assert(s->phase==1);
 BspCan_Service(2221);assert(s->phase==2);
 assert(s->last_error_esr==0x00FF0037 && writes==2);
 puts("CAN recovery: PASS (real BSP, abort before init, bus sync, lock, timeout/retry)");
}
