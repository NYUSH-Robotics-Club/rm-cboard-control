/* Diagnostic-only yaw transport events. MCU milliseconds wrap naturally.
 * Result time is observation time (polling), NOT an exact wire timestamp.
 * seq identifies an event; detail links submit to command and result to submit.
 */
#ifndef MOTOR_TRACE_H
#define MOTOR_TRACE_H
#include <stdint.h>
#define MOTOR_TRACE_BATCH 48U
enum { MOTOR_TRACE_COMMAND=1, MOTOR_TRACE_SUBMIT, MOTOR_TRACE_TX_OK,
       MOTOR_TRACE_TX_ARB_LOST, MOTOR_TRACE_TX_ERROR, MOTOR_TRACE_FEEDBACK,
       MOTOR_TRACE_REJECT, MOTOR_TRACE_TX_UNKNOWN, MOTOR_TRACE_DEFER };
typedef struct {
    uint32_t ms, seq, kind; /* kind low8=event; bits8..9=mailbox+1, 0=not applicable */
    int32_t raw; /* command/current counts; result events carry raw TSR bits */
    uint32_t detail; /* linked seq; feedback: signed RPM reinterpreted as uint32 */
} MotorTraceEvent;
typedef struct {
    uint32_t channel, tx_id, rx_id, slot, count, lost;
    uint32_t esr, tsr, hal_error, free_mailboxes; /* bus snapshot at batch drain */
    MotorTraceEvent events[MOTOR_TRACE_BATCH];
} MotorTraceBatch;
#endif
