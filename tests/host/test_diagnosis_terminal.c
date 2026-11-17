#include "diagnosis_terminal.h"
#include "diagnosis.h"
#include "diagnosis_rtt.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

volatile DiagnosisShared g_diagnosis;
static char incoming[4096], output[8192];
static size_t read_at, input_size, output_size;
static bool blocked;
void DiagnosisRtt_Init(void) { }
size_t DiagnosisRtt_Read(void *data, size_t length)
{
    (void)length;
    if (read_at==input_size) return 0;
    *(char *)data=incoming[read_at++];return 1;
}
bool DiagnosisRtt_Write(const void *data, size_t length)
{
    if(blocked)return false;
    assert(output_size+length<sizeof(output));
    memcpy(output+output_size,data,length);output_size+=length;output[output_size]=0;return true;
}
void Diagnosis_Step(uint32_t now)
{
    g_diagnosis.uptime_ms=now;
    if(g_diagnosis.request.seq){g_diagnosis.ack=g_diagnosis.request.seq;g_diagnosis.result=DIAG_OK;}
}
static void feed(const char *line)
{
    input_size=strlen(line);read_at=0;memcpy(incoming,line,input_size);
    output_size=0;output[0]=0;
    DiagnosisTerminal_Step(100);
}
int main(void)
{
    DiagnosisTerminal_Init();
    feed("hello 1\n");assert(strstr(output,"\"protocol\":2"));
    feed("read 2 1 1 7\n");
    assert(g_diagnosis.request.op==DIAG_SEND && g_diagnosis.request.id==0x7ff);
    assert(g_diagnosis.request.length==8 && g_diagnosis.request.data[0]==0x07330001);
    assert(strstr(output,"\"token\":2,\"result\":0"));
    feed("send 3 1 0x123 0 0102FF\n");
    assert(g_diagnosis.request.id==0x123 && g_diagnosis.request.length==3 && g_diagnosis.request.data[0]==0xff0201);
    feed("send 4 1 0x123 0 -\n");assert(g_diagnosis.request.length==0);
    const char *bad[]={"send 5\n","send 5 1 1 0 FF extra\n","send 5 1 1 0 F\n","read 5 1 -1 7\n","read 5 1 1 256\n","hello 0\n","send 5 1 1 0 GG\n","unknown 5 1 2 3 4 5 6\n"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++){
        feed(bad[i]);assert(strstr(output,"\"result\":1"));assert(g_diagnosis.request.seq==4);
    }
    feed("config 6 1 1000000 1");assert(g_diagnosis.request.seq==4);
    input_size=1;read_at=0;incoming[0]='\n';DiagnosisTerminal_Step(101);
    assert(g_diagnosis.request.seq==6 && g_diagnosis.request.op==DIAG_CONFIGURE);
    blocked=true;feed("status 7\n");assert(output_size==0);
    blocked=false;DiagnosisTerminal_Step(102);assert(strstr(output,"\"type\":\"status\""));
    memset(incoming,'x',300);incoming[300]='\n';read_at=0;input_size=301;output_size=0;output[0]=0;
    DiagnosisTerminal_Step(103);DiagnosisTerminal_Step(104);
    assert(strstr(output,"\"result\":1"));
    feed("hello 8\n");assert(strstr(output,"\"protocol\":2"));
    g_diagnosis.events[0]=(DiagnosisEvent){.seq=1,.kind=DIAG_RX,.channel=1,.id=0};
    g_diagnosis.event_head=1;output_size=0;output[0]=0;
    DiagnosisTerminal_Step(105);assert(strstr(output,"\"type\":\"event\""));
    puts("diagnosis RTT terminal: PASS");
}
