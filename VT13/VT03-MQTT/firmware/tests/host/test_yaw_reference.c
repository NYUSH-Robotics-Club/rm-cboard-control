/* 验证按真实时间生成目标、锁位、跨零/多圈连续性及反馈失效，不模拟机械稳定性。 */
#include "yaw_reference.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float a, float b) { assert(fabsf(a-b)<0.2f); }

int main(void) {
    YawReference r;
    float dt;
    /* 总时长相同，1/4/5/20ms调用都应产生3度目标增量。 */
    unsigned periods[]={1,4,5,20};
    for(unsigned j=0;j<4;j++) {
        YawReference_Seed(&r,4555,100,100);
        for(unsigned t=100+periods[j];t<=200;t+=periods[j]) {
            assert(YawReference_Update(&r,4555,0,t,t,&dt));
            YawReference_Advance(&r,1,30,dt,1);
        }
        near(r.target_ticks,4555+3*8192.0f/360);
        float held=r.target_ticks;
        assert(YawReference_Update(&r,4555,0,200,200,&dt));
        YawReference_Advance(&r,1,30,dt,1);near(r.target_ticks,held);
    }
    YawReference_Seed(&r,4555,100,100);
    for(unsigned t=104;t<=2100;t+=4) {
        assert(YawReference_Update(&r,4555,0,t,t,&dt));
        YawReference_Advance(&r,1,30,dt,1);
    }
    near(r.target_ticks-r.position_ticks,30*2.0f*8192.0f/360);
    float held=r.target_ticks;
    /* 回中严格保持目标；反向输入直接减少连续目标。 */
    assert(YawReference_Update(&r,4355,0,2104,2104,&dt));
    YawReference_Advance(&r,0,30,dt,1);near(r.target_ticks,held);
    YawReference_Advance(&r,1,30,dt,1);assert(r.target_ticks>held);
    YawReference_Advance(&r,-1,30,dt,1);near(r.target_ticks,held);
    YawReference_Seed(&r,8190,100,100);
    assert(YawReference_Update(&r,2,0,104,104,&dt));near(r.position_ticks,8194);
    assert(YawReference_Update(&r,8190,0,108,108,&dt));near(r.position_ticks,8190);
    YawReference_Seed(&r,4555,100,100);
    for(unsigned n=1;n<=100;n++) {
        assert(YawReference_Update(&r,(4555+n*100)%8192,180,100+4*n,100+4*n,&dt));
        YawReference_Advance(&r,1,30,dt,1);
        near(r.target_ticks,4555 + 30.0f * 0.004f * n * 8192.0f / 360.0f);
        near(r.target_ticks-r.position_ticks,
             30.0f * 0.004f * n * 8192.0f / 360.0f - 100.0f*n);
    }
    near(YawReference_Wrap(r.position_ticks),(4555+10000)%8192);
    /* 新帧不能补救已丢失的长时间历史，失效后必须显式重新Seed。 */
    assert(!YawReference_Update(&r,r.last_raw,0,530,530,&dt));
    assert(!YawReference_Update(&r,r.last_raw,0,534,534,&dt));
    YawReference_Seed(&r,123,534,534);near(r.target_ticks,123);
    assert(!YawReference_Update(&r,4219,0,538,538,&dt));
    YawReference_Seed(&r,123,600,600);
    assert(!YawReference_Update(&r,123,0,600,621,&dt));
    YawReference_Seed(&r,123,600,600);
    assert(!YawReference_Update(&r,123,500,604,604,&dt));
    YawReference_Seed(&r,123,UINT32_MAX-2U,UINT32_MAX-2U);
    assert(YawReference_Update(&r,123,0,1,1,&dt));near(dt,0.004f);
    /* 重复同一毫秒的新反馈允许小位移，但不重复积分控制时间。 */
    assert(YawReference_Update(&r,125,0,1,1,&dt));near(dt,0);
    /* 二倍输入增益应使同一时间内的目标增量翻倍。 */
    YawReference_Seed(&r,4555,100,100);
    assert(YawReference_Update(&r,4555,0,104,104,&dt));
    YawReference_Advance(&r,0.5f,30,dt,2.0f);
    near(r.target_ticks,4555 + 30.0f * 0.004f * 2.0f * 0.5f * 8192.0f / 360.0f);
    puts("yaw reference: PASS (time, hold, input gain, unwrap, continuity loss)");
}
