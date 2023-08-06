#include <msp430.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
int data_buf0[128];
int data_buf1[128];

#include<math.h>
#define PI 3.14159265   //定义圆周率值
struct compx {float real,imag;};                        //定义一个复数结构struct compx s[128];
struct compx s1[128];
struct compx s[128];                                  //FFT输入和输出：从S[1]开始存放，根据大小自己定义
float result[128],result1[128];
int phase=0,phase1=0;
float phase_dif[20],phase_dif_aver,phase_plus;
unsigned char flag_phase=0;
unsigned int j=0;
float d;

struct compx EE(struct compx a,struct compx b)
{
    struct compx c;
    c.real=a.real*b.real-a.imag*b.imag;
    c.imag=a.real*b.imag+a.imag*b.real;
return(c);
}

void FFT(struct compx *xin)
{
    int f,m,nv2,nm1,i,k,l,j=0;
    struct compx u,w,t;
    nv2=128/2;
    //变址运算，即把自然顺序变成倒位序，采用雷德算法
    nm1=128-1;
    for(i=0;i<nm1;i++)
    {
        if(i<j) //如果i<j,即进行变址
        {
            t=xin[j];
            xin[j]=xin[i];
            xin[i]=t;
        }
        k=nv2;
        //求j的下一个倒位序
        while(k<=j) //如果k<=j,表示j的最高位为1
        {
            j=j-k; //把最高位变成0
            k=k/2; //k/2，比较次高位，依次类推，逐个比较，直到某个位为0
        }
        j=j+k; //把0改为1
    }
    {
        int le,lei,ip;//FFT运算核，使用蝶形运算完成FFT运算
        f=128;
        for(l=1;(f=f/2)!=1;l++) ;//计算l的值，即计算蝶形级数////////
            for(m=1;m<=l;m++) // 控制蝶形结级数
            {
                                    //m表示第m级蝶形，l为蝶形级总数l=log（2）N
                le=2<<(m-1);         //le蝶形结距离，即第m级蝶形的蝶形结相距le点
                lei=le/2; //同一蝶形结中参加运算的两点的距离
                u.real=1.0; //u为蝶形结运算系数，初始值为1
                u.imag=0.0;
                w.real=cos(PI/lei); //w为系数商，即当前系数与前一个系数的商
                w.imag=-sin(PI/lei);
                for(j=0;j<=lei-1;j++) //控制计算不同种蝶形结，即计算系数不同的蝶形结
                {
                    for(i=j;i<=128-1;i=i+le) //控制同一蝶形结运算，即计算系数相同蝶形结
                    {
                        ip=i+lei; //i，ip分别表示参加蝶形运算的两个节点
                        t=EE(xin[ip],u); //蝶形运算，详见公式
                        xin[ip].real=xin[i].real-t.real;
                        xin[ip].imag=xin[i].imag-t.imag;
                        xin[i].real=xin[i].real+t.real;
                        xin[i].imag=xin[i].imag+t.imag;
                    }
                    u=EE(u,w); //改变系数，进行下一个蝶形运算
                }
            }
    }
}
void upVcc(void)//核心电压上升
{
    PMMCTL0_H = 0xA5;
    SVSMLCTL |= SVSMLRRL_1 + SVMLE;
    PMMCTL0 = PMMPW +PMMCOREV_3;
    while((PMMIFG & SVSMLDLYIFG)==0);
    PMMIFG &=~ (SVMLVLRIFG + SVMLIFG + SVSMLDLYIFG);
    if((PMMIFG & SVMLIFG)==1)
        while((PMMIFG & SVMLVLRIFG)==0);
    SVSMLCTL &=~ SVMLE;
    PMMCTL0_H = 0x00;
}
void timerup(void)//配置时钟25MHZ
{

UCSCTL3 = SELREF_2;
UCSCTL4 |= SELA_2;
__bis_SR_register(SCG0);
UCSCTL0 = 0x0000;
UCSCTL1 = DCORSEL_7;//50Mhz范围
UCSCTL2 = FLLD_0 + 762;    //(762+1)*32768==25MHZ
__bic_SR_register(SCG0);

__delay_cycles(782000);

while(SFRIFG1 & OFIFG)
{
    UCSCTL7 &=~ (XT2OFFG + XT1LFOFFG + DCOFFG);
    SFRIFG1 &=~ OFIFG;

}
//UCSCTL4 = UCSCTL4&(~(SELS_7|SELM_7))|SELS_3|SELM_3;
}

void clock_init()
{

    P5SEL |= BIT2|BIT3; //将IO配置为XT2功能
    UCSCTL6 &= ~XT2OFF; //使能XT2

    UCSCTL4 = UCSCTL4&(~(SELA_7))|SELA_1; //先将ACLK配置为VLOCLK
    UCSCTL3 |= SELREF_2;                  //将REFCLK配置为REFCLK

    while (SFRIFG1 & OFIFG){
      UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);         // 清除三类时钟标志位
                                // 这里需要清除三种标志位，因为任何一种
                                // 标志位都会将OFIFG置位
      SFRIFG1 &= ~OFIFG;                                  // 清除时钟错误标志位
    }
    UCSCTL4 = UCSCTL4&(~(SELS_7))|SELS_5;     //将SMCLK时钟源配置为XT2
}
// 冒泡法排序函数
void BubbleSort(float pt[], int Cnt)
{
    int     k      = 0;
        float temp = 0;
    while (Cnt > 0)
    {
        for (k=0; k<Cnt-1; k++)
        {
            if (pt[k] < pt[k+1])
            {
                temp    = pt[k];
                pt[k]   = pt[k+1];
                pt[k+1] = temp;
            }
        }
        Cnt--;
    }
}

/**
 * main.c
 */
int main(void)
{
    unsigned int i;
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    upVcc();
    timerup();
    clock_init();
    P5SEL |= BIT7;              // P5.7/TB1 option select
    P5DIR |= BIT7;              // Output direction
    P6SEL |= 0x03;              // Enable A/D channel A0 A1

    //Setup Timer B0
    TBCCR0 = 128;
    TBCCR1 = 64;
    TBCCTL1 = OUTMOD_3;                       // CCR1 set/reset mode
    TBEX0 = TBIDEX_0;                         //CLK/1 divide
    TBCTL = TBSSEL_1+MC_1+TBCLR + ID_0;       // SMCLK, Up-Mode

    // Setup ADC12
    ADC12CTL0 = ADC12ON+ADC12MSC+ADC12SHT0_8;   // Turn on ADC12, extend sampling time
                                                // to avoid overflow of results
    ADC12CTL1 = ADC12SHS_3+ADC12CONSEQ_3;       // Use sampling timer, repeated sequence
    ADC12MCTL0 = ADC12INCH_0;                  // ref+=AVcc, channel = A0
    ADC12MCTL1 = ADC12INCH_1 + ADC12EOS;       // ref+=AVcc, channel = A1
    ADC12CTL0 |= ADC12ENC;                    // Enable conversions

    // Setup DMA COMMON
    DMACTL0 = DMA0TSEL_24 + DMA1TSEL_24;      // ADC12IFGx triggered
    DMACTL4 = DMARMWDIS;                      // Read-modify-write disable
    // Setup DMA0
    DMA0CTL &= ~DMAIFG;
    DMA0CTL = DMADT_4+DMAEN+DMADSTINCR_3;     // Rpt single tranfer, Destination address is incremented.
    DMA0SZ = 128;        // DMA0 size = 100

    __data20_write_long((uintptr_t) &DMA0SA,(uintptr_t) &ADC12MEM0); // Source block address
    __data20_write_long((uintptr_t) &DMA0DA,(uintptr_t) &data_buf0[0]);

    // Setup DMA1
    DMA1CTL &= ~DMAIFG;
    DMA1CTL = DMADT_4+DMAEN+DMADSTINCR_3; // Rpt single tranfer, Destination address is incremented.
    DMA1SZ = 128;       // DMA0 size = 100

    __data20_write_long((uintptr_t) &DMA1SA,(uintptr_t) &ADC12MEM1); // Source block address
    __data20_write_long((uintptr_t) &DMA1DA,(uintptr_t) &data_buf1[0]);
    while(data_buf0[127]==0 | data_buf1[127]==0);

    //float acosx=acos((s[1].real*s1[1].real+s[1].imag*s1[1].imag)/(s[1].real*s[1].real));
    while (1) {
        for(i=0;i<128;i++)
        {
            s[i].real=data_buf0[i];
            s[i].imag=0;
        }
        FFT(s);  //进行快速福利叶变换
        for(i=0;i<128;i++) //求变换后结果的模值，存入复数的实部部分
            if(i==0)
            result[0]=sqrt(s[i].real*s[i].real+s[i].imag*s[i].imag)/128;//幅值
            else
                result[i]=sqrt(s[i].real*s[i].real+s[i].imag*s[i].imag)*2/128;
        for(i=0;i<128;i++)
        {
            s1[i].real=data_buf0[i];
            s1[i].imag=0;
        }
        FFT(s1);  //进行快速福利叶变换
        for(i=0;i<128;i++) //求变换后结果的模值，存入复数的实部部分
            if(i==0)
            result1[0]=sqrt(s1[i].real*s1[i].real+s1[i].imag*s1[i].imag)/128;//幅值
            else
                result1[i]=sqrt(s1[i].real*s1[i].real+s1[i].imag*s1[i].imag)*2/128;
    /**************以下是相位差部分*************/
        for(i=0;i<128;i++)
        {
            if(data_buf0[i]>1638-100 && data_buf0[i]<1638+100) {
                    phase=i;
                    flag_phase=1;
            }
            if(data_buf1[i]>1638-100 && data_buf1[i]<1638+100) {
                phase1=i;
                if(abs(phase1-phase)<=64) {
                    //phase_dif[j]=(90-(abs(phase1-phase))/128.0*360);
                    phase_plus+=((abs(phase1-phase))/128.0*360-34.475);//10->44.29,40->15.6--默认0--->55°，40--->15.6°
                    j++;
                    if(j==20){ phase_dif_aver=phase_plus/j;j=0;phase_plus=0;d=tan(phase_dif_aver);}//相位差平均值
                }

            }
        }
    }
}

