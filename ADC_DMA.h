#include <msp430.h>
#include <stdint.h>

int data_buf0[128];
int data_buf1[128];
/*********************************************************************
快速福利叶变换C函数
函数简介：此函数是通用的快速傅里叶变换C
语言函数，移植性强，以下部分不依赖硬件。此函数采用联合体的形式表示一个复数，输入为自然顺序的复数（输入实数是可令复数虚部为0），输出为经过FFT变换的自然顺序的复数
使用说明：使用此函数只需更改宏定义128的值即可实现点数的改变，128的 应该为2的N次方，不满足此条件时应在后面补0
函数调用：FFT(s);
时间：2010-2-20
版本：Ver1.0 参考文献：
**********************************************************************/
#include<math.h>
#define PI 3.14159265   //定义圆周率值
struct compx {float real,imag;};                        //定义一个复数结构struct compx s[128];
struct compx s[128];                                  //FFT输入和输出：从S[1]开始存放，根据大小自己定义
float result[128],result1[128];
/*******************************************************************
函数原型：
struct compx EE(struct compx b1,struct compx b2)
函数功能：对两个复数进行乘法运算
输入参数：两个以联合体定义的复数a,b
输出参数：a和b的乘积，以联合体的形式输出
*******************************************************************/
struct compx EE(struct compx a,struct compx b)
{
    struct compx c;
    c.real=a.real*b.real-a.imag*b.imag;
    c.imag=a.real*b.imag+a.imag*b.real;
return(c);
}
/*****************************************************************
函数原型：void FFT(struct compx *xin,int N)
函数功能：对输入的复数组进行快速傅里叶变换（FFT）
输入参数：*xin复数结构体组的首地址指针，struct型
*****************************************************************/
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
    TBCTL = TBSSEL_2+MC_1+TBCLR + ID_0;       // SMCLK, Up-Mode

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
        s[i].real=data_buf0[i];
        s[i].imag=0;
    }
    FFT(s);  //进行快速福利叶变换
    for(i=0;i<128;i++) //求变换后结果的模值，存入复数的实部部分
        if(i==0)
        result1[0]=sqrt(s[i].real*s[i].real+s[i].imag*s[i].imag)/128;//幅值
        else
            result1[i]=sqrt(s[i].real*s[i].real+s[i].imag*s[i].imag)*2/128;
    while (1) {

    }
}

