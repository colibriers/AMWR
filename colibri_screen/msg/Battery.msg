int16 Chg_state
int8 SOC  # State of Charge:percent
float32 Voltage
int16 Bat_alarm
#struct Chg_state
#{
#uint16_t CING:1; //充电状态
#uint16_t DING:1; //放电状态
#uint16_t OCCSG:1; //充电过流
#uint16_t SHORT:1; //短路保护
#uint16_t OCDSG1:1; //放电过流 1 级
#uint16_t OCDSG2:1;//放电过流 2 级
#uint16_t wOCCSG:1; //充电电流警告值
#uint16_t wOCDSG:1; //放电电流警告值
#};
#Bat_alarm 数据结构如下
#struct _ALARM_{
#uint16_t bit0:1; //电压报警,压差保护,断线保护
#uint16_t bit1:1; //充电 fet 损坏报警
#uint16_t bit2:1; //SD ERR 1,error 0,normal
#uint16_t SPI_ERR:1;//ML5238 通信
#uint16_t E2PROM_ERR:1;#/外部存储器 E2PROM ERR 1,error 0,normal
#uint16_t bit5:1; //预留
#uint16_t FCC_UPDATING:1;//充电学习开启状态
#uint16_t FCC_DSGLEARN:1;//放电学习开启状态
#};
#
#
#
#
#
#
#
##
#
#
